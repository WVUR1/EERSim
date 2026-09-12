"""Thread-safe desktop facade. Physics and robot commands share one worker."""
from __future__ import annotations

import copy
import queue
import threading
import time
from pathlib import Path

from PySide6.QtCore import QObject, Signal

from .bridge import build_workspace
from .config import default_config
from .runner import FixedStepClock, Session


class SimulationEngine(QObject):
    frame = Signal(object)
    state = Signal(dict)
    log = Signal(str)
    built = Signal(str)
    failed = Signal(str)

    def __init__(self, defer_configuration: bool = False):
        super().__init__()
        self._defer_configuration = defer_configuration
        self._queue: queue.Queue = queue.Queue()
        self._closing = threading.Event()
        self._thread = threading.Thread(target=self._work, name="EERKit physics", daemon=True)
        self._thread.start()

    def _send(self, action, *args):
        if not self._closing.is_set():
            self._queue.put((action, args))

    def configure(self, config: dict):
        self._send("configure", copy.deepcopy(config))

    def build(self, workspace: str):
        self._send("build", workspace)

    def start(self, mode: str):
        self._send("start", mode)

    def pause(self):
        self._send("pause")

    def resume(self):
        self._send("resume")

    def stop(self):
        self._send("stop")

    def reset(self):
        self._send("reset")

    def single_step(self):
        self._send("step")

    def set_controller(self, controller: dict):
        self._send("controller", copy.deepcopy(controller))

    def set_camera(self, yaw=-45.0, pitch=-55.0, distance=5.0, target=None):
        self._send("camera", float(yaw), float(pitch), float(distance), copy.deepcopy(target))

    def set_joint(self, joint: str, position: float):
        self._send("joint", joint, float(position))

    def export_csv(self, path: str):
        self._send("export", path)

    def close(self):
        self._closing.set()
        self._queue.put(("close", ()))
        # Runtime watchdog bounds a stuck robot. This join keeps Bullet teardown
        # off the Qt thread; no QThread is destroyed while still running.
        self._thread.join(timeout=7)

    def _work(self):
        session = None
        executable = None
        scene_valid = False
        camera = {"yaw": -45, "pitch": -55, "distance": 5.0}
        config = default_config()
        clock = FixedStepClock(config.get("physics_hz", 240))
        last_wall = time.perf_counter()
        last_render = last_state = 0.0
        measured_wall, measured_steps, rtf = last_wall, 0, 0.0
        dirty = True
        last_renderer = None
        try:
            if not self._defer_configuration:
                session = Session(config, self.log.emit)
                scene_valid = True
            while not self._closing.is_set():
                try:
                    for _ in range(64):
                        try:
                            action, args = self._queue.get_nowait()
                        except queue.Empty:
                            break
                        if action == "close":
                            return
                        if action == "configure":
                            # Construct successfully before discarding the previous scene.
                            scene_valid = False
                            replacement = Session(args[0], self.log.emit)
                            if session is not None:
                                session.close()
                            session = replacement
                            config = args[0]
                            clock = FixedStepClock(config.get("physics_hz", 240))
                            last_renderer = None
                            scene_valid = True
                            self.log.emit("Scene configuration applied")
                        elif action == "camera":
                            camera = dict(zip(("yaw", "pitch", "distance", "target"), args))
                        elif action == "controller":
                            if session is not None:
                                session.controller = args[0]
                        elif session is None:
                            raise ValueError("Apply valid scene settings before using simulation controls")
                        elif action == "build":
                            session.stop()
                            executable = None
                            session.executable = None
                            executable = build_workspace(Path(args[0]), self.log.emit)
                            self.built.emit(str(executable))
                        elif action == "start":
                            if not scene_valid:
                                raise ValueError("Fix and apply the scene settings before starting")
                            session.start(args[0], executable)
                            last_renderer = None
                            clock.pending = 0
                        elif action == "pause":
                            session.paused = True
                            clock.pending = 0
                        elif action == "resume":
                            session.paused = False
                            clock.pending = 0
                        elif action == "stop":
                            session.stop()
                            clock.pending = 0
                        elif action == "reset":
                            session.reset()
                            last_renderer = None
                            clock.pending = 0
                        elif action == "step":
                            session.paused = True
                            session.step()
                            measured_steps += 1
                        elif action == "joint":
                            if session.running:
                                raise ValueError("Stop robot code before manually commanding joints")
                            session.manual_joints[args[0]] = args[1]
                            # A manual target is stepped continuously until paused/reset.
                            session.paused = False
                        elif action == "export":
                            session.export_csv(args[0])
                            self.log.emit(f"Saved {len(session.samples)} telemetry samples to {args[0]}")
                        if action not in ("controller", "camera"):
                            last_wall = time.perf_counter()
                            if action in ("configure", "build", "start", "pause", "resume", "stop", "reset"):
                                # A new run must not inherit old ticks or include
                                # scene loading, compilation, or paused wall time.
                                measured_wall, measured_steps, rtf = last_wall, 0, 0.0
                            # Lifecycle changes must reach the GUI even when a
                            # render clears dirty before the next telemetry interval.
                            last_state = 0.0
                        dirty = True

                    if session is None:
                        self._closing.wait(.015)
                        continue
                    now = time.perf_counter()
                    active = (session.running or bool(session.manual_joints)) and not session.paused
                    if active:
                        clock.add(now - last_wall, float(config.get("speed", 1.0)))
                        count = clock.take(16)
                        work_deadline = time.perf_counter() + .020
                        for index in range(count):
                            session.step()
                            measured_steps += 1
                            if not session.running and not session.manual_joints:
                                break
                            if time.perf_counter() >= work_deadline:
                                # Return unprocessed ticks to the accumulator so
                                # heavy contact work still lets camera/controller
                                # commands through. No physics time is discarded.
                                clock.pending += (count - index - 1) * session.dt
                                break
                        dirty = dirty or count > 0
                    else:
                        clock.pending = 0
                    last_wall = now
                    # The accumulator keeps the pre-work timestamp so the next
                    # pass includes this work in its backlog. Throughput instead
                    # observes wall time after the counted ticks have completed.
                    completed_at = time.perf_counter()
                    elapsed = completed_at - measured_wall
                    if elapsed >= 0.5:
                        rtf = measured_steps * session.dt / elapsed
                        measured_steps, measured_wall = 0, completed_at
                    if dirty and now - last_state >= 0.05:
                        self.state.emit({**session.state(), "real_time_factor": rtf,
                                         "physics_backlog_s": clock.pending})
                        last_state = now
                    render_interval = 1.0 / max(1.0, float(config.get("render_fps", 30)))
                    # Keep all physics steps when overloaded and lower rendering frequency.
                    if dirty and now - last_render >= render_interval and (clock.pending < 0.1 or now - last_render > 0.25):
                        self.frame.emit(session.world.render(960, 720, **camera))
                        renderer = getattr(session.world, "renderer_status", {})
                        if renderer != last_renderer:
                            last_renderer = copy.deepcopy(renderer)
                            self.log.emit(f"Renderer: {renderer.get('device', 'unknown')} · {renderer.get('reason', '')}")
                            self.state.emit({**session.state(), "real_time_factor": rtf,
                                             "physics_backlog_s": clock.pending})
                        last_render = time.perf_counter()
                        dirty = False
                    self._closing.wait(0.001 if active else 0.015)
                except Exception as exc:
                    if session is not None:
                        session.stop()
                    clock.pending = 0
                    self.failed.emit(str(exc))
                    self.log.emit(f"Error: {exc}")
                    if session is not None:
                        self.state.emit({**session.state(), "real_time_factor": 0.0})
        except Exception as exc:
            self.failed.emit(str(exc))
            self.log.emit(f"Simulator startup failed: {exc}")
        finally:
            if session is not None:
                session.close()
