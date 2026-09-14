"""Thread-safe desktop facade. Physics and robot commands share one worker."""
from __future__ import annotations

import copy
from collections import deque
import math
import queue
import threading
import time
from pathlib import Path

from PySide6.QtCore import QObject, Signal

from .bridge import build_workspace
from .config import default_config
from .runner import FixedStepClock, Session
from .replay_engine import ReplayController


class _LatestFrame:
    """One pixel packet, with generation-safe notification/acknowledgement.

    Only a small token enters Qt's event queue. A busy GUI consumes the newest
    packet for that token; an old source's token cannot acknowledge a new one.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._packet = None
        self._token = None
        self._serial = 0

    def publish(self, source, generation, pixels):
        with self._lock:
            self._packet = {"generation": generation, "pixels": pixels}
            if self._token is not None and (self._token["source"], self._token["generation"]) == (source, generation):
                return None
            self._serial += 1
            self._token = {"frame_token": self._serial, "source": source, "generation": generation}
            return self._token.copy()

    def take(self, token):
        with self._lock:
            if token != self._token:
                return None
            packet = self._packet
            self._packet = self._token = None
            return packet


class _RenderCadence:
    """Completed-render cadence over the most recent second, in wall time."""

    def __init__(self):
        self.times = deque(maxlen=241)

    def reset(self):
        self.times.clear()

    def record(self, now):
        self.times.append(now)

    def fps(self, now):
        while self.times and self.times[0] < now - 1.0:
            self.times.popleft()
        if len(self.times) < 2 or self.times[-1] <= self.times[0]:
            return 0.0
        return (len(self.times) - 1) / (self.times[-1] - self.times[0])


def _reuse_configuration(previous, incoming, *, reuse_unchanged=False):
    """Only presentation settings may change without replacing a live world.

    Camera commands already update the view independently. Explicit Apply of
    unchanged settings still reloads external assets; Start opts into reuse.
    Session/world config stays untouched, including a replay's pinned settings.
    """
    presentation = {"render_fps", "speed", "camera"}
    changed = {key for key in previous.keys() | incoming.keys()
               if previous.get(key) != incoming.get(key)}
    if changed - presentation:
        return False
    if not reuse_unchanged and not changed.intersection({"render_fps", "speed"}):
        return False
    for key, default in (("render_fps", 30), ("speed", 1)):
        value = float(incoming.get(key, default))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be positive and finite")
    return True


class SimulationEngine(QObject):
    frame = Signal(object)
    state = Signal(dict)
    replay_frame = Signal(object)
    replay_state = Signal(dict)
    log = Signal(str)
    built = Signal(str)
    reset_completed = Signal(dict)
    failed = Signal(str)

    def __init__(self, defer_configuration: bool = False, coalesce_frames: bool = False):
        super().__init__()
        self._defer_configuration = defer_configuration
        self._latest_frame = _LatestFrame() if coalesce_frames else None
        self._queue: queue.Queue = queue.Queue()
        self._closing = threading.Event()
        self._replay_seek_lock = threading.Lock()
        self._replay_seek_pending = False
        self._replay_seek_value = 0.0
        self._thread = threading.Thread(target=self._work, name="EERKit physics", daemon=True)
        self._thread.start()

    def _send(self, action, *args):
        if not self._closing.is_set():
            self._queue.put((action, args))

    def configure(self, config: dict, *, reuse_unchanged: bool = False):
        self._send("configure", copy.deepcopy(config), bool(reuse_unchanged))

    def take_frame(self, token):
        return self._latest_frame.take(token) if self._latest_frame is not None else None

    def _emit_frame(self, source, generation, pixels):
        signal = self.replay_frame if source == "replay" else self.frame
        if self._latest_frame is None:
            signal.emit({"generation": generation, "pixels": pixels})
        else:
            token = self._latest_frame.publish(source, generation, pixels)
            if token is not None:
                signal.emit(token)

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

    def replay_enter(self):
        self._send("replay_enter")

    def replay_live(self):
        self._send("replay_live")

    def replay_play(self):
        self._send("replay_play")

    def replay_pause(self):
        self._send("replay_pause")

    def replay_seek(self, seconds):
        # A drag retains one queue marker and the newest requested position.
        with self._replay_seek_lock:
            self._replay_seek_value = float(seconds)
            if not self._replay_seek_pending and not self._closing.is_set():
                self._replay_seek_pending = True
                self._queue.put(("replay_seek", ()))

    def _take_replay_seek(self):
        with self._replay_seek_lock:
            self._replay_seek_pending = False
            return self._replay_seek_value

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
        # Native scene reset also invalidates its renderer. Acknowledge only
        # after that new live view has actually produced its first frame.
        pending_reset_generation = None
        cadence = _RenderCadence()
        replay = ReplayController(self.log.emit)
        replay_wall = last_wall
        last_replay_state = None
        # Keep ownership when native process/world cleanup fails. A following
        # Apply retries it before creating another replacement scene.
        self._retired_sessions = []

        def retry_retired_sessions():
            for previous in tuple(self._retired_sessions):
                try:
                    previous.close()
                except Exception as exc:
                    self.log.emit(f"Scene cleanup still pending: {exc}")
                else:
                    self._retired_sessions.remove(previous)

        def retire_session(previous):
            if not any(previous is item for item in self._retired_sessions):
                self._retired_sessions.append(previous)
            retry_retired_sessions()

        def replay_status():
            nonlocal last_replay_state
            status = replay.metadata()
            if not replay.active:
                status["camera"] = copy.deepcopy(camera)
            if status != last_replay_state:
                self.replay_state.emit(status)
                last_replay_state = status

        def state_payload(**values):
            return {**session.state(), "viewport_fps": cadence.fps(time.perf_counter()), **values}

        def complete_step():
            nonlocal last_state
            was_running = bool(session.running)
            observed = session.step()
            replay.capture(session, observed)
            if not session.running:
                replay.finish(session, "completed", observed)
                if was_running:
                    # Completion is a lifecycle transition, even between periodic
                    # telemetry samples. Publish it before rendering clears dirty.
                    last_state = 0.0
            return observed

        try:
            if not self._defer_configuration:
                session = Session(config, self.log.emit)
                scene_valid = True
            replay_status()
            while not self._closing.is_set():
                try:
                    for _ in range(64):
                        try:
                            action, args = self._queue.get_nowait()
                        except queue.Empty:
                            break
                        if action == "close":
                            return
                        reuse_configuration = False
                        if action == "configure" and scene_valid:
                            try:
                                reuse_configuration = _reuse_configuration(
                                    config, args[0], reuse_unchanged=bool(args[1]) if len(args) > 1 else False)
                            except Exception:
                                # Failed lightweight validation must also block
                                # a Start command already queued behind Apply.
                                scene_valid = False
                                raise
                        if reuse_configuration:
                            # Engine-only presentation state: no reset, stop, lost
                            # backlog, or recorder invalidation on an FPS change.
                            config = args[0]
                            self.log.emit("Scene configuration applied")
                            self.state.emit(state_payload(real_time_factor=rtf, physics_backlog_s=clock.pending))
                            dirty = True
                            continue
                        if action.startswith("replay_"):
                            try:
                                if action == "replay_enter":
                                    if session is not None:
                                        replay.enter(session, config.get("renderer", "auto"))
                                    else:
                                        replay.changed_view()
                                        replay.reason = "Apply scene settings before opening replay"
                                elif action == "replay_live":
                                    replay.leave()
                                elif action == "replay_seek":
                                    replay.seek(self._take_replay_seek())
                                elif action == "replay_play":
                                    replay.play()
                                elif action == "replay_pause":
                                    replay.pause()
                            except Exception as exc:
                                # A presentation error must not disable a paused
                                # live robot or turn into a simulation step.
                                replay.pause()
                                replay.reason = str(exc)
                                self.log.emit(f"Replay: {exc}")
                            clock.pending = 0
                            replay_wall = last_wall = time.perf_counter()
                            measured_wall, measured_steps, rtf = last_wall, 0, 0.0
                            last_state = last_render = 0.0
                            cadence.reset()
                            dirty = True
                            if session is not None and scene_valid and action in ("replay_enter", "replay_live"):
                                self.state.emit(state_payload(real_time_factor=0., physics_backlog_s=0.))
                            replay_status()
                            continue
                        if replay.active and action in ("controller", "joint", "step", "resume", "stop"):
                            # Guard queued input as well as the GUI controls.
                            continue
                        if action in ("configure", "build", "start", "reset"):
                            replay.leave()
                            replay.discard()
                        if action == "configure":
                            # One old failed cleanup may remain owned after a
                            # replacement. Never accumulate more native scenes.
                            retry_retired_sessions()
                            if self._retired_sessions:
                                raise RuntimeError("Previous scene cleanup is still pending. Retry Apply after its native worker stops; another scene was not created.")
                            scene_valid = False
                            replacement = Session(args[0], self.log.emit)
                            previous, session = session, replacement
                            if previous is not None:
                                retire_session(previous)
                            config = args[0]
                            clock = FixedStepClock(config.get("physics_hz", 240))
                            last_renderer = None
                            scene_valid = True
                            self.log.emit("Scene configuration applied")
                        elif action == "camera":
                            values = dict(zip(("yaw", "pitch", "distance", "target"), args))
                            if replay.active:
                                replay.set_camera(values)
                            else:
                                camera = values
                        elif action == "controller":
                            if session is not None:
                                session.controller = args[0]
                        elif session is None:
                            raise ValueError("Apply valid scene settings before using simulation controls")
                        elif not scene_valid and action in ("step", "joint", "resume"):
                            raise ValueError("Apply scene settings or retry Reset before using simulation controls")
                        elif action == "build":
                            session.stop()
                            executable = None
                            session.executable = None
                            executable = build_workspace(Path(args[0]), self.log.emit)
                            self.built.emit(str(executable))
                        elif action == "start":
                            if not scene_valid:
                                raise ValueError("Fix and apply the scene settings before starting")
                            scene_valid = False
                            session.start(args[0], executable)
                            scene_valid = True
                            replay.begin(session, camera)
                            last_renderer = None
                            clock.pending = 0
                        elif action == "pause":
                            session.paused = True
                            replay.pause()
                            clock.pending = 0
                        elif action == "resume":
                            session.paused = False
                            clock.pending = 0
                        elif action == "stop":
                            session.stop()
                            replay.finish(session, "stopped")
                            clock.pending = 0
                        elif action == "reset":
                            scene_valid = False
                            session.reset()
                            scene_valid = True
                            last_renderer = None
                            clock.pending = 0
                            pending_reset_generation = replay.generation
                        elif action == "step":
                            session.paused = True
                            complete_step()
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
                                cadence.reset()
                            # Lifecycle changes must reach the GUI even when a
                            # render clears dirty before the next telemetry interval.
                            last_state = 0.0
                        dirty = True
                        replay_status()

                    if session is None:
                        self._closing.wait(.015)
                        continue
                    now = time.perf_counter()
                    if replay.active:
                        was_playing = bool(replay.cursor and replay.cursor.playing)
                        replay.advance(max(0., now - replay_wall))
                        replay_wall = now
                        clock.pending = 0
                        last_wall = now
                        playing = bool(replay.cursor and replay.cursor.playing)
                        dirty = dirty or (was_playing and not playing)
                        render_interval = 1.0 / max(1., float(config.get("render_fps", 30)))
                        if (dirty or playing) and now - last_render >= render_interval:
                            last_render = time.perf_counter()
                            pixels = replay.render(960, 720)
                            replay_status()
                            if pixels is not None:
                                cadence.record(time.perf_counter())
                                self._emit_frame("replay", replay.generation, pixels)
                            dirty = not replay.active
                            if not replay.active:
                                last_render = 0.
                                if scene_valid:
                                    self.state.emit(state_payload(real_time_factor=0., physics_backlog_s=0.))
                        else:
                            replay_status()
                        self._closing.wait(.001 if playing else .015)
                        continue
                    if not scene_valid:
                        self._closing.wait(.015)
                        continue
                    render_interval = 1.0 / max(1.0, float(config.get("render_fps", 30)))
                    active = (session.running or bool(session.manual_joints)) and not session.paused
                    if active:
                        clock.add(now - last_wall, float(config.get("speed", 1.0)))
                        count = clock.take(16)
                        # Physics remains ordered and unskipped. End the batch
                        # when the viewport is due, but always complete at least
                        # one due tick so a slow renderer cannot starve physics.
                        work_deadline = min(time.perf_counter() + .020, last_render + render_interval)
                        for index in range(count):
                            complete_step()
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
                    if dirty and completed_at - last_state >= 0.05:
                        replay_status()
                        self.state.emit(state_payload(real_time_factor=rtf, physics_backlog_s=clock.pending))
                        last_state = completed_at
                    render_now = time.perf_counter()
                    # Backlog must not impose a four-FPS viewport cap. Render
                    # deadlines use completed work, and start-to-start cadence
                    # does not add the full render cost to every frame period.
                    if dirty and render_now - last_render >= render_interval:
                        last_render = render_now
                        pixels = session.world.render(960, 720, **camera)
                        if pending_reset_generation is not None and pixels is None:
                            raise RuntimeError("The reset scene renderer did not return a frame")
                        cadence.record(time.perf_counter())
                        replay_status()
                        self._emit_frame("live", replay.generation, pixels)
                        renderer = getattr(session.world, "renderer_status", {})
                        if renderer != last_renderer:
                            last_renderer = copy.deepcopy(renderer)
                            self.log.emit(f"Renderer: {renderer.get('device', 'unknown')} · {renderer.get('reason', '')}")
                            self.state.emit(state_payload(real_time_factor=rtf, physics_backlog_s=clock.pending))
                        if pending_reset_generation == replay.generation:
                            self.reset_completed.emit(state_payload(real_time_factor=0., physics_backlog_s=0.))
                            pending_reset_generation = None
                        dirty = False
                    self._closing.wait(0.001 if active else 0.015)
                except Exception as exc:
                    if pending_reset_generation is not None:
                        scene_valid = False
                    pending_reset_generation = None
                    clock.pending = 0
                    self.failed.emit(str(exc))
                    self.log.emit(f"Error: {exc}")
                    if session is not None:
                        try:
                            session.stop()
                        except Exception as cleanup_error:
                            scene_valid = False
                            self.log.emit(f"Scene disable after failure: {cleanup_error}")
                        try:
                            replay.finish(session, "failed")
                        except Exception as replay_error:
                            self.log.emit(f"Replay finalization after failure: {replay_error}")
                        if scene_valid:
                            try:
                                self.state.emit(state_payload(real_time_factor=0.0))
                            except Exception as state_error:
                                scene_valid = False
                                self.log.emit(f"Scene observation after failure: {state_error}")
                    if not scene_valid:
                        self.log.emit("Scene is unavailable. Apply scene settings to recover, or retry Reset.")
                    replay_status()
        except Exception as exc:
            self.failed.emit(str(exc))
            self.log.emit(f"Simulator startup failed: {exc}")
        finally:
            try:
                replay.close()
            finally:
                if session is not None:
                    retire_session(session)
                else:
                    retry_retired_sessions()
