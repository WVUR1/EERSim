"""Independent fixed-base URDF workbench for live motor binding checks.

All Bullet calls stay on one dedicated worker. Joint movement uses World and its
cartridge torque model, exactly as the main simulator does. Rendering never
changes the fixed 240 Hz integration step.
"""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path
import queue
import tempfile
import threading
import time
from typing import Any

from PySide6.QtCore import QObject, Signal

from .config import default_config
from .runner import FixedStepClock


class UrdfPreview(QObject):
    """Thread-safe QObject facade for a robot import dialog.

    set_motor uses positive physical smart ports and electrical millivolts.
    Reversing a binding therefore reverses the visible joint response. It does
    not apply a second PROS software reversal. The robot base is fixed in space;
    gravity, joint limits, inertia, gearing and motor torque remain simulated.
    """

    frame = Signal(object)
    state = Signal(dict)
    failed = Signal(str)

    def __init__(self, parent: QObject | None = None):
        super().__init__(parent)
        self._queue: queue.Queue[tuple[str, tuple[Any, ...]]] = queue.Queue()
        self._closing = threading.Event()
        self._thread = threading.Thread(target=self._work, daemon=True, name="EERKit URDF preview")
        self._thread.start()

    def _send(self, action: str, *args: Any) -> None:
        if not self._closing.is_set():
            self._queue.put((action, args))

    def configure(self, urdf: str, motors: list[dict]) -> None:
        """Replace the preview robot and clear all previous motor commands."""
        self._send("configure", str(urdf), deepcopy(motors))

    def set_motor(self, port: int, voltage: int) -> None:
        """Jog a positive physical port at a clamped -12000..12000 mV."""
        self._send("motor", int(port), int(voltage))

    def stop_all(self) -> None:
        """Remove power from all preview motors, preserving joint positions."""
        self._send("stop")

    def set_camera(self, yaw: float, pitch: float, distance: float,
                   target: list[float] | None = None) -> None:
        self._send("camera", float(yaw), float(pitch), float(distance), deepcopy(target))

    def close(self) -> None:
        """Stop emissions immediately; allow at most three seconds for teardown."""
        if self._closing.is_set():
            return
        self._closing.set()
        self._queue.put(("close", ()))
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=3)

    def _emit_state(self, world, camera: dict, bounds: list, bindings: list,
                    revision: int, commands: dict, backlog: float) -> None:
        if not self._closing.is_set():
            self.state.emit({**world.snapshot(), "ready": True, "fixed_base": True,
                             "renderer": dict(world.renderer_status),
                             "camera": deepcopy(camera), "bounds": deepcopy(bounds),
                             "bindings": deepcopy(bindings), "revision": revision,
                             "motor_commands": deepcopy(commands),
                             "running": any(command.get("value", 0) != 0 for command in commands.values()),
                             "physics_backlog_s": backlog})

    def _work(self) -> None:
        world = None
        temporary = None
        camera = {"yaw": 35.0, "pitch": -28.0, "distance": 1.3, "target": [0.0, 0.0, 0.15]}
        bounds: list[list[float]] = []
        bindings: list[dict] = []
        commands: dict[str, dict] = {}
        revision = 0
        ready = False
        clock = FixedStepClock(240)
        last_wall = time.perf_counter()
        last_frame = last_state = 0.0
        dirty = False
        try:
            # Imported lazily so a missing physics dependency is reported through
            # the GUI signal rather than breaking the import dialog's module.
            import pybullet as p
            from .physics import World

            temporary = tempfile.TemporaryDirectory(prefix="eersim-preview-")
            field = Path(temporary.name) / "preview-field.json"
            field.write_text(json.dumps({
                "schema_version": 1, "name": "URDF workbench",
                "field": {"size": [3.6576, 3.6576], "walls": False, "floor": False},
                "objects": [],
            }), encoding="utf-8")
            while not self._closing.is_set():
                try:
                    for _ in range(32):
                        try:
                            action, args = self._queue.get_nowait()
                        except queue.Empty:
                            break
                        if action == "close":
                            return
                        if action == "configure":
                            ready = False
                            commands.clear()
                            if world is not None:
                                world.disable()
                                world.close()
                                world = None
                            config = default_config()
                            config.update(robot_urdf=str(Path(args[0]).expanduser().resolve()),
                                          robot_config="", motors=args[1], motor_overrides={},
                                          game_pack=str(field), fixed_base=True,
                                          start_pose=[0.0, 0.0, 0.0, 0.0], physics_hz=240)
                            world = World(config)
                            bindings = deepcopy(args[1])
                            commands = {str(motor["port"]): {"mode": "voltage", "value": 0, "brake": "coast"}
                                        for motor in world.motor_bindings}
                            boxes = [p.getAABB(world.robot, link, physicsClientId=world.client)
                                     for link in range(-1, p.getNumJoints(world.robot, physicsClientId=world.client))]
                            lower = [min(box[0][axis] for box in boxes) for axis in range(3)]
                            upper = [max(box[1][axis] for box in boxes) for axis in range(3)]
                            bounds = [lower, upper]
                            center = [(low + high) / 2 for low, high in zip(lower, upper)]
                            radius = math.sqrt(sum((high - low) ** 2 for low, high in zip(lower, upper))) / 2
                            camera = {"yaw": 35.0, "pitch": -28.0,
                                      "distance": max(.45, min(20.0, radius / math.sin(math.radians(26)) * 1.3)),
                                      "target": center}
                            revision += 1
                            ready = True
                            clock.pending = 0.0
                            last_wall = time.perf_counter()
                            last_frame = last_state = 0.0
                            self._emit_state(world, camera, bounds, bindings, revision, commands, clock.pending)
                        elif action == "motor":
                            if world is None or not ready:
                                raise ValueError("Load a valid URDF and motor binding before jogging a motor")
                            port, voltage = args
                            if not 1 <= port <= 21:
                                raise ValueError("Preview motor ports must be positive physical smart ports from 1 to 21")
                            if str(port) not in commands:
                                raise ValueError(f"Preview motor port {port} has no joint binding")
                            commands[str(port)] = {"mode": "voltage", "value": max(-12000, min(12000, voltage)), "brake": "coast"}
                        elif action == "stop":
                            if world is not None:
                                world.disable()
                            for command in commands.values():
                                command["value"] = 0
                            clock.pending = 0.0
                            last_wall = time.perf_counter()
                        elif action == "camera":
                            yaw, pitch, distance, target = args
                            if not all(math.isfinite(value) for value in (yaw, pitch, distance)) or distance <= 0:
                                raise ValueError("Preview camera values must be finite, with positive distance")
                            if target is not None:
                                if not isinstance(target, (list, tuple)) or len(target) != 3 or not all(math.isfinite(float(v)) for v in target):
                                    raise ValueError("Preview camera target must contain finite x, y, z")
                                camera["target"] = [float(v) for v in target]
                            camera.update(yaw=yaw, pitch=max(-89, min(89, pitch)), distance=max(.1, min(20, distance)))
                        dirty = True
                    now = time.perf_counter()
                    if world is not None and ready:
                        clock.add(now - last_wall)
                        for _ in range(clock.take(16)):
                            if self._closing.is_set():
                                return
                            world.step({"motors": commands})
                            dirty = True
                        if dirty and now - last_state >= .1:
                            self._emit_state(world, camera, bounds, bindings, revision, commands, clock.pending)
                            last_state = now
                        # Retain every physics tick when rendering falls behind.
                        # Throttle only pictures, down to 4 Hz during heavy load.
                        if dirty and now - last_frame >= .05 and (clock.pending < .1 or now - last_frame > .25):
                            pixels = world.render(640, 480, **camera)
                            if not self._closing.is_set():
                                self.frame.emit(pixels)
                            last_frame = time.perf_counter()
                            dirty = False
                    else:
                        clock.pending = 0.0
                    last_wall = now
                    self._closing.wait(.001 if ready else .015)
                except Exception as exc:
                    if world is not None:
                        world.disable()
                    for command in commands.values():
                        command["value"] = 0
                    ready = False
                    clock.pending = 0.0
                    last_wall = time.perf_counter()
                    if not self._closing.is_set():
                        self.failed.emit(str(exc))
                        self.state.emit({"ready": False, "fixed_base": True, "running": False,
                                         "revision": revision, "camera": deepcopy(camera),
                                         "motor_commands": deepcopy(commands), "error": str(exc)})
        except Exception as exc:
            if not self._closing.is_set():
                self.failed.emit(f"Could not start URDF preview: {exc}")
        finally:
            if world is not None:
                world.close()
            if temporary is not None:
                temporary.cleanup()
