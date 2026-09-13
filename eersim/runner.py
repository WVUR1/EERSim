"""Headless session orchestration, shared by the desktop and batch runner.

Robot code and the selected physics engine advance in lockstep. Rendering never supplies a physics
dt; every step represents exactly 1 / physics_hz simulated seconds.
"""
from __future__ import annotations

import copy
import csv
import math
from pathlib import Path
from typing import Callable

from .performance import PerformanceRecorder

MODES = {
    "skills_driver": ("Skills Driver", "opcontrol", 60.0),
    "skills_auto": ("Skills Auto", "autonomous", 60.0),
    "match_auto": ("Match Auto", "autonomous", 30.0),
    "driver_auto": ("Driver Auto", "opcontrol", 90.0),
}


class FixedStepClock:
    """Wall-time pacing only. Backlog is retained, never discarded or enlarged."""

    def __init__(self, hz: float):
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("Physics frequency must be positive and finite")
        self.dt = 1.0 / hz
        self.pending = 0.0

    def add(self, elapsed: float, speed: float = 1.0) -> None:
        if not math.isfinite(elapsed) or elapsed < 0 or not math.isfinite(speed) or speed <= 0:
            raise ValueError("Elapsed time and speed must be finite and nonnegative (speed > 0)")
        self.pending += elapsed * speed

    def take(self, max_steps: int = 16) -> int:
        count = min(max_steps, int((self.pending + self.dt * 1e-9) / self.dt))
        self.pending = max(0.0, self.pending - count * self.dt)
        return count


class Session:
    """One robot process and one world; all methods belong to one worker thread."""

    def __init__(self, config: dict, log: Callable[[str], None] | None = None,
                 world_factory=None, runtime_factory=None):
        if world_factory is None:
            from .physics_backend import create_world
            world_factory = create_world
        if runtime_factory is None:
            from .bridge import NativeRuntime
            runtime_factory = NativeRuntime
        self.config = copy.deepcopy(config)
        self.log = log or (lambda _: None)
        self.performance = PerformanceRecorder()
        with self.performance.span("session.setup.world"):
            self.world = world_factory(self.config)
        # A configured, unstepped preview is already the initial scene.
        # PhysX may include its required bootstrap tick in this state.
        self._world_pristine = True
        self._prepared_config = copy.deepcopy(self.config)
        physics = getattr(self.world, "physics_status", {})
        if physics:
            self.log(f"Physics: {physics.get('backend', 'unknown')} · {physics.get('device', 'unknown')}"
                     + (f" · {physics['reason']}" if physics.get("reason") else ""))
        self.runtime_factory = runtime_factory
        self.runtime = None
        self.executable: Path | None = None
        self.dt = 1.0 / self.config.get("physics_hz", 240)
        self.mode = "disabled"
        self.phase = "disabled"
        self.running = False
        self.paused = False
        self.initialized = False
        self.elapsed_ticks = 0
        self.init_ticks = 0
        self.duration = 0.0
        self.controller = {"analog": {}, "digital": {}}
        self.manual_joints: dict[str, float] = {}
        self.samples: list[dict] = []
        self.dropped_samples = 0
        self._last_lcd = None
        self._world_warnings: set[str] = set()

    def start(self, mode: str, executable: Path | None = None) -> None:
        if mode not in MODES:
            raise ValueError(f"Unknown competition mode: {mode}")
        if executable is not None:
            self.executable = Path(executable)
        if self.executable is None:
            raise RuntimeError("Select and build a PROS workspace before starting")
        reuse_prepared = self._world_pristine and self.config == self._prepared_config
        # Consume before callbacks/runtime creation: a failed start must reset next time.
        self._world_pristine = False
        self.stop()
        if not reuse_prepared:
            with self.performance.span("session.setup.reset"):
                self.world.reset(self.config)
            self._prepared_config = copy.deepcopy(self.config)
        with self.performance.span("session.setup.native"):
            self.runtime = self.runtime_factory(self.executable)
        self.mode, self.phase = mode, "initialize"
        self.running, self.paused, self.initialized = True, False, False
        self.elapsed_ticks = self.init_ticks = 0
        game_durations = getattr(self.world, "game", {}).get("mode_durations", {})
        self.duration = float(self.config.get("duration_overrides", {}).get(
            mode, game_durations.get(mode, MODES[mode][2])))
        if not math.isfinite(self.duration) or self.duration <= 0:
            self.stop()
            raise ValueError("Mode duration must be positive and finite")
        self.samples.clear()
        self.dropped_samples = 0
        self._last_lcd = None
        self._world_warnings.clear()
        self.log(f"Initializing robot for {MODES[mode][0]} ({self.duration:g} simulated seconds)")

    def stop(self) -> None:
        if self.runtime is not None:
            try:
                with self.performance.span("session.native.disable_roundtrip"):
                    self.runtime.step(self.world.snapshot(), "disabled", self.dt, {"analog": {}, "digital": {}})
            except Exception as exc:
                self.log(f"Disable callback: {exc}")
            finally:
                with self.performance.span("session.native.close"):
                    self.runtime.close()
                self.runtime = None
        self.running, self.paused = False, False
        self.phase = "disabled"
        self.manual_joints.clear()
        self.controller = {"analog": {}, "digital": {}}
        if hasattr(self.world, "disable"):
            self.world.disable()

    def reset(self) -> None:
        self._world_pristine = False
        self.stop()
        with self.performance.span("session.setup.reset"):
            self.world.reset(self.config)
        self._prepared_config = copy.deepcopy(self.config)
        self._world_pristine = True
        self.elapsed_ticks = self.init_ticks = 0
        self.initialized = False
        self.mode = "disabled"
        self.samples.clear()

    def step(self) -> dict:
        # Manual steps/jogs and competition both consume the prepared state.
        self._world_pristine = False
        commands: dict = {"motors": {}, "adi": {}}
        if self.running and self.runtime is not None:
            with self.performance.span("session.native.input_snapshot"):
                native_input = self.world.snapshot()
            with self.performance.span("session.native.roundtrip"):
                result = self.runtime.step(native_input, self.phase, self.dt, self.controller)
            self.performance.increment("session.native.calls")
            # Startup callbacks can home mechanisms using the same sensor/actuator
            # loop as competition callbacks; only the competition clock waits.
            commands = result
            lcd = result.get("lcd")
            if lcd and lcd != self._last_lcd:
                self.log("LCD | " + (" | ".join(str(x) for x in lcd) if isinstance(lcd, list) else str(lcd)))
                self._last_lcd = lcd
            for diagnostic in result.get("diagnostics", []):
                self.log(str(diagnostic))
            if self.phase == "initialize":
                self.init_ticks += 1
                if result.get("initialized"):
                    self.initialized = True
                    self.phase = MODES[self.mode][1]
                    self.log(f"Robot initialized; running {MODES[self.mode][0]}")
                elif self.init_ticks * self.dt > 30:
                    raise TimeoutError("initialize()/competition_initialize() exceeded 30 simulated seconds")
            else:
                self.elapsed_ticks += 1
        if self.manual_joints and not self.running:
            commands["joints"] = dict(self.manual_joints)
        with self.performance.span("session.world.step"):
            snapshot = self.world.step(commands)
        self.performance.increment("session.steps")
        if snapshot is None:
            snapshot = self.world.snapshot()
        for diagnostic in snapshot.get("diagnostics", []):
            message = str(diagnostic)
            if message not in self._world_warnings:
                self.log(message)
                self._world_warnings.add(message)
        with self.performance.span("session.telemetry"):
            self._record(snapshot)
        if self.running and self.elapsed_ticks >= math.ceil(self.duration / self.dt - 1e-9):
            self.log("Mode time expired; robot disabled")
            self.stop()
            # disable() may change joint observations in an injected world.
            # Preserve the existing post-disable final-state observation.
            if "joints" in snapshot:
                snapshot["joints"] = self.world.joints
        with self.performance.span("session.state"):
            return self.state(snapshot)

    def _record(self, snapshot: dict) -> None:
        pose = snapshot.get("pose", [0, 0, 0, 0])
        observed = snapshot.get("otos", [0, 0, 0])
        # Bound interactive memory; report the cap instead of silently losing history.
        if len(self.samples) < 500_000:
            self.samples.append({"time": snapshot.get("time", 0), "mode_time": self.elapsed_ticks * self.dt,
                                 "mode": self.mode, "x_m": pose[0], "y_m": pose[1],
                                 "z_m": pose[2], "yaw_rad": pose[3],
                                 "otos_x_in": observed[0], "otos_y_in": observed[1],
                                 "otos_heading_deg": observed[2],
                                 "imu_heading_deg": snapshot.get("imu", {}).get("heading", 0),
                                 "sensor_age_ms": snapshot.get("sensor_age_ms", 0)})
        else:
            self.dropped_samples += 1
            if self.dropped_samples == 1:
                self.log("Telemetry buffer full (500,000 samples); reset or export before a new run")

    def state(self, snapshot: dict | None = None) -> dict:
        snapshot = snapshot if snapshot is not None else self.world.snapshot()
        # World.step()/snapshot() already own an isolated joint observation for
        # this revision. Reuse it within this state assembly; do not read every
        # joint again or retain a mutable snapshot for a later tick. Injected
        # worlds may omit joints, so keep their existing property fallback.
        joints = snapshot["joints"] if "joints" in snapshot else self.world.joints
        return {**snapshot, "mode": self.mode, "phase": self.phase, "running": self.running,
                "paused": self.paused, "initialized": self.initialized,
                "mode_time": self.elapsed_ticks * self.dt, "duration": self.duration,
                "remaining": max(0.0, self.duration - self.elapsed_ticks * self.dt),
                "joints": joints, "motor_bindings": self.world.motor_bindings,
                "physics": copy.deepcopy(getattr(self.world, "physics_status", {})),
                "renderer": getattr(self.world, "renderer_status", {}),
                "telemetry_samples": len(self.samples), "dropped_samples": self.dropped_samples}

    def export_csv(self, path: str | Path) -> None:
        with Path(path).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["time", "mode_time", "mode", "x_m", "y_m", "z_m", "yaw_rad",
                                                       "otos_x_in", "otos_y_in", "otos_heading_deg",
                                                       "imu_heading_deg", "sensor_age_ms"])
            writer.writeheader()
            writer.writerows(self.samples)

    def close(self) -> None:
        self.stop()
        self.world.close()
