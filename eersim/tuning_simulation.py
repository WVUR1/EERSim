"""Engine-thread motor capture and bounded simulation-only experiments."""
from __future__ import annotations

import math
import uuid

from .tuning import PIDController, controller_options, project_sample, validate_target


def motor_sample(snapshot, commands, capture_id):
    motors = {}
    for port, state in snapshot.get("motors", {}).items():
        command = commands.get("motors", {}).get(str(port), {})
        mode = command.get("mode", "voltage")
        motors[str(port)] = {"position_deg": math.degrees(state.get("position", 0)),
                             "velocity_rpm": state.get("velocity", 0)*30/math.pi,
                             # Never fabricate electrical telemetry for the model's
                             # implicit VEX position/velocity actuator.
                             "voltage_mv": float(command.get("value", 0)) if mode == "voltage" else None,
                             "command_mode": mode, "command_value": command.get("value", 0)}
    return {"time": float(snapshot["time"]), "source": "simulation", "capture_id": capture_id,
            "motors": motors, "pose": snapshot.get("pose"), "sensor_time": snapshot.get("sample_time")}


class SimulationTuner:
    def __init__(self, sample, status):
        self.emit_sample, self.emit_status = sample, status
        self.capturing = self.active = False
        self.capture_id = ""
        self.next_sample = -math.inf
        self.sample_period = .02
        self.request = None
        self.target = None
        self._session = None
        self.last_command = None

    def _status(self, state, message, **extra):
        self.emit_status({"state": state, "message": message, **extra})

    def capture(self, session, enabled, target=None):
        if self.active:
            raise ValueError("Stop the tuning experiment before changing capture")
        self.capturing = bool(enabled)
        self.target = target
        self._session = session
        self.capture_id = uuid.uuid4().hex
        self.next_sample = -math.inf
        self.last_command = None
        session.motor_observer = self.observe if enabled else None
        self._status("capturing" if enabled else "idle", "Recording PROS motor commands and feedback" if enabled else "Motor capture stopped", capture_id=self.capture_id)

    def observe(self, snapshot, commands):
        now = float(snapshot["time"])
        selected = {str(abs(p)) for p in self.target["ports"]} if self.target else set(snapshot.get("motors", {}))
        signature = tuple((port, commands.get("motors", {}).get(port, {}).get("mode", "voltage"),
                           commands.get("motors", {}).get(port, {}).get("value", 0)) for port in sorted(selected))
        # Record command edges even between nominal samples. Otherwise the
        # identification model would apply a pulse at the wrong timestamp.
        if self.capturing and (now + 1e-10 >= self.next_sample or signature != self.last_command):
            sample = motor_sample(snapshot, commands, self.capture_id)
            if self.target:
                sample["motors"] = {p: row for p, row in sample["motors"].items() if p in selected}
            if self.active and self.request.get("kind") == "validate":
                elapsed = now - self.started
                sample["control_mode"] = self.options["mode"]
                sample["target"] = (0. if elapsed < .3 else self.options["step_target"]
                                    if elapsed < .3 + self.options["duration_s"] else None)
            self.emit_sample(sample)
            self.next_sample = now + self.sample_period
            self.last_command = signature

    def experiment(self, session, request):
        target, axis = request["target"], request["axis"]
        ports, weights = validate_target(target, axis)
        available = {int(m["port"]) for m in session.world.motor_bindings}
        missing = {abs(p) for p, w in zip(ports, weights) if w and abs(p) not in available}
        if missing:
            raise ValueError(f"Map the selected ports to robot joints first: {sorted(missing)}")
        kind = request.get("kind", "characterize")
        if kind not in ("characterize", "validate"):
            raise ValueError("Unknown tuning experiment")
        options = controller_options(request)
        ticks = max(1, round(options["controller_period_ms"] / 1000 / session.dt))
        options["controller_period_ms"] = ticks * session.dt * 1000
        voltage = float(request.get("voltage_mv", 3000))
        if not math.isfinite(voltage) or not 100 <= abs(voltage) <= 12000:
            raise ValueError("Characterization voltage must have magnitude 100..12000 mV")
        if kind == "validate" and options["mode"] == "position" and abs(options["step_target"]) >= options["max_travel_deg"]:
            raise ValueError("Position target must be inside the allowed travel")
        controller = PIDController(request.get("gains", {}), options, options["step_target"]) if kind == "validate" else None
        # Exclusively own the actuators; user code is stopped before any test pulse.
        session.stop()
        session.manual_joints.clear()
        session.paused = False
        snapshot = session.world.snapshot()
        sample = motor_sample(snapshot, {"motors": {}}, "")
        initial = project_sample(sample, target, axis)
        self.initial_positions = {p: sample["motors"][str(abs(p))]["position_deg"] for p, w in zip(ports, weights) if w}
        self.origin = initial["position_deg"]
        self.started = float(snapshot["time"])
        self.next_control = self.started + .3
        self.previous_control = self.next_control
        self.voltage = 0.
        self.request, self.options = dict(request), options
        self.request["kind"] = kind
        self.request["voltage_mv"] = voltage
        self.target, self.axis, self.controller = target, axis, controller
        self.capture_id = uuid.uuid4().hex
        self.next_sample = -math.inf
        self.last_command = None
        self.sample_period = ticks * session.dt
        self.active = self.capturing = True
        self._session = session
        session.motor_observer = None
        self._status("running", "Characterizing selected motors" if kind == "characterize" else "Validating candidate in simulation",
                     kind=kind, capture_id=self.capture_id, controller_period_ms=options["controller_period_ms"])

    def step(self, session):
        snapshot = session.world.snapshot()
        sample = motor_sample(snapshot, {"motors": {}}, self.capture_id)
        elapsed = float(snapshot["time"]) - self.started
        duration = self.options["duration_s"]
        for port, initial in self.initial_positions.items():
            current = sample["motors"][str(abs(port))]["position_deg"]
            if abs(current-initial) >= self.options["max_travel_deg"]:
                self.cancel(session, "Travel limit reached; motors disabled", state="completed")
                return session.state(snapshot)
        if elapsed >= duration + .8 - 1e-10:
            self.observe(snapshot, {"motors": {}})
            self.cancel(session, "Experiment complete; motors disabled", state="completed")
            return session.state(snapshot)
        value = 0.
        if .3 <= elapsed < .3 + duration:
            if self.request["kind"] == "characterize":
                value = self.request["voltage_mv"]
            else:
                if float(snapshot["time"]) + 1e-10 >= self.next_control:
                    response = project_sample(sample, self.target, self.axis)
                    measured = response["position_deg"] - self.origin if self.options["mode"] == "position" else response["velocity_rpm"]
                    interval = max(self.sample_period, float(snapshot["time"])-self.previous_control)
                    self.voltage = self.controller.update(self.options["step_target"]-measured, interval)
                    self.previous_control = float(snapshot["time"])
                    self.next_control = self.previous_control + self.sample_period
                value = self.voltage
        commands = {"motors": {}, "adi": {}}
        for port, weight in zip(self.target["ports"], self.target["axes"][self.axis]):
            if weight:
                commands["motors"][str(abs(port))] = {"mode": "voltage", "value": value*weight*(1 if port > 0 else -1), "brake": "coast"}
        self.observe(snapshot, commands)
        session._world_pristine = False
        observed = session.world.step(commands)
        return session.state(observed if observed is not None else session.world.snapshot())

    def cancel(self, session=None, message="Tuning stopped", *, state="cancelled"):
        was_active = self.active
        self.active = self.capturing = False
        owned = session or self._session
        if owned is not None:
            owned.motor_observer = None
            if was_active:
                owned.world.disable()
                owned.paused = True
        self._session = None
        self._status(state, message)
