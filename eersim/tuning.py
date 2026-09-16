"""Response identification and provisional PID design in motor encoder units.

No hardware is actuated here. A least-squares, delayed first-order velocity
model is fitted to captured voltage/encoder data. Candidate controllers are
evaluated together with NumPy across model uncertainty. A fitted model is not
physical validation, and drivetrain axes are encoder projections, not odometry.
"""
from __future__ import annotations

import csv
import json
import math
from functools import lru_cache
from pathlib import Path
from typing import Callable

import numpy as np

SCHEMA = "eersim.tuning/1"
MAX_SAMPLES = 60_000


class TuningDataError(ValueError):
    """A rejected capture with measured evidence suitable for a debug report."""
    def __init__(self, message, diagnostics):
        super().__init__(message)
        self.diagnostics = dict(diagnostics)


def _capture_diagnostics(trace, samples=()):
    voltage = np.asarray([value for value in trace["voltage_mv"] if value is not None], dtype=float)
    velocity = np.asarray(trace["velocity_rpm"], dtype=float)
    result = {"samples": trace["samples"], "duration_s": trace["duration_s"],
              "sample_period_s": trace["sample_period_s"],
              "peak_velocity_rpm": float(np.max(np.abs(velocity))),
              "velocity_span_rpm": float(np.ptp(velocity))}
    if samples and samples[-1].get("termination"):
        result["termination"] = str(samples[-1]["termination"])
    if len(voltage) == len(velocity):
        threshold = max(20., float(np.max(np.abs(voltage))) * .01)
        powered = np.flatnonzero(np.abs(voltage) > threshold)
        intervals = np.diff(np.asarray(trace["time"], dtype=float))
        result.update({"peak_voltage_mv": float(np.max(np.abs(voltage))),
                       "voltage_span_mv": float(np.ptp(voltage)),
                       "final_voltage_mv": float(voltage[-1]),
                       "powered_duration_s": float(np.sum(intervals[np.abs(voltage[:-1]) > threshold])),
                       "baseline_duration_s": float(trace["time"][powered[0]]) if len(powered) else float(trace["time"][-1]),
                       "coast_duration_s": float(trace["time"][-1] - trace["time"][powered[-1] + 1])
                       if len(powered) and powered[-1] + 1 < len(voltage) else 0.})
    return result


def _retry_advice(target, diagnostics):
    steps = []
    if diagnostics.get("termination") == "travel_limit":
        steps.append("The trial reached its travel limit. The final zero-voltage stop sample is not a measured coast-down; any earlier coast data may be incomplete. Shorten the powered interval or reduce voltage; increase the travel bound only within the mechanism's usable range.")
    elif abs(diagnostics.get("final_voltage_mv", 0)) > 20:
        steps.append("The capture ends while voltage is still applied. Shorten the powered interval so the trial includes the stop/coast response; increase the travel bound only within the mechanism's usable range.")
    if target.get("kind") in {"tank", "h_drive", "x_drive"}:
        steps.append("Reset the robot, select the complete drivetrain and one axis, and repeat in clear field space without contacting a wall or goal.")
    elif len(target.get("ports", [])) == 1:
        steps.append("If this is a drive wheel, select the complete drivetrain target and forward/turn/strafe axis instead of powering one wheel against the other wheels.")
    else:
        steps.append("Reset to the same starting pose and use one mechanism/axis with a consistent payload and direction.")
    steps.append("Export the capture if the same failure repeats; keep the voltage and encoder traces with these measured diagnostics.")
    return "Try next: " + " ".join(steps)


def _finite(value, name):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def validate_target(target, axis):
    ports = target.get("ports", [])
    if not ports or any(isinstance(p, bool) or not isinstance(p, int) or not 1 <= abs(p) <= 21 for p in ports):
        raise ValueError("Select valid signed motor ports in 1..21")
    if len({abs(p) for p in ports}) != len(ports):
        raise ValueError("A motor port cannot occur twice in one target")
    weights = [_finite(v, "Axis coefficient") for v in target.get("axes", {}).get(axis, [])]
    if len(weights) != len(ports) or not any(weights) or max(map(abs, weights)) > 1:
        raise ValueError("Select an axis with one coefficient in -1..1 per motor")
    return list(ports), weights


@lru_cache(maxsize=128)
def _projection(columns, index):
    matrix = np.asarray(columns, dtype=float).T
    if np.linalg.matrix_rank(matrix) != len(columns):
        raise ValueError("Drivetrain axes overlap ambiguously; define independent motor mixtures")
    # Unequal motor counts make drive axes nonorthogonal. The left inverse
    # separates the declared mixtures without turn contaminating forward data.
    return tuple(np.linalg.pinv(matrix)[index])


def project_sample(sample, target, axis):
    ports, weights = validate_target(target, axis)
    axes = list(target["axes"])
    columns = tuple(tuple(validate_target(target, name)[1]) for name in axes)
    coefficients = _projection(columns, axes.index(axis))
    result = {"time": _finite(sample["time"], "Sample time")}
    for name in ("position_deg", "velocity_rpm", "voltage_mv"):
        total = 0.0
        for port, weight in zip(ports, coefficients):
            if abs(weight) < 1e-12:
                continue
            motor = sample.get("motors", {}).get(str(abs(port)))
            if motor is None:
                raise ValueError(f"Capture is missing motor port {abs(port)}")
            value = motor.get(name)
            if value is None and name == "voltage_mv":
                total = None
                break
            total += _finite(value, name) * weight * (1 if port > 0 else -1)
        result[name] = total
    return result


def analyze_response(samples, target, axis, mode="position"):
    if mode not in ("position", "velocity"):
        raise ValueError("Choose position or velocity control")
    if len(samples) < 3:
        raise ValueError("Record at least three motor samples")
    if len(samples) > MAX_SAMPLES:
        raise ValueError(f"Capture exceeds {MAX_SAMPLES:,} samples; select a shorter run")
    sources = {row.get("source", "unknown") for row in samples}
    if len(sources) > 1:
        raise ValueError("Do not mix hardware and simulation captures")
    runs = {row.get("capture_id") for row in samples if row.get("capture_id")}
    if len(runs) > 1:
        raise ValueError("Select one capture at a time")
    rows = [project_sample(row, target, axis) for row in samples]
    times = np.array([r["time"] for r in rows])
    differences = np.diff(times)
    if np.any(differences <= 0):
        raise ValueError("Capture timestamps must increase; start a new capture after Reset")
    positions = np.array([r["position_deg"] for r in rows])
    velocity = np.array([r["velocity_rpm"] for r in rows])
    # Detect taring/discontinuities without imposing a particular mechanism speed.
    predicted = (velocity[:-1] + velocity[1:]) * 3 * differences
    residuals = np.abs(np.diff(positions) - predicted)
    thresholds = np.maximum(30, np.abs(predicted) * 3)
    bad = np.flatnonzero(residuals > thresholds)
    if len(bad):
        index = int(bad[0])
        measured = float(positions[index+1] - positions[index])
        details = {"interval_index": index, "time_s": float(times[index+1]-times[0]),
                   "interval_s": float(differences[index]), "measured_delta_deg": measured,
                   "velocity_predicted_delta_deg": float(predicted[index]),
                   "residual_deg": float(residuals[index]), "allowed_residual_deg": float(thresholds[index])}
        mismatches = []
        for port in target["ports"]:
            before = samples[index].get("motors", {}).get(str(abs(port)), {})
            after = samples[index+1].get("motors", {}).get(str(abs(port)), {})
            if all(before.get(key) is not None and after.get(key) is not None for key in ("position_deg", "velocity_rpm")):
                change = float(after["position_deg"])-float(before["position_deg"])
                expected = (float(before["velocity_rpm"])+float(after["velocity_rpm"]))*3*float(differences[index])
                mismatches.append({"port": abs(port), "measured_delta_deg": change,
                                   "velocity_predicted_delta_deg": expected, "residual_deg": abs(change-expected)})
        details["motor_mismatches"] = sorted(mismatches, key=lambda item: item["residual_deg"], reverse=True)[:3]
        sensor_time = samples[index+1].get("sensor_time")
        if sensor_time is not None:
            details["feedback_age_s"] = float(times[index+1])-float(sensor_time)
        motor_details = "; ".join(f"port {item['port']}: measured {item['measured_delta_deg']:.2f}°, "
                                  f"expected {item['velocity_predicted_delta_deg']:.2f}°"
                                  for item in details["motor_mismatches"])
        feedback_details = (f"Feedback age {details['feedback_age_s']*1000:.2f} ms. "
                            if "feedback_age_s" in details else "")
        raise TuningDataError(
            f"Encoder position jumps were detected: position and velocity disagree at {details['time_s']:.3f} s. "
            f"Over {differences[index]*1000:.2f} ms, this axis changed {measured:.2f}°, while its endpoint velocities predict {predicted[index]:.2f}°. "
            f"Mismatch {residuals[index]:.2f}° exceeds the {thresholds[index]:.2f}° tolerance. "
            + (f"Largest motor mismatches: {motor_details}. " if motor_details else "")
            + feedback_details
            + "This can indicate an encoder reset, wrapped angle, stale feedback or unresolved motion between samples; it is not proof that your code called tare. "
            "Try next: record a fresh continuous trial. For simulation, repeat with encoder noise, latency and dropout at zero; export the capture if it recurs.", details)
    return {"time": (times - times[0]).tolist(),
            "position_deg": (positions - positions[0]).tolist(),
            "velocity_rpm": velocity.tolist(),
            "response": ((positions - positions[0]) if mode == "position" else velocity).tolist(),
            "voltage_mv": [r["voltage_mv"] for r in rows],
            "sample_period_s": float(np.median(differences)), "samples": len(rows),
            "duration_s": float(times[-1] - times[0]), "source": next(iter(sources)),
            "units": "motor degrees" if mode == "position" else "motor rpm"}


def identify_model(samples, target, axis):
    trace = analyze_response(samples, target, axis)
    diagnostics = _capture_diagnostics(trace, samples)
    if len(samples) < 35 or trace["duration_s"] < .6:
        raise TuningDataError(f"Record at least 0.6 seconds and 35 samples including voltage changes and coast-down. "
                              f"This capture contains {len(samples)} samples over {trace['duration_s']:.3f} s. "
                              + _retry_advice(target, diagnostics), diagnostics)
    if any(value is None for value in trace["voltage_mv"]):
        raise TuningDataError("Voltage telemetry is unavailable for this capture. "
                              "Try next: use Characterize in simulation or record voltage-driven robot code. "
                              "A position/velocity command is not measured electrical voltage.", diagnostics)
    t = np.array(trace["time"])
    dt = trace["sample_period_s"]
    if np.max(np.diff(t)) > dt * 5:
        diagnostics["largest_gap_s"] = float(np.max(np.diff(t)))
        raise TuningDataError(f"Capture has large timing gaps: {diagnostics['largest_gap_s']*1000:.2f} ms "
                              f"compared with the usual {dt*1000:.2f} ms. "
                              "Try next: reconnect the logger and record a fresh continuous trial; use a shorter capture if the host is overloaded.", diagnostics)
    uniform = np.arange(0, t[-1] + dt * .01, dt)
    velocity = np.interp(uniform, t, trace["velocity_rpm"])
    # Inputs are held commands, not linearly interpolated between transitions.
    indices = np.clip(np.searchsorted(t, uniform, side="right") - 1, 0, len(t) - 1)
    voltage = np.asarray(trace["voltage_mv"])[indices]
    if np.ptp(voltage) < 250 or np.std(voltage) < 80:
        diagnostics["voltage_std_mv"] = float(np.std(voltage))
        raise TuningDataError(f"Not enough voltage excitation: span {np.ptp(voltage):.0f} mV (minimum 250 mV), "
                              f"standard deviation {np.std(voltage):.0f} mV (minimum 80 mV). "
                              "Try next: include a stopped baseline, a powered move, and a coast-down. Use Characterize for a bounded simulation trial.", diagnostics)
    if np.ptp(velocity) < 3:
        raise TuningDataError(f"Motor response is too small to identify: velocity span {np.ptp(velocity):.2f} RPM (minimum 3 RPM). "
                              "Try next: check the selected ports and signs, joint limits and mechanism load. "
                              "For a drivetrain, choose its complete forward/turn/strafe axis. Increase voltage only if the mechanism has room to move.", diagnostics)
    if np.max(np.abs(voltage)) > 12500:
        raise ValueError("Voltage must be recorded in millivolts, within the V5 voltage range")
    best = None
    max_delay = min(10, len(uniform) // 12, max(0, int(.15 / dt)))
    for delay in range(max_delay + 1):
        x = np.column_stack((velocity[delay:-1], voltage[:len(voltage)-delay-1], np.ones(len(voltage)-delay-1)))
        y = velocity[delay+1:]
        if np.linalg.matrix_rank(x) < 3:
            continue
        split = max(20, int(len(y) * .75))
        coefficients = np.linalg.lstsq(x[:split], y[:split], rcond=None)[0]
        a, b, c = map(float, coefficients)
        if not .001 < a < .9999 or b <= 0:
            continue
        residual = y - x @ coefficients
        mse = float(np.mean(residual ** 2))
        score = mse * (1 + delay * .03)
        if best is None or score < best[0]:
            best = score, a, b, c, delay, mse, float(np.mean(residual[split:] ** 2)), float(np.var(y))
    if best is None:
        raise TuningDataError("Could not fit a stable positive motor response. "
                              f"Measured voltage span {diagnostics['voltage_span_mv']:.0f} mV, velocity span {diagnostics['velocity_span_rpm']:.2f} RPM, "
                              f"sample period {dt*1000:.2f} ms and coast duration {diagnostics['coast_duration_s']:.3f} s. "
                              "Check motor reversal and axis mapping. " + _retry_advice(target, diagnostics), diagnostics)
    _, a, b, c, delay, mse, holdout, variance = best
    r2 = 1 - mse / max(variance, 1e-12)
    diagnostics.update({"r_squared": r2, "minimum_r_squared": .65,
                        "one_step_rmse_rpm": math.sqrt(mse), "holdout_rmse_rpm": math.sqrt(holdout),
                        "maximum_holdout_rmse_rpm": math.sqrt(max(variance*.5, 4.)),
                        "pole": a, "delay_s": delay*dt})
    if r2 < .65 or holdout > max(variance * .5, 4.0):
        raise TuningDataError("Response does not fit the tuning model reliably. "
                              f"R²={r2:.3f} (minimum 0.650); holdout error={math.sqrt(holdout):.2f} RPM "
                              f"(maximum {math.sqrt(max(variance*.5, 4.)):.2f} RPM). "
                              f"Recorded {len(samples)} samples over {trace['duration_s']:.3f} s, "
                              f"with {diagnostics['coast_duration_s']:.3f} s after powered motion. "
                              + _retry_advice(target, diagnostics), diagnostics)
    gain = b / (1 - a)
    tau = -dt / math.log(a)
    # Evaluate a free-running prediction, so a good one-step fit cannot conceal drift.
    predicted = np.zeros(len(velocity))
    predicted[0] = velocity[0]
    for index in range(len(velocity)-1):
        predicted[index+1] = a * predicted[index] + b * voltage[max(0, index-delay)] + c
    normalized_rmse = float(np.sqrt(np.mean((velocity-predicted)**2)) / max(np.ptp(velocity), 1))
    diagnostics["prediction_normalized_rmse"] = normalized_rmse
    diagnostics["maximum_prediction_normalized_rmse"] = .3
    if normalized_rmse > .3:
        raise TuningDataError("Model prediction drifts too far from the measured response. "
                              f"Free-running prediction error is {normalized_rmse*100:.1f}% of the measured velocity span "
                              "(maximum 30.0%). A good one-step fit alone does not validate this model. "
                              + _retry_advice(target, diagnostics), diagnostics)
    return {"type": "delayed_first_order_velocity", "gain_rpm_per_mv": gain,
            "time_constant_s": tau, "delay_s": delay * dt,
            "bias_mv": -c / b, "sample_period_s": dt, "r_squared": r2,
            "prediction_normalized_rmse": normalized_rmse,
            "source": trace["source"], "samples": len(samples),
            "duration_s": trace["duration_s"], "fit_diagnostics": diagnostics}


def controller_options(options=None):
    incoming = dict(options or {})
    mode = incoming.get("mode", "position")
    if mode not in ("position", "velocity"):
        raise ValueError("Choose position or velocity control")
    result = {"mode": mode,
              "controller_period_ms": _finite(incoming.get("controller_period_ms", 20), "Controller period"),
              "step_target": _finite(incoming.get("step_target", 90 if mode == "position" else 60), "Step target"),
              "output_limit_mv": _finite(incoming.get("output_limit_mv", abs(incoming.get("voltage_mv", 3000))), "Voltage limit"),
              "duration_s": _finite(incoming.get("duration_s", 3), "Duration"),
              "large_error": _finite(incoming.get("large_error", 1e6), "Integral threshold"),
              "antiwind": _finite(incoming.get("antiwind", 1000), "Integral clamp"),
              "max_travel_deg": _finite(incoming.get("max_travel_deg", 360), "Travel limit")}
    if not 1 <= result["controller_period_ms"] <= 1000:
        raise ValueError("Controller period must be in 1..1000 ms")
    if not 100 <= result["output_limit_mv"] <= 12000 or not .2 <= result["duration_s"] <= 60:
        raise ValueError("Choose 100..12000 mV and a duration in 0.2..60 seconds")
    if result["step_target"] == 0 or result["large_error"] <= 0 or result["antiwind"] < 0 or result["max_travel_deg"] <= 0:
        raise ValueError("Target must be nonzero; travel/threshold positive and integral clamp nonnegative")
    return result


class PIDController:
    """EERKit's error derivative and clamped integral contribution, with output cap.

    Integral storage itself is not clipped, matching eerkit::PID::calculate.
    Reset initializes last_error to the current error to avoid derivative kick.
    """
    def __init__(self, gains, options, initial_error=0):
        self.gains = tuple(_finite(gains[key], key) for key in ("kp", "ki", "kd"))
        if min(self.gains) < 0:
            raise ValueError("PID gains must be nonnegative")
        self.options = controller_options(options)
        self.integral = 0.0
        self.last_error = initial_error

    def update(self, error, dt):
        error, dt = _finite(error, "PID error"), _finite(dt, "PID interval")
        if dt <= 0:
            raise ValueError("PID interval must be positive")
        kp, ki, kd = self.gains
        self.integral = self.integral + error * dt if 0 < abs(error) < self.options["large_error"] else 0.
        clamp = self.options["antiwind"]
        output = kp * error + ki * max(-clamp, min(clamp, self.integral)) + kd * (error-self.last_error) / dt
        self.last_error = error
        limit = self.options["output_limit_mv"]
        return max(-limit, min(limit, output))


def _cancelled(cancel):
    if cancel is not None and (cancel.is_set() if hasattr(cancel, "is_set") else cancel()):
        raise InterruptedError("Tuning cancelled")


def _evaluate_candidates(gains, model, options, *, robust=True, cancel=None):
    """Vectorized independent trials; keeps Python out of the candidate loop."""
    dt = options["controller_period_ms"] / 1000
    horizon = max(options["duration_s"], min(15., model["time_constant_s"] * 6))
    count = max(30, int(math.ceil(horizon/dt)))
    variants = [(1., 1.)] if not robust else [(1., 1.), (.75, 1.3), (1.25, .7)]
    gain_array = np.repeat(np.asarray(gains, dtype=float), len(variants), axis=0)
    scale = np.tile(np.asarray(variants), (len(gains), 1))
    k = model["gain_rpm_per_mv"] * scale[:, 0]
    tau = model["time_constant_s"] * scale[:, 1]
    a = np.exp(-dt / tau)
    b = k * (1-a)
    lag = max(0, int(math.ceil(model["delay_s"] / dt)))
    queue = np.zeros((lag+1, len(gain_array)))
    velocity = np.zeros(len(gain_array))
    position = np.zeros(len(gain_array))
    integral = np.zeros(len(gain_array))
    target = options["step_target"]
    last_error = np.full(len(gain_array), target)
    absolute_error = np.zeros(len(gain_array))
    overshoot = np.zeros(len(gain_array))
    travel = np.zeros(len(gain_array))
    saturated = np.zeros(len(gain_array))
    response_trace, voltage_trace = [], []
    clamp, limit = options["antiwind"], options["output_limit_mv"]
    for tick in range(count):
        if tick % 64 == 0:
            _cancelled(cancel)
        response = position.copy() if options["mode"] == "position" else velocity.copy()
        error = target - response
        integral = np.where((np.abs(error) < options["large_error"]) & (error != 0), integral + error*dt, 0)
        u = gain_array[:, 0]*error + gain_array[:, 1]*np.clip(integral, -clamp, clamp) + gain_array[:, 2]*(error-last_error)/dt
        u = np.clip(u, -limit, limit)
        last_error = error.copy()
        queue[tick % (lag+1)] = u
        delayed = queue[(tick-lag) % (lag+1)]
        next_velocity = a*velocity + b*(delayed-model["bias_mv"])
        position += (velocity+next_velocity) * 3 * dt
        velocity = next_velocity
        absolute_error += np.abs(error) * dt * (1 + tick / count)
        overshoot = np.maximum(overshoot, np.maximum(0, response*np.sign(target)-abs(target)))
        travel = np.maximum(travel, np.abs(position))
        saturated += np.abs(u) >= limit * .999
        if not robust:
            response_trace.append(float(response[0]))
            voltage_trace.append(float(u[0]))
    score = absolute_error / max(abs(target), 1) + overshoot / max(abs(target), 1) * horizon * 4
    score += np.abs(last_error) / max(abs(target), 1) * horizon * 2
    score += np.where(travel > options["max_travel_deg"], 1e6 + travel, 0)
    scores = score.reshape(len(gains), len(variants)).max(axis=1)
    return scores, {"time": (np.arange(count)*dt).tolist(), "target": [target]*count,
                    "response": response_trace, "voltage_mv": voltage_trace,
                    "saturation_fraction": float(saturated[0]/count)}


def response_metrics(trace):
    t = np.asarray(trace["time"])
    y = np.asarray(trace["response"])
    target = float(trace["target"][-1])
    tolerance = max(abs(target)*.02, .1)
    outside = np.flatnonzero(np.abs(y-target) > tolerance)
    settling = None if len(outside) and outside[-1] >= len(t)-2 else float(t[outside[-1]+1] if len(outside) else 0)
    return {"overshoot_percent": float(max(0, np.max(y*np.sign(target))-abs(target))/abs(target)*100),
            "settling_time_s": settling, "steady_state_error": float(target-np.mean(y[-max(1,len(y)//10):])),
            "saturation_fraction": trace.get("saturation_fraction", 0)}


def autotune(samples, target, axis, options=None, cancel=None, progress: Callable | None = None):
    _cancelled(cancel)
    options = controller_options(options)
    model = identify_model(samples, target, axis)
    if progress:
        progress({"fraction": .15, "message": "Response identified; testing candidate controllers"})
    dt = options["controller_period_ms"] / 1000
    if dt < model["sample_period_s"] * .9:
        raise ValueError("Controller period is faster than the recorded data. Capture at the intended control rate")
    tau, k = model["time_constant_s"], model["gain_rpm_per_mv"]
    candidates = []
    for speed in (.4, .7, 1., 1.4, 2.):
        if options["mode"] == "position":
            omega = min(2/tau, 1/(5*dt), 1/max(.02, model["delay_s"]*3)) * speed
            kp = tau*omega*omega/(6*k)
            kd = max(0, (2*.9*omega*tau-1)/(6*k))
            for p in (.5, 1., 2.):
                for i in (0., .1, .4):
                    for d in (.5, 1., 2.):
                        candidates.append((kp*p, kp*i/max(tau,.1), kd*d))
        else:
            settling = max(tau, dt*5, model["delay_s"]*3)/speed
            for p in (.5, 1., 2.):
                for i in (0., .5, 1., 2.):
                    candidates.append((tau/(k*settling)*p, 1/(k*settling)*i, 0.))
    scores, _ = _evaluate_candidates(candidates, model, options, cancel=cancel)
    index = int(np.argmin(scores))
    if scores[index] >= 1e6:
        raise ValueError("No candidate stayed inside the travel limit. Reduce target/duration or adjust the allowed travel")
    gains = dict(zip(("kp", "ki", "kd"), candidates[index]))
    _, trace = _evaluate_candidates([candidates[index]], model, options, robust=False, cancel=cancel)
    metrics = response_metrics(trace)
    warnings = ["Candidate gains from a fitted response model; validate on the simulated mechanism and then the physical robot.",
                "Drivetrain axes use projected motor encoders. These gains are not inch/metre or IMU-heading path-controller gains.",
                "Controller uses derivative of error, integral separation/clamp, and millivolt output. Preserve these units and settings."]
    if abs(model["bias_mv"]) > options["output_limit_mv"] * .15:
        warnings.append("Significant holding/friction bias was identified; test both directions and payloads.")
    diagnostics = model["fit_diagnostics"]
    if diagnostics.get("termination") == "travel_limit":
        warnings.append("Characterization stopped at its travel limit. The fitted response does not validate the complete stop/coast behavior; repeat with a shorter powered interval before relying on these gains.")
    elif diagnostics.get("coast_duration_s", 0) < 2 * model["sample_period_s"]:
        warnings.append("The capture contains fewer than two measured coast intervals after the last powered move. A final zero-voltage sample alone is not coast-down evidence.")
    if options["output_limit_mv"] > diagnostics["peak_voltage_mv"] * 1.05:
        warnings.append(f"The {options['output_limit_mv']:.0f} mV controller limit exceeds the {diagnostics['peak_voltage_mv']:.0f} mV observed excitation. Predictions at the higher voltage extrapolate beyond the capture; characterize at the intended limit to check them.")
    if metrics["saturation_fraction"] > .01:
        units = "motor degrees" if options["mode"] == "position" else "motor RPM"
        gain_units = "motor degree" if options["mode"] == "position" else "motor RPM"
        warnings.append(f"Predicted output reaches the {options['output_limit_mv']:.0f} mV limit for {metrics['saturation_fraction']*100:.1f}% of this step. "
                        f"Kp is in mV per {gain_units}; the proportional term alone saturates above {options['output_limit_mv']/gains['kp']:.2f} {units} of error. "
                        "Saturation is modeled, but this fitted-model prediction still requires a mechanism validation run.")
    if metrics["settling_time_s"] is None:
        warnings.append(f"The nominal prediction does not settle within 2% of the target during its {trace['time'][-1]:.2f} s horizon. This is a candidate, not a successful tuning result; inspect the trace and validate before exporting gains to robot code.")
    if progress:
        progress({"fraction": 1., "message": "Candidate ready for validation"})
    return {"schema": SCHEMA, "status": "candidate", "source": model["source"],
            "target": target, "axis": axis, "gains": gains, "options": options,
            "model": model, "metrics": metrics, "validation": trace,
            "candidate_count": len(candidates), "uncertainty_trials": len(candidates)*3,
            "warnings": warnings, "controller": "eerkit_error_pid_millivolts",
            "error_units": "motor_degrees" if options["mode"] == "position" else "motor_rpm"}


def export_result(path, result):
    Path(path).write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def export_capture(path, samples):
    path = Path(path)
    if path.suffix.lower() == ".csv":
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=["time", "source", "capture_id", "port", "position_deg", "velocity_rpm", "voltage_mv"])
            writer.writeheader()
            for sample in samples:
                for port, motor in sample.get("motors", {}).items():
                    writer.writerow({"time": sample["time"], "source": sample.get("source", "unknown"),
                                     "capture_id": sample.get("capture_id", ""), "port": port,
                                     **{key: motor.get(key) for key in ("position_deg", "velocity_rpm", "voltage_mv")}})
    else:
        with path.open("w", encoding="utf-8") as stream:
            for sample in samples:
                stream.write(json.dumps(sample, allow_nan=False) + "\n")


def read_capture(path):
    path = Path(path)
    if path.stat().st_size > 64 * 1024 * 1024:
        raise ValueError("Capture exceeds 64 MiB; select a shorter recording")
    samples = []
    if path.suffix.lower() == ".csv":
        with path.open(newline="", encoding="utf-8-sig") as stream:
            by_time = {}
            for row in csv.DictReader(stream):
                key = (row.get("capture_id", ""), row["time"])
                if key not in by_time:
                    sample = {"time": float(row["time"]), "source": row.get("source", "unknown"),
                              "capture_id": key[0], "motors": {}}
                    by_time[key] = sample
                    samples.append(sample)
                physical_port = int(row["port"])
                if not 1 <= physical_port <= 21:
                    raise ValueError("CSV capture ports must be physical positive ports in 1..21")
                port = str(physical_port)
                if port in by_time[key]["motors"]:
                    raise ValueError("Duplicate motor/timestamp in CSV capture")
                if row.get("source", "unknown") != by_time[key]["source"]:
                    raise ValueError("Do not mix hardware and simulation captures")
                by_time[key]["motors"][port] = {
                    name: float(row[name]) if row.get(name) else None for name in ("position_deg", "velocity_rpm", "voltage_mv")}
    else:
        with path.open(encoding="utf-8-sig") as stream:
            for line in stream:
                if not line.strip():
                    continue
                if line.startswith("EERSIM_TUNE"):
                    raise ValueError("This is raw per-port terminal output. Use Connect / record in the tuning window, then export a complete JSONL or CSV capture")
                sample = json.loads(line)
                if sample is not None:
                    samples.append(sample)
                if len(samples) > MAX_SAMPLES:
                    raise ValueError(f"Capture exceeds {MAX_SAMPLES:,} samples")
    if not samples or len(samples) > MAX_SAMPLES or any(not isinstance(s, dict) or "motors" not in s or "time" not in s for s in samples):
        raise ValueError("Select a nonempty EERSim motor capture")
    return samples
