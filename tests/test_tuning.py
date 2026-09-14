import json
import math
import threading

import numpy as np
import pytest

from eersim.tuning import (PIDController, analyze_response, autotune, export_capture,
                           identify_model, project_sample, read_capture, validate_target)


TARGET = {"id": "arm", "name": "Arm", "kind": "motor_group", "ports": [1, -2], "axes": {"shaft": [1, 1]}}


def capture(*, count=300, dt=.02, gain=.04, tau=.16, delay=2, source="simulation"):
    velocity, position = 0., 0.
    queue = [0.] * (delay+1)
    result = []
    a = math.exp(-dt/tau)
    for tick in range(count):
        voltage = 2000. if .4 <= tick*dt < 2 else (-1200. if 3 <= tick*dt < 4 else 0.)
        result.append({"time": tick*dt, "source": source, "capture_id": "one", "motors": {
            "1": {"position_deg": position, "velocity_rpm": velocity, "voltage_mv": voltage},
            "2": {"position_deg": -position, "velocity_rpm": -velocity, "voltage_mv": -voltage}}})
        queue[tick % len(queue)] = voltage
        following = a*velocity + gain*(1-a)*queue[(tick-delay) % len(queue)]
        position += (velocity+following)*3*dt
        velocity = following
    return result


def test_identification_recovers_known_response_and_delay_from_signed_group():
    model = identify_model(capture(), TARGET, "shaft")
    assert model["gain_rpm_per_mv"] == pytest.approx(.04, rel=.03)
    assert model["time_constant_s"] == pytest.approx(.16, rel=.05)
    assert model["delay_s"] == pytest.approx(.04, abs=.021)
    assert model["prediction_normalized_rmse"] < .04


@pytest.mark.parametrize("mode,step,travel", [("position", 90, 240), ("velocity", 30, 3000)])
def test_tuned_candidates_settle_on_identified_model(mode, step, travel):
    result = autotune(capture(), TARGET, "shaft", {"mode": mode, "step_target": step,
                      "duration_s": 4, "max_travel_deg": travel})
    assert result["status"] == "candidate"
    assert result["uncertainty_trials"] == result["candidate_count"]*3
    assert result["metrics"]["settling_time_s"] is not None
    assert result["validation"]["response"][0] == 0
    assert abs(result["metrics"]["steady_state_error"]) < abs(step)*.03
    assert max(map(abs, result["validation"]["voltage_mv"])) <= 3000
    json.dumps(result, allow_nan=False)


def test_cancel_is_checked_before_identification():
    stop = threading.Event(); stop.set()
    with pytest.raises(InterruptedError):
        autotune([], TARGET, "shaft", cancel=stop)


@pytest.mark.parametrize("fault,expected", [("missing_voltage", "Voltage telemetry"),
    ("no_excitation", "voltage excitation"), ("tare", "Encoder position jumps"),
    ("reset", "timestamps"), ("mixed_source", "mix hardware"), ("mixed_run", "one capture")])
def test_unusable_measurements_are_rejected_instead_of_emitting_gains(fault, expected):
    samples = capture()
    if fault == "missing_voltage": samples[30]["motors"]["1"]["voltage_mv"] = None
    if fault == "no_excitation":
        for sample in samples:
            sample["motors"]["1"]["voltage_mv"] = 0
            sample["motors"]["2"]["voltage_mv"] = 0
    if fault == "tare": samples[30]["motors"]["1"]["position_deg"] += 300
    if fault == "reset": samples[30]["time"] = 0
    if fault == "mixed_source": samples[30]["source"] = "hardware"
    if fault == "mixed_run": samples[30]["capture_id"] = "two"
    with pytest.raises(ValueError, match=expected): identify_model(samples, TARGET, "shaft")


def test_group_projection_handles_h_drive_strafe_without_longitudinal_motors():
    target = {"ports": [1,-2,3], "axes": {"strafe": [0,0,1]}}
    assert project_sample({"time": 0, "motors": {"3": {"position_deg": 40, "velocity_rpm": 2, "voltage_mv": 1200}}}, target, "strafe")["position_deg"] == 40
    with pytest.raises(ValueError, match="twice"):
        validate_target({"ports": [1,-1], "axes": {"shaft": [1,1]}}, "shaft")


def test_unequal_count_tank_axes_do_not_mix_turn_with_forward():
    target = {"ports":[1,2,3],"axes":{"forward":[1,1,1],"turn":[1,1,-1]}}
    sample = {"time":0,"motors":{str(p):{"position_deg":v,"velocity_rpm":v,"voltage_mv":v} for p,v in [(1,50),(2,50),(3,-50)]}}
    assert project_sample(sample,target,"forward")["position_deg"] == pytest.approx(0)
    assert project_sample(sample,target,"turn")["position_deg"] == pytest.approx(50)


@pytest.mark.parametrize("suffix", [".csv", ".jsonl"])
def test_capture_export_roundtrip_preserves_unknown_voltage_and_group_data(tmp_path, suffix):
    samples = capture(count=5)
    samples[2]["motors"]["2"]["voltage_mv"] = None
    path = tmp_path / ("run"+suffix)
    export_capture(path, samples)
    assert read_capture(path) == samples


def test_pid_integral_separation_reset_and_seconds_units():
    pid = PIDController({"kp": 2,"ki": 3,"kd": .1}, {"large_error": 10,"antiwind": 1}, initial_error=20)
    assert pid.update(20,.02) == 40
    assert pid.update(5,.02) == pytest.approx(10+.3-75)
    for _ in range(100): pid.update(5,.02)
    assert pid.update(5,.02) == 13
    assert pid.update(0,.02) == -25


def test_nonfinite_output_limit_and_negative_sample_period_rejected():
    with pytest.raises(ValueError, match="finite"):
        autotune(capture(), TARGET, "shaft", {"output_limit_mv": float("nan")})
    with pytest.raises(ValueError, match="period"):
        autotune(capture(), TARGET, "shaft", {"controller_period_ms": -1})
