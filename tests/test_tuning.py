import json
import math
import threading
from pathlib import Path

import numpy as np
import pytest

from eersim.tuning import (PIDController, TuningDataError, analyze_response, autotune, export_capture,
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


def test_discontinuity_reports_measured_interval_and_motor_without_claiming_tare():
    samples = capture()
    samples[30]["motors"]["1"]["position_deg"] += 300
    samples[30]["sensor_time"] = samples[30]["time"] - .04
    with pytest.raises(TuningDataError, match="Encoder position jumps") as caught:
        identify_model(samples, TARGET, "shaft")
    details = caught.value.diagnostics
    assert details["interval_s"] == pytest.approx(.02)
    assert details["residual_deg"] == pytest.approx(150)
    assert details["allowed_residual_deg"] == 30
    assert details["motor_mismatches"][0]["port"] == 1
    assert details["motor_mismatches"][0]["residual_deg"] == pytest.approx(300)
    assert details["feedback_age_s"] == pytest.approx(.04)
    assert "port 1: measured" in str(caught.value)
    assert "expected" in str(caught.value)
    assert "Feedback age 40.00 ms" in str(caught.value)
    assert "not proof" in str(caught.value)
    assert "Try next:" in str(caught.value)
    json.dumps(details, allow_nan=False)


def test_travel_cutoff_final_zero_is_not_mistaken_for_coast_down():
    samples = capture(count=30)
    samples[-1]["termination"] = "travel_limit"
    for motor in samples[-1]["motors"].values():
        motor["voltage_mv"] = 0.
    with pytest.raises(TuningDataError, match="0.6 seconds and 35 samples") as caught:
        identify_model(samples, TARGET, "shaft")
    details = caught.value.diagnostics
    assert details["duration_s"] == pytest.approx(.58)
    assert details["coast_duration_s"] == 0
    assert details["termination"] == "travel_limit"
    assert "Shorten the powered interval" in str(caught.value)
    assert "not a measured coast-down" in str(caught.value)


def test_fit_quality_is_saved_with_measured_coast_and_excitation():
    model = identify_model(capture(), TARGET, "shaft")
    details = model["fit_diagnostics"]
    assert details["r_squared"] >= details["minimum_r_squared"]
    assert details["holdout_rmse_rpm"] <= details["maximum_holdout_rmse_rpm"]
    assert details["prediction_normalized_rmse"] <= details["maximum_prediction_normalized_rmse"]
    assert details["coast_duration_s"] == pytest.approx(1.98)
    assert details["powered_duration_s"] == pytest.approx(2.6)
    assert details["baseline_duration_s"] == pytest.approx(.4)
    assert details["peak_voltage_mv"] == pytest.approx(2000)
    json.dumps(model, allow_nan=False)


def test_candidate_warns_about_voltage_extrapolation_and_modeled_saturation():
    result = autotune(capture(), TARGET, "shaft", {"step_target": 180, "duration_s": 4, "max_travel_deg": 300})
    warnings = " ".join(result["warnings"])
    assert "3000 mV controller limit exceeds the 2000 mV" in warnings
    assert result["metrics"]["saturation_fraction"] > .01
    assert "proportional term alone saturates" in warnings
    assert "mechanism validation" in warnings


def test_noisy_unmodeled_response_is_rejected_with_fit_evidence():
    samples = capture()
    noise = np.random.default_rng(17).normal(0, 35, len(samples))
    velocity = np.asarray([row["motors"]["1"]["velocity_rpm"] for row in samples]) + noise
    position = np.concatenate(([0.], np.cumsum((velocity[:-1]+velocity[1:])*3*.02)))
    for index, sample in enumerate(samples):
        for port, sign in (("1",1), ("2",-1)):
            sample["motors"][port]["position_deg"] = float(position[index]*sign)
            sample["motors"][port]["velocity_rpm"] = float(velocity[index]*sign)
    with pytest.raises(TuningDataError, match="does not fit") as caught:
        identify_model(samples, TARGET, "shaft")
    details = caught.value.diagnostics
    assert details["r_squared"] < details["minimum_r_squared"]
    assert "holdout error=" in str(caught.value)
    assert "Try next:" in str(caught.value)


def test_candidate_with_unsettled_prediction_is_explicitly_labeled():
    result = autotune(capture(), TARGET, "shaft", {"step_target": 720, "duration_s": 4,
                      "max_travel_deg": 2000, "output_limit_mv": 500})
    assert result["metrics"]["settling_time_s"] is None
    assert any("does not settle" in warning for warning in result["warnings"])
    assert result["status"] == "candidate"


def test_recorded_default_xdrive_trial_identifies_without_relaxing_quality_gates():
    fixture = json.loads((Path(__file__).parent / "fixtures" / "tuning_xdrive_physx_short.json").read_text(encoding="utf-8"))
    provenance, samples = fixture["provenance"], fixture["samples"]
    target = provenance["request"]["target"]
    result = autotune(samples, target, "shaft", {"controller_period_ms": provenance["controller_period_ms"],
                      "output_limit_mv": 2000, "duration_s": 2, "max_travel_deg": 180, "step_target": 90})
    model, details = result["model"], result["model"]["fit_diagnostics"]
    assert model["r_squared"] == pytest.approx(.8618086843, abs=.0001)
    assert model["prediction_normalized_rmse"] == pytest.approx(.1103969135, abs=.0001)
    assert details["coast_duration_s"] > .45
    assert details["peak_voltage_mv"] == 2000
    assert result["options"]["controller_period_ms"] == pytest.approx(1000 / 60)
    assert result["metrics"]["settling_time_s"] is not None
    assert max(map(abs, result["validation"]["voltage_mv"])) <= 2000
    assert not any("extrapolate" in warning for warning in result["warnings"])
    assert any("mechanism validation" in warning for warning in result["warnings"])
    # This is a prediction on a recorded plant fit, not a claim that native
    # closed-loop validation or transfer to the physical robot has passed.
    assert result["status"] == "candidate"
    json.dumps(result, allow_nan=False)
