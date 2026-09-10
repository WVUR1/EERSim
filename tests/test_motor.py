"""Reference checks against VEX's published specifications and performance graph."""
import math

import pytest

from eersim.motor import CARTRIDGES, available_torque, cartridge_for, external_output, pwm_torque


@pytest.mark.parametrize("name,rpm,ratio,stall,ticks", [
    ("red", 100, 36, 2.1, 1800), ("green", 200, 18, 1.05, 900), ("blue", 600, 6, .35, 300),
])
def test_official_cartridge_values_and_stall(name, rpm, ratio, stall, ticks):
    preset = CARTRIDGES[name]
    assert preset.rpm == rpm
    assert preset.internal_ratio == ratio
    assert preset.stall_torque == stall
    assert preset.ticks_per_rev == ticks
    assert preset.current_limit == 2.5
    motor = {**preset.to_dict(), "max_power": preset.peak_power}
    assert pwm_torque(motor, 1, 0) == pytest.approx(stall)
    assert pwm_torque(motor, -1, 0) == pytest.approx(-stall)
    assert pwm_torque(motor, 0, 0) == 0


@pytest.mark.parametrize("name", CARTRIDGES)
def test_loaded_regulated_speed_and_eleven_watt_band(name):
    preset = CARTRIDGES[name]
    # Full rated speed must retain torque under load (graph endpoint, about 1/3
    # stall), unlike a bare DC model that gives zero torque at its free speed.
    assert available_torque(preset, preset.nominal_speed) == pytest.approx(preset.stall_torque / 3)
    powers = [available_torque(preset, preset.nominal_speed * n / 1000) * preset.nominal_speed * n / 1000
              for n in range(1001)]
    assert 12.6 <= max(powers) <= 12.75
    assert .27 <= sum(power >= 11 for power in powers) / len(powers) <= .34
    # Continuous rating and observed graph peak are deliberately distinct.
    assert preset.rated_power == 11
    assert preset.peak_power == 12.75


@pytest.mark.parametrize("name", CARTRIDGES)
def test_reduced_current_limit_caps_stall_torque(name):
    preset = CARTRIDGES[name]
    motor = {**preset.to_dict(), "current_limit": 1.25, "max_power": preset.peak_power}
    assert pwm_torque(motor, 1, 0) == pytest.approx(preset.stall_torque * .5)
    motor["current_limit"] = 0
    assert pwm_torque(motor, 1, 0) == 0


def test_external_gearing_preserves_power_and_does_not_change_cartridge():
    preset = CARTRIDGES["blue"]
    shaft_speed = preset.nominal_speed * .65
    torque = available_torque(preset, shaft_speed)
    speed, output = external_output(shaft_speed, torque, 3, .9)
    assert speed == pytest.approx(shaft_speed / 3)
    assert output == pytest.approx(torque * 3 * .9)
    assert speed * output == pytest.approx(shaft_speed * torque * .9)
    assert preset.rpm == 600
    assert cartridge_for(600) is preset


def test_partial_pwm_and_braking_obey_direction_and_current_bound():
    motor = CARTRIDGES["green"].to_dict()
    for speed in (-30, -10, 0, 10, 30):
        for duty in (-1, -.5, 0, .5, 1):
            torque = pwm_torque(motor, duty, speed)
            assert abs(torque) <= 1.05
            assert torque == pytest.approx(-pwm_torque(motor, -duty, -speed))
        assert pwm_torque(motor, 0, speed) * speed <= 0
