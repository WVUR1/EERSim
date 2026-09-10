"""VEX V5 11 W physical cartridges and a published-curve motor approximation.

VEX documents 100/200/600 rpm, red 2.1 N·m stall, 36/18/6 internal ratios,
1800/900/300 encoder ticks/rev, a 2.5 A limit, and an 11 W rating. Green/blue
stall torques below are inferred by ideal gear-ratio scaling from the red shaft.
The performance graph shows about 12.75 W peak despite the 11 W table rating.

The graph points below are approximate hand digitization of VEX's RED torque
curve, not unpublished firmware constants or an independent motor measurement.
Partial-voltage/back-EMF, controller gains and current estimation are models.
No thermal derating, battery sag, motor inductance or individual unit variation
is asserted. Sources checked September 10, 2026:
https://kb.vex.com/hc/en-us/articles/360060929971-Understanding-V5-Smart-Motors
https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance
https://kb.vex.com/hc/article_attachments/43551360496660
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any

SPEC_SOURCE = "https://kb.vex.com/hc/en-us/articles/360060929971-Understanding-V5-Smart-Motors"
CURVE_SOURCE = "https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance"
CURVE_IMAGE = "https://kb.vex.com/hc/article_attachments/43551360496660"


@dataclass(frozen=True)
class Cartridge:
    name: str
    label: str
    internal_ratio: int
    rpm: int
    stall_torque: float
    ticks_per_rev: int
    rated_power: float = 11.0
    peak_power: float = 12.75
    current_limit: float = 2.5
    controller_period: float = 0.01

    @property
    def nominal_speed(self) -> float:
        return self.rpm * math.pi / 30

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


CARTRIDGES = {
    "red": Cartridge("red", "Red · 36:1 · 100 rpm", 36, 100, 2.1, 1800),
    "green": Cartridge("green", "Green · 18:1 · 200 rpm", 18, 200, 1.05, 900),
    "blue": Cartridge("blue", "Blue · 6:1 · 600 rpm", 6, 600, .35, 300),
}

# (fraction of software-limited cartridge speed, measured-graph red torque Nm)
RED_TORQUE_CURVE = (
    (0.00, 2.10), (0.20, 2.10), (0.40, 2.08), (0.50, 2.07),
    (0.55, 2.05), (0.60, 2.00), (0.65, 1.87), (0.70, 1.65),
    (0.75, 1.50), (0.80, 1.30), (0.85, 1.13), (0.90, .94),
    (0.95, .81), (1.00, .70),
)


def cartridge_for(value: str | int | float = "green") -> Cartridge:
    if isinstance(value, str) and value.lower() in CARTRIDGES:
        return CARTRIDGES[value.lower()]
    try:
        rpm = float(value)
    except (TypeError, ValueError):
        rpm = math.nan
    for preset in CARTRIDGES.values():
        if rpm == preset.rpm:
            return preset
    raise ValueError(f"Unknown physical V5 cartridge {value!r}; choose red, green or blue")


def torque_fraction(speed_fraction: float) -> float:
    """Full-voltage current-limited envelope; loaded nominal speed has torque.

    The speed governor is separate. Returning a nonzero envelope at 100% is
    necessary to represent VEX's specified ability to sustain speed under load.
    """
    value = min(1.0, abs(float(speed_fraction)))
    for (x0, y0), (x1, y1) in zip(RED_TORQUE_CURVE, RED_TORQUE_CURVE[1:]):
        if value <= x1:
            return (y0 + (y1 - y0) * (value - x0) / (x1 - x0)) / 2.1
    return RED_TORQUE_CURVE[-1][1] / 2.1


def available_torque(cartridge: Cartridge | str, speed_rad_s: float, *,
                     stall_torque: float | None = None, current_limit: float | None = None,
                     peak_power: float | None = None) -> float:
    """Maximum motoring shaft torque at a speed (N·m), before external gearing."""
    preset = cartridge if isinstance(cartridge, Cartridge) else cartridge_for(cartridge)
    stall = preset.stall_torque if stall_torque is None else stall_torque
    current = preset.current_limit if current_limit is None else current_limit
    peak = preset.peak_power if peak_power is None else peak_power
    envelope = stall * min(torque_fraction(speed_rad_s / preset.nominal_speed), current / preset.current_limit)
    return min(envelope, peak / max(abs(speed_rad_s), .001))


def pwm_torque(motor: dict[str, Any], duty: float, speed_rad_s: float) -> float:
    """Approximate raw-PWM shaft torque, calibrated to the full-voltage graph.

    A current-limited DC/back-EMF approximation extends the measured full-voltage
    envelope to partial PWM and braking. It is intentionally not called an
    exact reproduction of V5's proprietary current/speed control electronics.
    """
    nominal = float(motor["rpm"]) * math.pi / 30
    speed = speed_rad_s / nominal
    stall = float(motor["stall_torque"])
    current_fraction = float(motor.get("current_limit", 2.5)) / 2.5
    gain = float(motor.get("pwm_torque_gain", 2.0))
    absolute_speed = abs(speed)
    if absolute_speed < .55:
        emf = absolute_speed / .55 * (gain - torque_fraction(.55))
    else:
        emf = gain - torque_fraction(absolute_speed)
        if absolute_speed > 1:
            emf += 1.5 * (absolute_speed - 1)
    emf = math.copysign(emf, speed)
    raw = gain * max(-1., min(1., duty)) - emf
    torque = stall * max(-current_fraction, min(current_fraction, raw))
    if torque * speed_rad_s >= 0:
        torque = math.copysign(min(abs(torque), stall * torque_fraction(speed)), torque)
    peak = float(motor.get("max_power", 12.75))
    torque = max(-peak / max(abs(speed_rad_s), .001), min(peak / max(abs(speed_rad_s), .001), torque))
    return torque


def pwm_no_load_speed(motor: dict[str, Any], duty: float) -> float:
    """Invert the inferred back-EMF curve, bounded by the firmware RPM cap."""
    gain = float(motor.get("pwm_torque_gain", 2.0))
    magnitude = min(1., abs(duty))
    knee = torque_fraction(.55)
    target = gain * magnitude
    if target <= gain - knee:
        fraction = .55 * target / (gain - knee)
    else:
        residual = gain - target
        fraction = 1.0
        for (x0, y0), (x1, y1) in zip(RED_TORQUE_CURVE, RED_TORQUE_CURVE[1:]):
            if x0 >= .55 and y1 / 2.1 <= residual <= y0 / 2.1:
                fraction = x0 + (x1 - x0) * (y0 / 2.1 - residual) / ((y0 - y1) / 2.1)
                break
    return math.copysign(fraction * float(motor["rpm"]) * math.pi / 30, duty)


def external_output(shaft_speed: float, shaft_torque: float, gear_ratio: float,
                    efficiency: float = 1.0) -> tuple[float, float]:
    """Motor revs/output rev: reduction divides speed and multiplies torque."""
    if gear_ratio <= 0 or not 0 < efficiency <= 1:
        raise ValueError("External ratio must be positive; efficiency must be in (0, 1]")
    return shaft_speed / gear_ratio, shaft_torque * gear_ratio * efficiency
