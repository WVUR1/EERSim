# PID tuning validation — 0.1.8

Source acceptance was run on Windows on September 14, 2026. This record covers
the new tuning workflow and selected regressions; it is not a full-suite result,
a general physics-accuracy measurement, or a hardware PID calibration.

| Check | Observed result | Scope |
| --- | --- | --- |
| Targeted acceptance | 176 passed, 2 skipped, 0 failures/errors; 178 cases total | Discovery, identification/controller math, GUI behavior, logger preparation/protocol/processes, native PROS bridge, and selected engine/reset/session regressions |
| Unchanged PROS capture | 162 automatic samples; source hash unchanged; signed voltage and motor motion observed | Desktop-compiled fixture with a `MotorGroup({1,-2})`; no logging edits to that fixture |
| Separate simulated characterization and validation | Candidate fitted; 153 validation samples; final recorded motor position 90.0000000000336° for a 90° request | Generated voltage characterization followed by the candidate controller on a Bullet rotor |
| Generated C logger | Compiled against PROS 4.2.2 headers with host-side API stubs | Header/type compatibility and mocked behavior; not an ARM firmware link or a V5 upload |

The two expected skips were a POSIX-only terminal-input contract on Windows and
an existing snapshot-comparison test requiring its separate comparison harness.
The acceptance supervisor owned 34 processes; the native fixture supervisor
owned 10. Both finished with zero remaining descendants.

The native fixture used Bullet CPU at 240 Hz, a fixed robot base, zero gravity,
two green-cartridge motors bound to one continuous rotor joint, and an empty
test arena. The rotor had simple specified inertia and damping. Its generated
characterization supplied the fit; the preceding 162-sample unchanged-PROS
capture was a separate logging check. The fitted model's metrics describe its
prediction, not measured physical-robot settling performance.

GUI checks include grouped drivetrain mapping, cancellation/lifecycle behavior,
capture handling, and retaining a candidate across Reset before validation.
The screenshot in the [user guide](pid-tuning.md) shows illustrative response
data and is not evidence of a physical recording.

The evidence is retained in `final-checks.xml`, `final-checks-process.json`,
`native-final/report.json`, and `native-final-process.json` in the private release
working files. The native report records source hashes for its exercised
bridge, runner, physics, and tuning modules.

No connected V5, ARM firmware upload, physical PID transfer, CUDA tuning run,
full-field tuning benchmark, or interactive installer/upgrade wizard was tested
in these source checks. Installer construction and packaged-artifact audits are
separate release checks; this record does not claim they passed. Candidate gains
still require validation with the intended controller units, load, and real
mechanism.
