# PID tuning fixes in 0.1.10

This update corrects continuous motor encoders in PhysX, makes the default X-drive easier to characterize, and exposes the evidence and limitations behind candidate gains. A successful response fit is not a successful robot tuning result. See the [PID tuning guide](pid-tuning.md) or the tuner's **How to use** tab for the workflow.

## Continuous encoders and measured physics

The pinned PhysX runtime represents an unlimited revolute joint with an angle that wraps by 4π radians. With the default robot's 2:1 external motor ratio, a wheel crossing the boundary produced a false −1440° motor-encoder jump after approximately +720° of accumulated motor rotation. Repeated trials could therefore trigger the tuner's discontinuity check even without a tare.

The adapter now tracks winding at each completed physics tick for continuous articulation joints. Native joint tensors and physical forces are unchanged; finite joints, motor ratios and reversals retain their existing behavior. Reset creates fresh winding state. The discontinuity and model-quality rejection thresholds were not relaxed.

Before/after full-scene CPU PhysX runs matched all 169 captured samples and 337 physics ticks for robot pose, RPM, voltage commands and sensor timestamps. Only encoder positions changed, by 0° or +1440° as appropriate. Detected wrap discontinuities fell from four to zero. The corrected long capture fitted at R² 0.9653 and normalized free-running error 8.63%.

## Default drivetrain trial and controller limits

The old 3000 mV / 2 s pulse can exhaust the default 180° travel bound before recording enough baseline, powered response and coast data. **Short drivetrain pulse** selects 2000 mV for 0.2 s without starting motion or changing the travel bound. Characterization pulse length and PID validation interval are now separate settings.

A local 120 Hz CPU PhysX trial used the complete shipped claw robot, full Override field and preload. The short pulse recorded 61 samples over 0.9917 s, including 0.4833 s of observed coast. Maximum individual motor travel was approximately 160.7°. The first-order fit passed unchanged criteria: R² 0.8618, normalized free-running error 11.04%, and holdout error 2.21 RPM. A sanitized [recorded capture with source hashes](../tests/fixtures/tuning_xdrive_physx_short.json) is a regression fixture; this profile is a starting trial, not a universal setting for every mechanism.

For a 90° motor-axis target, 2000 mV limit, actual 16.6666667 ms controller interval, 2 s validation interval and 180° per-motor bound, the fitted-model search proposed Kp 361.99, Ki 0 and Kd 13.09. Its nominal prediction settled in 0.35 s, but **the subsequent native full-scene validation did not pass**: at 1.15 s into the trial, one motor reached 182.71° and the travel guard stopped the experiment. The projected axis was 100.49°, leaving −10.49° target error. The requested control interval and coast observation were incomplete; no native settling time is established.

Repeating validation with a 30° target also hit the unchanged per-motor travel guard, at 1.45 s: the projected axis was 31.13° (−1.13° error), while one motor reached 183.09°. Reducing this step target alone did not produce a validated controller.

This is a concrete limitation of the single-axis fitted model: a projected drivetrain response can hide unequal wheel motion and contact/load effects. The travel guard remains active. The validation harness's `status: passed` means execution and capture completed; its `termination: travel_limit` does not mean controller quality passed. These gains remain candidates and are not recommended as validated robot settings.

## Workflow and diagnostics

- The target list can derive a separately labelled X-drive from explicit robot bindings and conventional URDF wheel geometry, including motor overrides, reversal and external ratios. Ambiguous or steered layouts require manual mapping; this fallback is not verification of a named C++ drivetrain.
- The tuner uses worker-confirmed applied robot settings. Reusing unchanged settings before Start preserves passive capture; a new run gets a fresh capture identity. A real scene change ends the old capture.
- Discontinuity errors report the interval, projected mismatch, expected motion, worst motor-port differences and available feedback age. Identification errors show measured sample coverage and fit errors with specific retry steps.
- Reports warn about truncated or missing coast data, extrapolation above observed voltage, output saturation and predictions that do not settle. A final zero-voltage sample is not a measured coast-down.
- Gains use motor degrees or motor RPM and millivolt output, with seconds in integral/derivative terms. For example, Kp 202 mV/degree requests 18,180 mV at 90° error and clips to a 3000 mV limit outside approximately 14.85°. The number alone does not establish controller quality. Zero I/D terms can be legitimate search results; validate the response rather than judging the magnitude alone.

All native results above are local **CPU PhysX**, without a renderer, GPU run or physical robot. They do not establish GPU behavior, hardware transfer, or successful default X-drive closed-loop tuning.

## Verification provenance

The recorded fixture carries the robot, field and runtime source hashes. The local native verification artifacts have these SHA-256 values:

| Artifact | SHA-256 | Meaning |
| --- | --- | --- |
| `wrap-comparison.json` | `863b51261d7580ac85a6d2d9befd9fdcd2cf45a2818fb68a6295400fdcf4bde0` | Before/after exact-trajectory and encoder comparison |
| Corrected `physx_world.py` used for native runs | `3664841e8b1dcd4f132d04418bd54c4d70c18151954eca826d33e43da3b56405` | Adapter implementation tested |
| `tuning_xdrive_physx_short.json` | `15a4ccb00cccb94335fd2627e5e7978362c310b60d126490817d64940407e4d6` | Committed motor-only characterization fixture |
| Native 90° `report.json` | `0cae0056529cf71e9c08d62f112da870fc7de66c027e50baec4dbcbd50940889` | Candidate settings, guard termination and measured result |
| Native 90° `validation.jsonl` | `186d1e2faf2f3455e85dfdc6206225e5cd080b405422bd6ca43b38008f90871a` | Recorded native validation motor samples |
| Native 30° `report.json` | `8823aec53be37bace4ea5b8cf736f5bed9091d727c0a6d6c44e504fa0c772318` | Smaller-target trial also stopped by the unchanged guard |

Final targeted regression checks passed: **426 passed, 2 platform-specific skips** across tuning, continuous encoders, GUI, capture, engine, reset, replay and native bridge tests. The supervised run owned 36 processes and left none running. Offscreen GUI review at 1120 × 700 found no horizontal overflow. The release verification report records installer hashes and frozen/source package checks. These tests do not replace the native or physical validation limits above.
