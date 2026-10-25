# MuJoCo migration: 0.1.11 measurements

Measured on October 25, 2026. MuJoCo 3.17.0 is now the default CPU physics
backend. The shared PROS bridge, motor cartridges, sensor model, rendering,
replay and PID workflows remain in place. Bullet and PhysX are still selectable.
Saved profiles with explicit engines retain their selections.

## Controlled throughput comparison

The same Linux host has two AMD EPYC 7H12 CPUs (128 physical / 256 logical
cores) and A100 GPUs. These tests used **CPU physics with CUDA visibility
disabled**. They used CPython 3.12.3, pip PyBullet 3.2.7, MuJoCo 3.17.0,
the complete dynamic Override field, the unchanged claw robot and native PROS
program, a 100-iteration solver limit, and two MuJoCo workers. Graphics were
not rendered. Runs were sequential; these are single measurements, not a
multi-run confidence interval.

Both Bullet runs reached their 300-second wall budget before finishing the
30-second autonomy. For a comparable window, the table uses exactly the first
three autonomous simulation seconds (360 or 720 ticks). The timer surrounds
`Session.step`, including the PROS exchange, motors, tires, sensors and physics.
Loading, compilation and benchmark CSV/report writing are outside that timer.

| Physics rate | Bullet realtime factor | MuJoCo realtime factor | Throughput ratio |
| --- | ---: | ---: | ---: |
| 120 Hz | 0.02512x | 0.15011x | 5.98x |
| 240 Hz | 0.01269x | 0.07448x | 5.87x |

These results are **below realtime**. The ratios compare against Bullet, not
against PhysX. This experiment did not repeat the historical PhysX CPU runs
and does not establish a speedup over an existing PhysX installation or predict
performance on a Ryzen/GeForce computer.

## Full autonomy and rate limitations

The 240 Hz full-field run **passed**: 30 simulated seconds took 413.03 seconds of measured stepping (**0.07263x realtime**). The strict validator requires lift, retention in `goal_red_5`, release of robot contact, normal controller completion and disabled mode at the end.

The 120 Hz run completed all 30 simulated seconds in **200.35 seconds** of
stepping (0.14974x), but **failed physical placement**. The pin lifted, slid
about 2 cm within the jaws during travel, then struck the goal rim and was
ejected on release. A controller message saying “Placement complete” was not
accepted as proof of scoring. That failed trajectory is retained in the record.

Use the default **240 Hz** for the claw. Isolated tests with the same robot,
goal, preload and unchanged native code place successfully at 240 and 480 Hz.
Exact goal/pin drop tests also pass at 120/240 Hz. The 120 Hz limitation is in
the moving grasp. Increasing friction impedance did not fix it; an unstable
NoSlip experiment was rejected, and neither modification was adopted.

MuJoCo's stabilized positive contact time constant is bounded by
`max(0.005, 2 * dt)` before restitution scaling. Changing the rate therefore
changes integration and contact compliance; these observations are not formal
convergence of a fixed compliant model or real-hardware friction calibration.
Friction coefficients, motor effort limits, robot geometry and PROS source were
not changed to make the higher-rate run pass.

## CPU workers and remaining cost

The passive comparison uses 480 ticks at 240 Hz (two simulated seconds), no
PROS commands, identical initial full fields and no rendering. Loading is excluded.

| Native workers | Passive realtime factor |
| --- | ---: |
| 0 | 0.05568x |
| 2 | 0.07344x |
| 4 | 0.08833x |
| 8 | 0.10345x |

Eight workers was the fastest count tested: **41% faster than two**, and 86%
faster than inline execution. Every final field-object pose/velocity and final
robot pose matched exactly across these four runs. This does not prove that
eight is optimal on other CPUs or for live autonomy. The default remains two;
the GUI exposes **Physics CPU workers · advanced**.

The 120 Hz profile spent about **43.5 ms per tick in native collision detection**,
compared with 2.8 ms assembling constraints, 2.6 ms in the second integration
stage and 0.46 ms in the PROS exchange. Scene construction took 28.24 seconds,
including roughly 20.77 seconds compiling MuJoCo geometry. Workspace building
is separate. Static meshes retain all **514,352 source triangles**; welding
identical vertices reduced their storage from 374,376 to 255,856 vertices.
There was no triangle decimation or lowered solver iteration limit.

The cached pipeline skips only invariant rigid-flex geometry work and duplicate
constraint assembly. Collision detection and contact solving still run every
tick, including resting objects. MuJoCo sleeping is disabled in this pipeline.
An independent full-field 100-tick check found exactly zero differences in
positions, velocities, accelerations, solved constraint forces and contacts
against the standard native pipeline, with no warnings.

## Verification and reproduction

The final changed-code test set passed **181 tests**, with one optional test
skipped. The separate opt-in native 240 Hz claw test passed. Native tests cover
torque/current limits, cartridges, gearing, damping, continuous encoders, rays,
contact materials, reset, replay, CPU rendering, PID capture and worker cleanup.

The broad suite passed 934 tests with five skips and the long unchanged Bullet
autonomy test deselected. Its stale in-memory MuJoCo test was rerun successfully
after the final source freeze. Two legacy Bullet exact-sleep assertions failed
and reproduced with the same velocities on the unmodified previous commit
`eeee33779bc230a136c1b00e51541a7246ac3bbd`, using the same Windows interpreter.
These pre-existing assertions were not weakened.

The Windows directory bundle includes MuJoCo 3.17.0, its native bindings and
notices, existing optional PhysX libraries, assets and the native compiler.
A headless frozen run compiled the bundled PROS code, selected MuJoCo, advanced
85 fixed ticks and disabled normally. It is an unsigned preview;
headless validation does not establish interactive viewport FPS or verify the
installer wizard on another machine.

```sh
python tools/benchmark_integration.py --backend mujoco --physics-hz 240 --cpu-threads 2 --max-wall-seconds 600 --output results/mujoco-240.json
python tools/benchmark_integration.py --backend bullet --physics-hz 240 --max-wall-seconds 300 --output results/bullet-240.json
python tools/benchmark_integration.py --backend mujoco --physics-hz 240 --cpu-threads 8 --no-pros --steps 480 --validation smoke --output results/workers-8.json
```

The [compact evidence record](mujoco-0.1.11-results.json) includes source and raw
capture hashes, hardware, timings, outcomes and the cache equivalence check.
See [integration details and limits](mujoco.md) for import behavior and setup.
