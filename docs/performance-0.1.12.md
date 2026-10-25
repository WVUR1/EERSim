# Idle physics regression: 0.1.12

Measured October 25, 2026. Version 0.1.11 kept all MuJoCo objects awake,
including resting field pieces. Version 0.1.12 enables native sleep/wake and
shows awake groups, sleeping groups, and active contacts in the GUI. This
reduces idle cost, but **MuJoCo remains below realtime and slower than PhysX
CPU on this field**. The 0.1.11 speedup was against Bullet, not PhysX.

For the current field, try **Field → Physics engine → PhysX · CPU** and
**Physics CPU workers · advanced → 0 · inline**, then **Apply settings**.
The Windows installer includes the required runtime. Source installs need
the optional `.[gpu]` dependencies even for PhysX CPU. Keep 240 Hz for the claw.
An explicit engine selection persists in saved profiles; Auto continues to
select MuJoCo. GPU rendering is a separate setting.

## Matched idle measurements

All runs used the same A100 server's two AMD EPYC 7H12 CPUs (128 physical /
256 logical cores), CPython 3.12.3, the complete Override field, unchanged claw
robot, 240 Hz, and a 100-iteration limit. These are **CPU physics** runs with
CUDA visibility disabled, no graphics, no replay and no PROS commands.
MuJoCo is 3.17.0; PhysX bindings are 0.5.11. Runs were sequential with the same
diagnostic tool overlay. Native diagnostic queries were outside timed steps;
profiling inside the step was enabled consistently.

The first column measures all 480 steps, including settling. The last column
measures only ticks 241–480, after most pieces settled. Loading and report/CSV
writing are excluded. Each row is one run, not a confidence interval.

| Engine | CPU workers | First 2 simulated seconds | Settled interval, 1–2 seconds |
| --- | ---: | ---: | ---: |
| MuJoCo 0.1.11 | 6 | 0.09386× | 0.09163× |
| MuJoCo 0.1.12 | 6 | 0.11434× | 0.12628× |
| PhysX CPU | 6 | 0.09856× | 0.38211× |
| PhysX CPU | 0 (inline) | 0.15009× | 0.44317× |

The MuJoCo fix improves the two-second average by 21.8% and the measured
settled interval by 37.8%. At two seconds, the old version had all 79 groups
awake and 3,518 active contacts. The fixed version had **74 sleeping, 5 awake,
and 208 active contacts**; all five preplaced pins remained scored. Six-worker
PhysX had 71 of 78 dynamic props asleep. Engine group/prop counts use different
native definitions and should not be treated as identical object counts.

An additional old-MuJoCo run with two workers measured 0.07449× over two
seconds. It has no matched two-worker candidate run and is not used for the
improvement ratio. More workers did help old MuJoCo, but inline PhysX was
faster than six workers in this experiment. These server results do not
predict Ryzen desktop speed or interactive viewport FPS.

## What changed, and what remains expensive

Passive pieces start awake and settle naturally. Robot mechanisms stay awake,
including branches under a fixed base. Impacts and actual external forces
wake sleeping pieces. Drag retains the same COM force and inertia-weighted
torque, applied through the passive-force path so tiny damping forces no
longer prevent sleep. Geometry, masses, friction, timestep, motor limits,
solver limits, and the native sleep tolerance are unchanged.

The stock MuJoCo step pipeline handles sleep/wake. Sleeping pieces stop
generating settled contacts, but the fixed field's rigid-flex surfaces still
pay vertex, bounds and BVH updates in the native pipeline. The candidate's
first two seconds spent an average 29.68 ms per tick in `mj_step1`, including
material adjustment, compared with 0.72 ms in `mj_step2`. Thus the bottleneck
is not simply how many objects remain awake. The release keeps the standard
MuJoCo wheel and all source collision triangles; it does not use a custom
native engine or approximate the field to obtain these results.

Scoring retains verified contact presence only while the relevant movable
groups remain asleep and exact link poses are unchanged. Wake, motion and
Reset invalidate that evidence. Cached contact forces are never returned
or reused. The GUI's active-contact count excludes this retained evidence.

## Validation and reproduction

The complete candidate autonomy passed at 240 Hz with six workers: 30
competition seconds took **241.10 seconds of measured stepping (0.12443×)**.
The unchanged native PROS program lifted the preload 0.19327 m, retained it
in `goal_red_5`, released contact, retreated, and disabled normally. The final
scene had six placed pins and 74 sleeping groups. Loading, native compilation,
25 initialization ticks and observation/diagnostic queries are excluded from
that competition stepping measurement.

The matched 0.1.11 run also passed, taking **333.18 seconds (0.09004×)**
with the same six workers, 240 Hz, field and controller. The fix therefore
increased full-run throughput by **38.2%**, or reduced measured stepping
time by **27.6%**. These ratios compare complete successful autonomy runs,
not only selected faster intervals.

PhysX CPU inline also passed the complete unchanged 30-second routine at
**0.44279× realtime**, 3.56 times the fixed MuJoCo throughput in this test.
This compares the tested engine/worker configurations (PhysX inline versus
MuJoCo six workers); it is not a claim about identical solver behavior or all
robots. All three full runs passed the same strict physical placement checks.

The combined changed-feature suite passed **173 tests, with one optional test
skipped**. The separate unchanged native 240 Hz claw integration passed.
Tests cover natural sleep, moving pieces remaining awake, robot response,
impact/external-force wakeup, scoring retention and removal, Reset, passive
drag through sleep transitions, replay, GUI diagnostics and release payloads.
An independent comparison against a native passive-force callback matched
free-flight states exactly and contact states to numerical roundoff.

The latest preceding CI run had two legacy Bullet exact-sleep failures and a
stale notice test. This release updates the notice test to verify the added
PyOpenGL license as well as Qt; both notice tests pass. The two unchanged
Bullet assertions also fail on the previous source and have not been weakened.

The unsigned Windows EXE passed a separate frozen run using isolated user
data and no external Python/compiler on PATH. It compiled the bundled PROS
source, advanced 505 fixed ticks (including initialization), reported native
sleeping, and disabled normally. This headless check does not establish
interactive viewport FPS or verify the installer wizard on another machine.

```sh
python tools/benchmark_integration.py --backend mujoco --physics-hz 240 --cpu-threads 6 --no-pros --steps 480 --validation smoke --diagnostics-seconds 1 --output results/idle-mujoco.json
python tools/benchmark_integration.py --backend physx_cpu --physics-hz 240 --cpu-threads 0 --no-pros --steps 480 --validation smoke --diagnostics-seconds 1 --output results/idle-physx.json
python tools/benchmark_integration.py --backend mujoco --physics-hz 240 --cpu-threads 6 --seconds 30 --max-wall-seconds 900 --diagnostics-seconds 1 --output results/claw-mujoco.json
```

Run sequentially on an otherwise idle machine. See the
[compact evidence record](mujoco-0.1.12-results.json) for source hashes, raw
capture hashes, activity samples and validation outcomes, and the
[integration notes](mujoco.md) for model limitations. The earlier 120 Hz
moving-grasp limitation remains; this release does not establish hardware
calibration or realtime performance.
