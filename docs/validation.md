# Local validation

Validated on September 10, 2026, on Windows with Intel Iris Xe graphics.

The local environment uses Python 3.11.16, conda-forge Bullet 3.25 (Python
distribution version 3.2.5), NumPy 2.4.6, PySide6 6.8.3, VTK 9.6.2, and
LLVM-MinGW Clang 22.1.8. The GPU reports OpenGL 4.5.

Local setup and Ubuntu CI both create the environment from `environment.yml`,
which pins conda-forge Bullet 3.25 and supplies the test dependencies. Install
the checkout with `python -m pip install -e . --no-deps` after activating that
environment so pip preserves its solver build. CI records the installed
versions and checks the conda Bullet package before running the full tests.
Its [official micromamba action](https://github.com/mamba-org/setup-micromamba)
activates this environment for every command. Different Bullet releases and
platform builds can change contact settling; a broad pip dependency range is
not a reproducible substitute for the validated environment.

## Code and physics

The separate EERKit workspace's 11 existing C++ translation units compile without edits.
Integration checks run its controller code through the native adapter, move the
robot through wheel contact, enforce the competition time limit, disable outputs,
and export telemetry. Separate checks exercise task scheduling and watchdogs,
serial odometry conventions, motor reversal, physical gear cartridges, external
reductions, current limits, collisions, airborne wheels, sensor noise, latency,
dropout, and repeatability across render rates.
An initialization regression compiles a feedback-driven homing routine and
requires working motor and pneumatic output in both startup callbacks, followed
by the exact requested number of competition ticks.

The claw tests require physical contact to grip and lift a pin, release it into
the measured goal socket, and exclude a regripped pin from the prototype
released-placement indicator. They do not use a grasp attachment constraint.
Loaded contact-surface checks verify the pin taper against the CAD, including
the small collision margin. Further checks cover rolling and falling energy,
resting pieces waking on impact, continued slow rolling, and contact chains
that must remain active while touching the robot.

## Graphics and desktop

The original detailed CAD meshes render through VTK/OpenGL. In a 90-object CAD
benchmark at 960 by 720 pixels, steady GPU frames took 19.5–24.9 ms; CPU frames
took approximately 1.5–1.8 seconds. The complete simulator world, including robot
and overlay geometry, rendered in 39–68 ms per steady frame. These are graphics
measurements, not claims about complete simulation throughput. Initial asset
loading and the first GPU frame are slower.

Independent geometry checks compare renderer transforms against Bullet for
offset and rotated inertia frames, articulated links, visual origins, mesh
scaling, capsule axes, and material colors. GPU failure also has a tested CPU
fallback that leaves sensor observations unchanged.

The desktop was exercised at 1366 by 700 and 1500 by 950 pixels. Workspace build,
competition controls, pause/resume/step/reset, keyboard control, camera orbit,
pan and zoom, import configuration, and live motor jogging were checked.
Screenshots in this directory show the running application.

The [sneak-peek video](sneak-peek.mp4) is a 34.8-second H.264 recording at
1280 by 848 pixels and 20 FPS. All 696 frames decoded successfully. It shows
the actual GUI and held motor controls, followed by the native PROS claw demo.
That segment records every 12 fixed 240 Hz physics ticks per video frame, giving
one times simulation-speed playback. The capture required a released preload
placement, the native completion message, and two further simulated seconds.
The closing camera shot holds the resulting state. It is not a wall-time
performance demonstration. A stale status-footer message visible in the capture
was subsequently corrected; the simulation and top-level state were valid.

## Route iteration throughput

The original EERKit source completed four simulated seconds of driver control
in 3.33 wall seconds on the practice field (1.20×) and 4.45 wall seconds on the
optional fixed-piece Override layout (0.90×). These headless measurements include
physics, native code and initialization, and exclude compilation, scene loading
and rendering. Two full-field validation/recording processes were running
concurrently. The fixed-piece layout was checked to contain exactly 73 immovable
cups and pins; its changed contact behavior is intentional and clearly labeled
in the GUI. These results do not represent full dynamic-field performance.

The full dynamic Override scene is substantially slower: a two-simulated-second
interval after the recording process closed took 107.8 wall seconds, about
0.019× real time. Many complex cup/pin contacts remained numerically active even
after the scoring robot had stopped. The physics timestep and solver iterations
were preserved. This is a material limitation of the current CPU implementation;
GPU graphics does not remove that contact cost. Use the fixed-piece layout for
route iteration and the full scene when movable game-piece interactions matter.

## Full-field physics profile

A headless profile on an Intel Core i7-1185G7 with the Windows environment above
measured 240 ticks of the loaded claw scene, with no robot program running.
Scene loading was excluded. One simulated second took 50.85 wall seconds
(0.0197x). `pybullet.stepSimulation` accounted for 49.06 seconds, or 96.5% of
the measured time; 2,753 contact points remained after that interval. This is
a settling-field profile, not a complete native claw run or a measurement of
the user's Ryzen/GeForce machine.

The field has 36 cups with 64 convex collision leaves each, plus 37 pins with
seven leaves each, before the demo preload. A short native Bullet trace shows
substantial work in both collision detection and constraint solving. The
installed PyBullet build provides serial CPU dynamics; VTK/OpenGL accelerates
graphics only. Python threads cannot parallelize this world's causal physics
ticks. Separate processes can run independent experiments concurrently, but
that does not accelerate a single field's timeline.

An experimental maximal-coordinate prop representation improved the short
profile only to 0.0256x and changes how leaf collision margins are applied.
It was not adopted. Alternative SAT contacts and larger solver batches did
not establish a useful improvement. The production timestep, 100 solver
iterations, collision geometry, and rest thresholds remain unchanged.

GPU dynamics would require a backend adapter and renewed contact, joint, motor,
and sensor validation. The [PhysX GPU backend](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/GPURigidBodies.html)
supports CUDA physics and articulations on Windows, but does not guarantee a
particular speedup for this scene. This release does not implement that backend.

The source GUI now measures RTF using the wall-clock timestamp after its counted
ticks finish and resets the measurement across lifecycle changes. This corrects
reporting; it does not speed up the physics solver.

For comparable measurements from a source environment, run:

```powershell
python tools/benchmark_physics.py --ticks 240 --max-wall-seconds 120 --json .sim-tools/physics-benchmark.json --profile .sim-tools/physics-benchmark.prof
python -m pstats .sim-tools/physics-benchmark.prof
```

The benchmark opens no windows and runs no robot code. Add `--practice` to
measure the simpler field or `--config experiment.json` for a saved scene.
Its default 30-second wall budget stops at a complete tick and reports the
actual number of measured ticks. Load/warmup time is excluded; profiling adds
overhead. Do not run competing benchmarks at the same time.

## Installed package

A wheel was installed into a separate environment and tested from a directory
outside the source tree with `PYTHONPATH` cleared. It found all 20 bundled URDFs
and their mesh references, loaded the default field, rendered on Intel Iris Xe,
and compiled and initialized the packaged PROS claw example. No manually
configured native DLL search path was needed.

## Limits of these checks

No measurements from the team's physical robot were supplied. Motor behavior
uses the published VEX curves with documented controller approximations; robot
mass distribution, contact friction, piece masses, and simplified collision
surfaces still need calibration. Passing simulation tests does not establish
agreement with a physical robot. The placement indicator is a mechanism-testing
metric and does not implement official scoring or all game rules.

An Ubuntu CI workflow is included; the local results above are Windows results.
The [Linux CI run on September 10, 2026](https://github.com/WVUR1/EERSim/actions/runs/34541525808)
finished with 112 passed, 12 skipped and two failures in the existing bounded-rest
tests. A pin in a cup and a pin in a goal retained small velocities where the
tests required exact native sleep after eight simulated seconds. These failures
persisted after verifying the same conda-forge Bullet 3.25 release used locally.
Their cause remains undiagnosed; no contact thresholds or assertions were
relaxed, and the Linux suite is not currently passing.
