# Local validation

Validated on September 10, 2026, on Windows with Intel Iris Xe graphics.

The local environment uses Python 3.11.16, conda-forge Bullet 3.25 (Python
distribution version 3.2.5), NumPy 2.4.6, PySide6 6.8.3, VTK 9.6.2, and
LLVM-MinGW Clang 22.1.8. The GPU reports OpenGL 4.5.

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
