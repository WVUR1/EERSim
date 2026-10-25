# MuJoCo physics

EERSim uses MuJoCo 3.17.0 CPU physics by default in this source revision.
Choose **Field → Physics engine → MuJoCo · CPU**, then **Apply settings**, to
require this engine explicitly. **Auto · MuJoCo CPU** reports a Bullet fallback
if the dependency is unavailable or scene construction fails. Saved profiles
with an explicit Bullet or PhysX selection retain their existing choice.

Run `setup.ps1` again after updating a Windows checkout. The portable Conda
environment and pip project dependencies both include the pinned MuJoCo
version. Windows bundle inputs collect MuJoCo's native libraries, resources
and package license metadata. The GUI continues to select OpenGL graphics
independently; CUDA is not required for MuJoCo CPU physics.

```sh
python -m eersim --headless --demo claw --physics-backend mujoco --seconds 30
```

## Shared robot workflows

The native PROS program, competition callbacks, motor ports, gear cartridges,
torque/current/power limits, external gear ratios, sensor conventions, noise
and latency settings use the existing EERSim hardware model. Motor commands
apply bounded forces or torques through MuJoCo; they do not prescribe robot
poses or wheel motion. The omniwheel model still derives traction from actual
contact loads and slip and applies an equal, opposite force to movable objects.
The importer retains the shared effective joint damping, including the smaller
omniwheel value. Body drag applies dissipative forces and inertia-weighted
torques using the existing PhysX rates (0.02 for robot links, 0.04 for field
objects). This is linear viscous drag and differs from Bullet's nonlinear
damping implementation.

The URDF motor preview also uses MuJoCo. Main-view rendering, replay capture,
reset and the PID tuning window consume the authoritative backend state.
Bullet remains an asset/metadata importer and the optional TinyRenderer
geometry shadow. It does not integrate a second physics simulation in
MuJoCo mode. Python's `pybullet build time` startup line can therefore still
appear even when the reported physics engine is MuJoCo.

## Geometry and contact behavior

The importer preserves explicit link mass/inertia, inertial frames, URDF joint
origins and axes, initial joint positions, and the field's collision leaves.
Continuous, revolute, prismatic and fixed joints are supported. Unsupported
joint types and URDF mimic/safety/calibration features produce an error;
they are not silently converted to fixed joints. Mesh paths may be relative
to their URDF or absolute local paths; package URIs must first be resolved.

Static concave OBJ/binary-STL collision meshes use rigid MuJoCo flex surfaces
so goal openings remain open. Moving boxes use an exact eight-corner convex
mesh when the scene has these surfaces, working around a MuJoCo 3.17
box/triangle contact miss without changing cuboid bounds or inertia.
Rigid-flex vertices are attached to a fixed body; they do not deform. Duplicate vertex coordinates can be welded without
removing triangles or changing the surface. MuJoCo requires a positive
surface radius, set to **1 micrometre**. Moving collision meshes retain the
source's separate convex leaves. Moving concave triangle meshes are rejected;
provide a convex decomposition for those links.

Sliding friction follows the configured material-product convention, including
the GUI's plastic/plastic and plastic/tile coefficients. Rolling and spinning
friction use the imported coefficients. MuJoCo has compliant contact constraints,
so matching these inputs does not make its contact trajectories identical to
Bullet or PhysX. Restitution is mapped approximately to contact damping; actual
rebound also depends on timestep, compliance and impact conditions.
The damping mapping holds stiffness constant to avoid injecting impact energy. It is not
a calibrated real VEX material measurement.

## Timing and CPU workers

Every physics tick advances exactly `1 / physics_hz` seconds. Rendering FPS,
replay and wall-clock backlog do not enlarge the timestep or discard physics
ticks. Solver iterations set a Newton-solver iteration limit; the solver can
converge before that limit. Equal iteration counts in different engines do
not imply equivalent physical accuracy.

Use the default **240 Hz** for the claw experiment. In the measured 120 Hz
run, the unchanged program lifted and carried the pin, but it slid roughly
2 cm inside the jaws and hit the goal rim on release. The strict placement
check failed. The isolated exact-goal fixture succeeded at 240 and 480 Hz.
The positive contact time constant is bounded by `max(0.005, 2 * dt)` before
restitution scaling, so rate changes affect both integration and stabilized
contact compliance. These checks do not prove convergence of an unchanged
compliant model, nor do they calibrate friction against a physical robot.
More solver iterations do not by themselves remove this rate dependence.

**Physics CPU workers · advanced** enables native within-step collision and
constraint work. The default is two workers; zero runs inline. The request is
bounded by available CPU capacity and the actual count is reported in physics
status. More workers can be slower. Dependent simulation ticks and PROS
callbacks still execute in order.

Version 0.1.12 enables MuJoCo's native sleeping islands. Passive props start
awake and settle under the normal solver; the native sleep tolerance remains
unchanged. Robot mechanisms remain awake so motors respond immediately,
including separate mechanisms attached beneath a fixed base. Impacts and real
external forces wake sleeping pieces. Fixed field geometry remains available
for collisions and does not count as a movable group.

Body drag uses the same COM force and inertia-weighted torque as before, now
applied through the passive-force path. A tiny drag force no longer counts as an
external disturbance that prevents sleep. The stock native step pipeline handles
waking and sleeping; the earlier custom static-flex cache is bypassed in this
mode. Masses, collision geometry, friction, timestep, and solver limits are unchanged.

The physics status shows **Awake groups**, **Sleeping groups**, and **Active
contacts**. A group is an independent movable object or connected mechanism;
an awake group can still look stationary. Sleep is based on the solver's motion
criteria, not distance from the robot. Moving or jittering objects keep being solved.

MuJoCo omits settled contact pairs from its active contact reports. Scoring
retains previously observed contact presence only while both movable parties
remain asleep and the exact link poses are unchanged. Waking, movement, and
Reset invalidate that evidence. No cached contact force is reused or reported.
Native instability/contact-buffer warnings still stop the run with an error;
a numerical reset cannot appear as a successful simulation.

## Reproduce measurements

Run these sequentially on the same machine with other heavy workloads idle:

```sh
python tools/benchmark_integration.py --backend bullet --physics-hz 120 --output results/bullet-120.json
python tools/benchmark_integration.py --backend mujoco --physics-hz 120 --output results/mujoco-120.json
python tools/benchmark_integration.py --backend mujoco --physics-hz 240 --max-wall-seconds 600 --output results/mujoco-240.json
```

These execute the unchanged native claw program and retain the complete field.
Add `--cpu-threads 0`, `2`, `4`, or `8` to compare worker counts. Omission
retains the configuration value. Use `--no-pros --steps 480 --validation smoke` at 240 Hz for a passive
worker comparison; that does not validate autonomous scoring.
A successful full run must physically lift the preload, retain it in the intended
red goal, release robot contact, complete the controller route, and disable at
the end of the mode. A timeout or missed goal remains a failed result even when
its measured throughput is useful. JSON reports include hardware/package data;
adjacent CSV and logs preserve tick timing and trajectory evidence.

Step-only real-time factor includes the PROS exchange, shared robot model and
physics. Loading, compilation, report writing and live rendering are measured
separately or excluded as identified in the report. Headless results do not
establish desktop FPS or performance on a different computer.
