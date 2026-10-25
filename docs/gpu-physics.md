# Physics backends

The source application defaults to MuJoCo CPU and also connects Bullet and PhysX to the same native PROS
callbacks, VEX cartridge/gearing model, contact-dependent omni-wheel forces,
pneumatics, and sensor noise/latency model. See [MuJoCo integration](mujoco.md)
for the default engine. The historical PhysX integration described below supplies the live rigid-body
poses, joint state, contact forces, and distance queries. Its robot is a
reduced-coordinate articulation; field mechanisms use regular PhysX joints.
The native robot code does not need a different control API for each backend.

This integration is separate from the earlier passive
[A100 benchmark study](../tools/physx_prototype/README.md). Those measurements
excluded robot controllers, tire forces, sensors, and rendering. They do not
establish the integrated simulator's throughput or accuracy. The integrated
backend has now completed the unchanged PROS claw program on a Linux A100,
including physical grip, lift, navigation, release, and a red-goal placement.
The original 0.1.1 run measured **0.0426× realtime** at 240 Hz and 100 TGS iterations.
This establishes functionality, not a speed advantage over CPU physics.
The Windows preview bundles the runtime and passed headless CPU/fallback checks.
Supplied RTX 4070 Ti captures verify GPU dynamics, GPU broadphase, a valid CUDA
context, and OpenGL rendering on Windows. The 0.1.3 follow-up runs the native PROS
claw demo and records its preload in a red goal at both rates. The 120 Hz case
completes 30 seconds of Skills Auto; the 240 Hz case reaches 16.44 competition
seconds before the 120-second capture budget expires. These observations add
Windows GPU autonomy evidence; the performance capture does not independently
validate grasp forces, lift clearance, or trajectory accuracy. See the
[follow-up analysis](diagnostics-4070ti-followup.md) and the
[earlier neutral baseline](diagnostics-4070ti.md).
See the [measurement record](validation.md#integrated-physx-acceptance).

The [historical A100 matched comparison](diagnostics-a100-iterations.md) runs the
0.1.3 claw at 120 Hz: two 100-iteration runs pass placement at about 0.29x
realtime, while both faster 32-iteration runs physically miss the goal. Solver
iterations remain 100 by default. The report includes repeated trajectory and
performance evidence plus a headless reproduction command.

## Install and select

Normal source setup installs MuJoCo CPU and the OpenGL application;
Bullet remains available for comparisons and CPU rendering. Add GPU physics to
the same environment with `python -m pip install -e ".[gpu]"`; for the private
Windows setup environment, use:

```powershell
& .\.sim-tools\physics-env\python.exe -m pip install -e ".[gpu]"
```

The extra pins `ovphysx==0.5.11`, `ovstage==0.1.1.355824`, and `scipy==1.16.3`.
The paired NVIDIA packages contain the native runtime; a full Isaac Sim or
CUDA toolkit installation is not required. The Windows installer build is
configured to bundle these packages and their licenses by default. An NVIDIA
driver remains a machine prerequisite.

In **Field → Physics engine**, choose **Auto · MuJoCo CPU**, **MuJoCo · CPU**,
**Bullet · CPU**, **PhysX · NVIDIA CUDA**, or **PhysX · CPU**, then click
**Apply settings**. **Auto** selects MuJoCo CPU and falls back to
Bullet with a visible reason if MuJoCo is unavailable or scene initialization fails.
The reason appears in Field, Telemetry, and the log. Explicit engine selections
raise an error on failure. A running scene does not silently change solvers
after a runtime error.

The CUDA probe requires compute capability **7.0 or newer** and a driver API
version of **12080 or newer** (CUDA 12.8 support). It checks the driver directly
through `nvcuda.dll` on Windows or `libcuda.so.1` on Linux. These are the
application's acceptance checks, not a promise of realtime performance.
The pinned runtime has Windows x64 and Linux wheels; its x86-64 wheels also
require an AVX-capable CPU. See the [pinned wheel metadata](https://pypi.org/project/ovphysx/0.5.11/)
and the implemented [device probe](../eersim/physx_worker.py).

**CUDA physics device** is a logical ordinal after `CUDA_VISIBLE_DEVICES`
filtering. For example, a process restricted to one GPU addresses it as device
0 even if `nvidia-smi` lists a different physical index. The equivalent CLI is:

```sh
python -m eersim --headless --demo claw --physics-backend physx_gpu \
  --physics-device 0 --solver-iterations 100 --seconds 30 --csv claw-cuda.csv
```

The JSON keys are `physics_backend`, `physics_device`,
`physics_solver_iterations`, and `physics_cpu_threads`. Saved experiments
retain these settings. Pin an explicit
backend for comparisons across machines; **Auto** can select different engines.

**Physics CPU workers · advanced** controls native MuJoCo tasks and host tasks
for both PhysX backends. It defaults to two; zero runs tasks inline. More workers are not
necessarily faster. `EERSIM_PHYSX_THREADS`, when set, overrides the saved or GUI
value for PhysX only; it does not affect MuJoCo workers. This setting does not parallelize CPU graphics, fixed physics steps, or
PROS callbacks.

## Physics, graphics, and timing

**Graphics renderer** independently selects OpenGL or CPU rendering. Separate
Physics and Graphics status lines identify the devices. An Intel/AMD OpenGL
renderer does not provide CUDA physics; an NVIDIA physics device does not
require GPU rendering to be enabled.

Each PhysX world runs in a private process because the pinned runtime has one
attached scene per process. The parent computes motor commands and sensor
observations on the CPU and exchanges batched state with that worker. Startup
verifies the live scene's GPU dynamics, GPU broadphase, and valid CUDA context;
it does not infer acceleration from a requested setting or a device name.
Robot contact manifolds and compact scoring contact pairs are transferred each
tick; complete contact details remain available on demand for that same state.
This avoids converting thousands of unused game-piece contact points into
Python objects on every tick. A pristine prepared scene is reused for the first
native start; reset or configuration changes still rebuild it. Rendered
geometry follows the authoritative PhysX poses; the retained Bullet geometry
is not stepped as a second physical simulation. The fixed-base URDF motor setup
preview uses an independent MuJoCo scene with the same VEX motor model.

All engines advance one fixed tick per control update, normally **1/240 s**.
Rendering does not enlarge or skip physics steps. RTF is completed simulated
time divided by wall time, and backlog is retained when the machine falls behind.
PhysX performs one normal initialization tick before its first state read;
competition timing still starts after the PROS initialization callbacks.

The advanced solver control accepts 1–255 iterations, default 100. This sets
MuJoCo's Newton iteration limit, Bullet's solver iteration limit, or PhysX TGS position iterations; PhysX velocity
iterations remain 1. Lower values change convergence. Equal iteration counts
across the engines are not equivalent numerical settings.

## Model and validation limits

PhysX implicit-drive torque feedback is currently a bounded estimate from the
drive demand and velocity error, not solver-measured joint reaction torque.
Motor current is derived from that estimate. Physical force limits still use
the VEX torque envelope, but this telemetry must not be treated as calibrated
load or electrical measurements.

Bullet and PhysX use the same configured friction coefficients with multiplication
as the pair-combination rule. Their contact, friction, sleeping, and joint
solvers still differ. The PhysX conversion approximates Bullet margins with
contact offsets; Bullet rolling/spinning friction has no direct material mapping.
PhysX uses native sleeping rather than Bullet's numerical-rest helper. Convex
subdivision and cooking can also change contact behavior. Matching profiles and
seeds do not guarantee matching trajectories.

PhysX requires positive mass on every robot link; massless fixed links must be
combined in the source URDF. Missing/ambiguous joint mappings and unsupported
controls produce errors. Validate each custom robot's loaded mechanisms,
acceleration, braking, grasp/release, and sensor conventions against hardware.
The [validation record](validation.md) separates existing Bullet/OpenGL checks
from new backend acceptance.

There is no Tensor Core switch. This integration uses rigid-body contact and
constraint calculations and has no supported Tensor Core acceleration path.
[DLPack tensor bindings](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/docs/tutorials/tensor_bindings.md)
exchange buffers; they do not enable Tensor Cores. The current controller/state
exchange uses CPU arrays and does not claim GPU zero-copy control or a neural
inference workload.
