# Physics backends

The source application connects both Bullet and PhysX to the same native PROS
callbacks, VEX cartridge/gearing model, contact-dependent omni-wheel forces,
pneumatics, and sensor noise/latency model. PhysX supplies the live rigid-body
poses, joint state, contact forces, and distance queries. Its robot is a
reduced-coordinate articulation; field mechanisms use regular PhysX joints.
The native robot code does not need a different control API for each backend.

This integration is separate from the earlier passive
[A100 benchmark study](../tools/physx_prototype/README.md). Those measurements
excluded robot controllers, tire forces, sensors, and rendering. They do not
establish the integrated simulator's throughput or accuracy. Windows CUDA and
the GPU-enabled installer still require validation; no integrated performance
numbers or newly published GPU installer are claimed here.

## Install and select

Normal source setup installs the Bullet/OpenGL application. Add GPU physics to
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

In **Field → Physics engine**, choose **Auto · CUDA when available**,
**Bullet · CPU**, **PhysX · NVIDIA CUDA**, or **PhysX · CPU**, then click
**Apply settings & reset scene**. **Auto** attempts CUDA and falls back to
Bullet when runtime dependencies, device checks, or scene initialization fail.
The reason appears in Field, Telemetry, and the log. Explicit PhysX selections
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

The JSON keys are `physics_backend`, `physics_device`, and
`physics_solver_iterations`. Saved experiments retain all three. Pin an explicit
backend for comparisons across machines; **Auto** can select different engines.

## Physics, graphics, and timing

**Graphics renderer** independently selects OpenGL or CPU rendering. Separate
Physics and Graphics status lines identify the devices. An Intel/AMD OpenGL
renderer does not provide CUDA physics; an NVIDIA physics device does not
require GPU rendering to be enabled.

Each PhysX world runs in a private process because the pinned runtime has one
attached scene per process. The parent computes motor commands and sensor
observations on the CPU and exchanges batched state with that worker. Rendered
geometry follows the authoritative PhysX poses; the retained Bullet geometry
is not stepped as a second physical simulation. The fixed-base URDF motor setup
preview remains an independent Bullet scene.

Both engines advance one fixed tick per control update, normally **1/240 s**.
Rendering does not enlarge or skip physics steps. RTF is completed simulated
time divided by wall time, and backlog is retained when the machine falls behind.
PhysX performs one normal initialization tick before its first state read;
competition timing still starts after the PROS initialization callbacks.

The advanced solver control accepts 1–255 iterations, default 100. This sets
Bullet's solver iteration limit or PhysX TGS position iterations; PhysX velocity
iterations remain 1. Lower values change convergence. Equal iteration counts
across the engines are not equivalent numerical settings.

## Model and validation limits

PhysX implicit-drive torque feedback is currently a bounded estimate from the
drive demand and velocity error, not solver-measured joint reaction torque.
Motor current is derived from that estimate. Physical force limits still use
the VEX torque envelope, but this telemetry must not be treated as calibrated
load or electrical measurements.

Both engines use the same configured friction coefficients with multiplication
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
