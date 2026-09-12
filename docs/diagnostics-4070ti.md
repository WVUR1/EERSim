# RTX 4070 Ti performance capture

The supplied `EERSim-diagnostics.zip.temp` is a valid ZIP; all 15 members pass CRC
checks. Its SHA256 is `cdd940dc4414dfbffc5563b3493a9c94645d5297464f46bb944a0cdfbdc0195c`.
The measurements below are the **0.1.2 baseline**, before the zero-force wakeup fix.
The accompanying [derived metrics](diagnostics-4070ti.json) contain formulas and
source keys without GPU UUIDs, PCI addresses, or personal directories.

Both cases used the default Override field and X-drive, a Ryzen 9 5900XT, and an
RTX 4070 Ti. Native verification reports GPU dynamics, GPU broadphase, and a valid
CUDA context (`scene_gpu_flags=11`); there was no CPU fallback. No PROS workspace
was selected, and both native tick counts are zero. These are neutral scene
measurements, not a claw-autonomy acceptance run.

| Measured quantity | 240 Hz | 120 Hz |
|---|---:|---:|
| Physics stepping realtime factor | 0.0634x | 0.1304x |
| Simulated seconds in 120 wall seconds | 7.567 | 15.575 |
| Mean complete tick | 65.761 ms | 63.882 ms |
| Native `PhysX.step_sync` | 60.485 ms | 58.575 ms |
| Native step share | 91.98% | 91.69% |
| Contact read/conversion | 1.685 ms | 1.620 ms |
| Control tensor writes | 0.187 ms | 0.185 ms |
| State tensor reads | 0.140 ms | 0.142 ms |
| Other tick work | 3.264 ms | 3.360 ms |
| Mean device utilization during physics | 36.0% | 34.9% |
| Mean device power during physics | 35.0 W | 41.5 W |
| Maximum GPU temperature during physics | 53 C | 55 C |
| Stationary rendering, subsequent frames | 9.18 ms | 10.24 ms |

The table partitions the complete tick using nonoverlapping worker spans and a
computed remainder. Do not add enclosing RPC, world, and session spans again.
The native step includes collision work, solving, result fetching, synchronization,
and internal readback. These logs cannot separate its CPU and GPU components or
establish kernel occupancy. Device utilization can include unrelated applications.
Rendering was measured separately at 960x720; it is not interactive GUI FPS.

Lowering the tick rate almost doubled simulated throughput because each tick
still cost about 64-66 ms and advanced twice as much simulated time. This does
not establish equal contact/control accuracy at the two rates.

## Collision workload

The exported scene contains 4,459 collision shapes:

| Source | Exported shapes |
|---|---:|
| 36 cups, 64 convex leaves each | 2,304 |
| 37 pins, 56 convex leaves each after exact subdivision | 2,072 |
| Static triangle meshes | 17 |
| Other field boxes | 54 |
| Floor | 1 |
| Robot, 11 links | 11 |

Cups and pins account for 98.1% of the shapes. The 88 dynamic actors already
include the 11 robot articulation links: 36 cups, 37 pins, four toggle rotors,
and 11 robot links. Six visual-only tape boxes are excluded from collisions.
The roughly 11,300 raw contact reports per tick include multiple compound
manifolds and mirrored subscribed endpoints; they are not 11,300 distinct body
pairs or solver constraints.

The scene and every body request 100 TGS position iterations, with both scene
minimum and maximum fixed at 100. This is different from Bullet's iteration
limit. [NVIDIA's performance guide](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/guides/performance.html)
recommends tuning iterations against acceptable fidelity; the same numerical
iteration count does not imply equivalent convergence across engines. The
diagnostic fix retains the configured iteration count and collision geometry.

## Confirmed unnecessary wakeups

The worker previously submitted a wrench row for every regular dynamic field
actor every tick, even when all force and torque components were zero. Our
GPU scene uses standard host readback (`direct_gpu_api=false`), so tensor writes
take the [CPU simulation view](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/ovruntime/source/omni.physx/plugins/tensors/SimulationBackend.cpp#L79).
Its [wrench implementation](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/ovruntime/source/omni.physx/plugins/tensors/cpu/CpuRigidBodyView.cpp#L503)
calls `addForce` and `addTorque` with default automatic waking on every selected
row. The [pinned PhysX contract](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/physx/include/PxRigidBody.h#L549)
states that this wakes an actor even for a zero force and that forces clear after
the next simulation step. Repeating these writes prevented settled field objects
from sleeping.

The correction submits only regular-body rows containing an exactly nonzero
force or torque. Application-point coordinates do not count as an active force,
and no epsilon discards small forces. Articulation controls continue normally.
Profiling counts actual submissions and skipped zero rows.

Sleeping also suppresses PhysX's ongoing contact reports. Resting scoring contact
presence is therefore retained only while every actor transform of both bodies
is exactly unchanged; movement or reset invalidates it. Robot force manifolds
remain fresh, and no cached forces are applied.

A tiny headless PhysX CPU test confirmed natural sleeping, waking caused by an
all-zero write, selective waking from a 1e-9 N pulse, and a one-tick force pulse
without residual acceleration. The supplied GPU capture predates the fix, so
it does **not** measure the resulting speedup. Repeating the same neutral capture
after updating will quantify that separately from a subsequent PROS claw run.

A separate test with the actual CAD pin and goal confirmed scoring through 120
consecutive stationary ticks with no native contact reports, then cleared scoring
after a native pose update moved the pin away. This tests contact invalidation,
not force-driven extraction from the socket. A short full-field CPU startup run
also retained all 4,459 shapes and skipped all 73 zero-force cup/pin rows on every
tick. It covered only 0.317 simulated seconds, so it is not a settled-field
performance measurement.
