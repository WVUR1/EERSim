# A100 activity and CPU worker diagnostics

GPU physics testing now uses the A100 server headlessly over SSH. The full
claw fixture retains the Override field, 120 Hz fixed physics, 100 TGS solver
iterations, original robot program and preload. Native inspection verifies GPU
dynamics, GPU broadphase and a valid CUDA context rather than inferring them
from the requested backend.

| Run | CPU workers | Step RTF | Loop RTF excluding queries | Native step, ms | Contacts, ms | Claw placement |
|---|---:|---:|---:|---:|---:|---|
| first | 2 | 0.2856x | 0.2756x | 19.422 | 2.635 | passed |
| first | 0 | 0.3186x | 0.3074x | 18.665 | 1.403 | passed |
| repeat | 0 | 0.3018x | 0.2907x | 18.770 | 1.716 | passed |
| repeat | 2 | 0.2895x | 0.2793x | 19.314 | 2.482 | passed |

All four completed runs passed lift and released goal placement. Every one of
their 3,613 recorded physical CSV rows, excluding only step wall time, matches
the previous validated 100-iteration run exactly, as do all 132 final field
object records. The preload lifted 199.91 mm and remained placed from 11.3167
through 30 seconds. Native actor motion and sleep states match at all 32 sampled
points across successful runs.

Inline CPU tasks (zero workers) improved step throughput by 11.55% in
the first comparison and 4.26% in the reverse-order repeat. These are
small, variable gains and remain below real time. Actual native worker counts
were verified. The application default remains two; the opt-in override supports
further controlled A100 experiments. The requested one-worker run timed out
during startup and produced zero steps, so it is not a performance measurement.
No geometry, sleep thresholds, motor settings or solver defaults changed.


## What is actually sleeping

The native scene contains 68 static actors, 78 regular dynamic actors and one
10-link robot articulation. At the one-second sample, all 37 field pins and
36 cups are asleep; the held preload remains awake. At the final sample, all
38 pins and 36 cups are asleep. The four toggle paddles and robot articulation
remain awake. No sampled awake actor has exactly zero velocity.

The nearby wall stack wakes during the robot approach and retreat, at the
8- and 14-second samples, and sleeps again by the next samples. Sleeping pieces
therefore still respond to interaction. These are actual `isSleeping()` reads
after completed physics steps, not an inference from skipped force writes or
small velocities. Robot links share the articulation sleep state.

The final paddle angular speeds are approximately 0.0014-0.0027 rad/s. Their
continuous restoring forces and residual movement are a candidate for a separate
constraint/control investigation; this experiment does not freeze them or alter
sleep thresholds. The native shape inventory remains 4,514: 4,432 convex meshes,
4 convex cores for wheel cylinders, 61 boxes and 17 triangle meshes. Sleeping
shapes remain in the scene for collision detection and wake-on-contact. Inventory
alone does not measure their per-step cost.

## Where the time goes

Across the four successful runs, the native step averaged
**18.66-19.42 ms** per competition tick, accounting for
66.6-71.4% of measured step time. Contact conversion averaged
1.40-2.63 ms. Within that contact total,
robot-contact materialization averaged 0.53-1.08 ms,
while the SDK binding read averaged 0.29-0.45 ms.
Reducing contact conversion work is therefore a measured CPU-side opportunity;
the native step still needs separate GPU/CPU profiling.

Tire calculations averaged 1.70-2.01 ms, sensors and
scoring 1.35-1.57 ms, native robot-program round trips
0.41-0.42 ms, and bulk body-state reads only
0.09-0.14 ms. State-read skipping is a lower-priority target.


Contact sub-stages are contained in `worker.contacts.read_convert`; do not add
them to that parent. Native step time includes SDK CPU work, GPU execution and
synchronization. It is not an isolated GPU-kernel measurement. Periodic activity
queries run outside timed simulation steps, and the report records their cost
separately. Loop RTF with that measured query cost subtracted is also retained;
loading, native compilation and rendering are excluded from both loop measures.

Some SDK statistics are incomplete on this pinned GPU path. The with-contact,
cache-hit and axis-solver-constraint counters are not populated. The geometry
pair matrix reports partial contact-manager buckets, groups some primitive
pairs as convex, and omits some buckets including convex-core contacts. Zero
entries do not establish absent contacts or zero collision cost. See the pinned
[NVIDIA narrowphase implementation](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/physx/source/gpunarrowphase/src/PxgNphaseImplementationContext.cpp)
and [GPU solver statistics](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/physx/source/gpusolver/src/PxgContext.cpp).

## Reproduce

Use a source checkout with the pinned dependencies installed and an available
CUDA device selected in `CUDA_VISIBLE_DEVICES`. No display server, GUI forwarding
or sudo is required. From the repository root:

```sh
EERSIM_PHYSX_THREADS=2 python tools/benchmark_integration.py \
  --backend physx_gpu --device 0 --fixture claw --physics-hz 120 \
  --iterations 100 --seconds 30 --validation claw --diagnostics-seconds 1 \
  --max-wall-seconds 900 --output results/claw-threads-2.json
```

Repeat with `EERSIM_PHYSX_THREADS=0` and a new output filename to run CPU tasks
inline. The application default remains two workers. The override is diagnostic;
a requested value of one timed out during scene startup in this test and produced
no simulation result. Its actual native worker count could not be queried.

`--diagnostics-seconds` defaults to zero/off. A positive value enables profiling
for that run and records prepared, initialized, periodic and final samples,
including native sleep state, actual CPU worker count, SDK statistics and
cumulative performance counters. Use differences of cumulative totals/counts for
interval costs; rolling percentiles cannot be subtracted. A failed requested
query preserves completed observations and timings while failing the diagnostic.

Source commit: `07f4818d8a871d0287bae4d969dbb06bb2d1cc95`. The
[sanitized evidence record](diagnostics-a100-activity.json) contains exact results,
per-stage costs, native sleep timelines, independent trajectory checks, archive
hashes and cleanup confirmation. The environment matches the preceding A100
comparison: Python 3.11.15, ovphysx 0.5.11, ovstage 0.1.1.355824, PyBullet 3.2.7,
NumPy 2.4.6 and SciPy 1.16.3, on an A100-SXM4-40GB with dual EPYC 7H12 CPUs and
driver 595.71.05. The source passed 123 focused tests; the native helper compiled
and passed ABI checks on Windows and was exercised on the Linux GPU scene.

The 7,019,144-byte evidence archive contains 23 JSON/CSV/log files and was
retrieved with SHA256
`cdefa0e81cc84c516a5efe891ecdc320f87adef37e7a2ad56fc16c8be44c4d39`.
Both controllers verified unchanged source. All owned processes exited. The
temporary server source, assets, environment and caches were removed, and
directory absence was verified. GPU 1 returned to 5 MiB allocated and 0% use.


The [previous solver comparison](diagnostics-a100-iterations.md) explains why
100 iterations remain the accuracy baseline. This fixture is a regression check,
not calibration against a physical VEX robot or complete official game scoring.
A100 results guide physics optimization; Windows-specific GUI, rendering and
installer behavior still require Windows validation.
