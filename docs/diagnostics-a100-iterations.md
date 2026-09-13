# A100 matched claw solver comparison

The A100 can run the full simulator and unchanged PROS program headlessly over
SSH, without sudo or GUI forwarding. Two matched pairs at **120 Hz** passed the
claw placement test at **100 iterations** and failed it at **32 iterations**.
The lower setting ran faster but changed the physical outcome; the default
remains 100.

| Pair | TGS iterations | Step realtime factor | Loop realtime factor | Physical lift | Placement validation |
|---|---:|---:|---:|---:|---|
| first | 100 | 0.2922x | 0.2814x | 199.91 mm | passed |
| first | 32 | 0.4414x | 0.4198x | 199.63 mm | failed |
| repeat | 100 | 0.2927x | 0.2825x | 199.91 mm | passed |
| repeat | 32 | 0.4336x | 0.4127x | 199.63 mm | failed |

Every run completed 3,600 competition ticks (30 seconds), plus 13 native
initialization ticks. All used the bundled claw, full dynamic Override field,
one preload, identical starting pose and PROS source. Within each pair the only
configuration difference was solver iterations. Live native verification
reported CUDA dynamics, GPU broadphase and a valid context (`scene_gpu_flags=11`)
on physical GPU 1, exposed as logical device 0. The scene contained 4,514 shapes.

At 100 iterations the released pin stayed upright, about **0.24 mm** from the
goal center. At 32 it ended tilted, about **108.15 mm** from the center. Both
runs lifted it by about 200 mm and the PROS program printed its completion
message. The 32-iteration failure is supported by final geometry, independently
of the scoring contact cache; program completion alone is not placement success.
The repeated runs reproduced the physical outcome: all 3,613 recorded CSV rows
matched exactly after excluding wall-time measurements, separately at each
iteration setting, as did final field-object states.

Across matched competition ticks, the 100/32 robot paths differed by 36.84 mm
RMS in the field plane, peaking at 186.31 mm during motion. Their final robot
positions differed by only 2.12 mm. Endpoints alone therefore hide consequential
differences during the scoring maneuver; the claw opened past 10 mm about
1.38 simulated seconds later at 32 iterations.

## Measured cost

In the first pair, reducing iterations increased competition throughput by
1.51x (102.67 to 67.96 wall seconds). Native synchronous step time averaged
19.53 versus 9.37 ms, accounting for about 67.7% versus 49.1% of all measured
step time. Contact conversion averaged 2.47 versus 2.67 ms. The saving came
from the native solver call; work outside it did not become faster.

Native time includes execution, synchronization, callbacks and readback. It
cannot be interpreted as GPU kernel time alone. Nested session/world/RPC spans
overlap and must not be added to their child spans. Contact/state lifetime
counters also include one startup read. Step realtime factor excludes loading,
compilation and benchmark observation/CSV work; loop realtime factor includes
initialization and observations but still excludes scene setup and rendering.

## Environment and reproduction

Source commit: `43a659f5a9927c6deabd08422f7cee95e56e99bd`. Hardware: A100-SXM4-40GB,
dual EPYC 7H12, driver 595.71.05. Runtime: Python 3.11.15, ovphysx 0.5.11,
ovstage 0.1.1.355824, NumPy 2.4.6, SciPy 1.16.3 and PyBullet 3.2.7. The Windows
capture used PyBullet 3.2.5 as the geometry loader. These Linux results do not
predict Windows GUI/driver performance or replace installer validation.

From a source checkout with its dependencies installed and an unused CUDA
device selected, run:

```sh
EERSIM_PROFILE=1 python tools/benchmark_integration.py \
  --backend physx_gpu --device 0 --physics-hz 120 --iterations 100 \
  --seconds 30 --validation claw --output results/claw-100.json
```

Repeat with `--iterations 32 --output results/claw-32.json`. The default
benchmark rate remains 240 Hz unless explicitly selected. The 120 Hz option,
fixed-clock accounting and performance capture passed 19 focused harness tests.

The [sanitized evidence record](diagnostics-a100-iterations.json) includes both
pairs, source/archive hashes, environment, repeat checks and final pin geometry.
Raw CSV/JSON/log reports were retrieved with matching SHA256 hashes. All owned
benchmark processes exited, and the temporary server source, assets, environment
and caches were deleted; directory absence was verified. The second SSH session
reset, but its complete reports were recovered and checked after reconnecting.

This validates one autonomous fixture, not agreement with a physical VEX robot
or every custom design. It provides a reason to preserve solver fidelity while
improving performance. The separate [Windows diagnostic helper](../tools/compare_claw_iterations/README.md)
remains available for machine-specific captures using the installed 0.1.3 EXE.
