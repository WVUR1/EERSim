# EERSim 0.1.5 performance and replay validation

The two Python optimizations reduced completed-step state assembly and tire-force
bookkeeping without changing geometry, timestep, solver iterations, motor or
sensor models, force accumulation order, or backend defaults. Session state
reuses the joint observations already copied for that completed step. Passive
hinges omit unused COM/anchor linear-velocity work, and three-component floating
cross products retain the original arithmetic order.

## Verified optimization comparison

These matched full-claw runs used ordinary bundled PROS code, the full Override
field, 120 Hz physics and 100 position / 1 velocity TGS iterations. They measured
the two optimizations **before replay recording was added**.

| Backend and dispatcher | Baseline step RTF | Optimized step RTF |
| --- | ---: | ---: |
| PhysX on the host CPU, zero workers | 0.8016 / 0.7997 | 0.9272 / 0.9250 |
| PhysX CUDA on NVIDIA A100, two workers | 0.3101 | 0.3270 |

The CPU runs used the explicit `EERSIM_PHYSX_THREADS=0` setting. The default is
still two workers, and the GUI does not change it to zero. Host-CPU execution on
an A100 server is not execution on the A100. CPU and CUDA are compared to their
own baselines; their trajectories are not asserted to match each other.

Each optimized run completed the 30-second autonomous mode, physically lifted
and released the preload into the intended goal, retained placement through
the end, completed native code and disabled normally. Within each backend,
all 3,613 physical CSV rows matched the corresponding baseline exactly after
excluding only `step_wall_seconds`. Initial/final field states and recorded
native activity samples also matched. These are finite regression checks: the
CSV does not capture every field object's continuous horizontal trajectory,
and prototype placement detection is not official VEX scoring.

Step RTF is completed competition simulated time divided by timed competition
step wall time. Loading, compilation, rendering, and periodic diagnostics are
outside this measurement. Neither measured backend reached realtime for this
workload. These results do not predict Windows, GeForce, or interactive GUI
throughput.

## Replay validation

The remote tiny-scene Bullet and PhysX CPU replay checks passed, including
separate render-only clients, backward seeking, preserved live state and
retention of copied visual assets after reset/close and deletion of the original
fixture assets. Actual offscreen Qt transport/layout acceptance passed with
stub engines and a supplied image; the narrow-window screenshot was inspected.
The 37 evidence files were transferred and hash-verified. Ninety source checks
passed at this stage. The Qt fixture verifies controls and layout separately
from the real native replay-image checks.

The final 0.1.5 source was compared with recording disabled and enabled:

| Same-source comparison | Recording off | Recording on | Physical regression |
| --- | ---: | ---: | --- |
| PhysX CPU, zero workers, 120 Hz / 100 TGS | 0.9152x | 0.9157x | Exact compared records |
| PhysX CUDA A100, two workers, 120 Hz / 100 TGS | 0.3267x | 0.3245x | Exact compared records |

Both pairs completed the same full claw routine. All 3,613 physical CSV rows,
170 initial/final object records, and 32 native activity samples matched within
each backend. These results include actual capture and finalization inside the
timed physics/control step. The observed CPU step-cost change was -0.0047 ms;
the GPU change was +0.1714 ms. One pair includes scheduling variation and does
not establish a confidence interval or imply that recording makes physics faster.

On CPU, directly timed recording work totaled 0.2753 seconds across all 3,613
physics ticks, including initialization and finalization: 0.0762 ms per tick.
Finalization took 0.0014 seconds. The clip contained 905 pose samples at 30 Hz,
including the initial and final states, covered 30.108 simulated seconds and
used 5,599,584 recording bytes plus 122,861,854 bytes of pinned visual assets.
Backward seeking reproduced the first image exactly and did not change the live
world, sensor/RNG state, commands, or native worker RPC count.

The asset parser now skips expensive tokenization of unrelated mesh coordinate
lines. All 217 shipped assets retained identical pinned paths and bytes in the
old/new parser comparison. Measured full-field replay preparation fell from
61.944 seconds to 2.420 seconds. This is a loading improvement; collision
geometry and motor/solver models are unchanged.

Full-field software playback remains expensive: creating the private scene,
seeking, checking, rendering three images, and cleaning up took 32.59 seconds,
including 9.23 seconds for the three renders. The remaining time was not
independently separated into those operations. These costs are outside step RTF.
The CPU competition loop measured 0.817x excluding periodic diagnostics, lower
than its 0.916x step RTF. Neither value includes all loading and playback costs.

Final native Bullet/PhysX CPU visual acceptance passed again; the corrected
replay suite passed 93 checks, including a pre-existing Qt application. GPU
physics acceptance passed, but a separate GPU replay-render test could not
establish the authorized EGL device UUID under the server's display permissions.
It stopped before creating a graphics context. This release therefore has no
new A100 GPU playback-render acceptance result.

Replay targets 30 pose samples per simulated second; actual sample rate can be
lower to fit the configured physics rate and recording budget. Seeking selects
a stored pose rather than interpolating or resimulating. One active recording
and one retained clip can coexist. The 64 MiB recording allocation per clip and
512 MiB copied-visual-asset budget per clip do not bound whole-app memory.
Completed, stopped, and failed clips with valid motion are labeled accordingly;
the latest retained clip lasts for the current app session. Reset/import keeps
it available, and Live returns to a paused authoritative world.

## Windows release identity

The unsigned [0.1.5 preview](https://github.com/WVUR1/EERSim/releases/tag/v0.1.5-windows-preview.1)
includes the installer checksum, build manifest, and
[artifact validation report](https://github.com/WVUR1/EERSim/releases/download/v0.1.5-windows-preview.1/EERSim-0.1.5-validation.json).
The report identifies the built source commit and exact installer/application
hashes. Its Windows verification scope is static frozen-code and payload checks
plus headless CLI/native compiler and protocol checks. It does not establish
new Windows GUI, CUDA physics, or installer lifecycle acceptance.

The final runtime CI run on `dccd75f` completed with **524 passed, 41 skipped,
and two failed**. Both failures are the existing Bullet exact-zero rest-velocity
assertions in `test_authored_cup_pin_stack_settles_then_wakes_on_collision` and
`test_bounded_rest_goal_pin_sleeps_and_contact_wakes_it`; CI remains failed.
CI skipped its wheel step. Separate local dependency/file-size checks and the
0.1.5 wheel build passed. The release documentation commit retains the same
runtime source, with explicit Git/Windows line-ending verification in the report.

Earlier [0.1.4 measurements](diagnostics-a100-0.1.4.md) and the supplied
[RTX 4070 Ti follow-up](diagnostics-4070ti-followup.md) remain separate historical
evidence with their own workloads and limitations.
