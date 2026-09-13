# RTX 4070 Ti: 32-iteration capture

The new capture successfully requests and reports **32 TGS solver iterations**
on CUDA PhysX (`scene_gpu_flags=11`, hardware acceleration true). Both rates
complete their 120-second wall budgets with exit code zero and no watchdog
timeout. However, this is an **idle default X-drive**, with no PROS workspace or
preload pin selected. It is not a matched repeat of the previous 100-iteration
claw-autonomy capture.

The [sanitized metrics](diagnostics-4070ti-iterations.json) retain the comparison and evidence keys.
The 52,119-byte ZIP has 15 members, all CRC checks pass, and its SHA256 is
`12fc9d78f9a79e8dc04c241efb59ed8f963d6730e5b661a0a6b67e282e243d77`.
Both current and historical archives were read in memory without executing or
extracting their contents.

| Current 32-iteration result | 240 Hz | 120 Hz |
|---|---:|---:|
| Simulated seconds | 35.988 | 69.675 |
| Whole-loop realtime factor | 0.300x | 0.581x |
| Physics-step realtime factor | 0.302x | 0.585x |
| Mean complete tick | 13.803 ms | 14.255 ms |
| Native PhysX step | 8.686 ms | 8.532 ms |
| Native step share | 62.93% | 59.85% |
| Contact read/conversion | 1.208 ms | 1.229 ms |
| Other tick work, excluding control/state reads | 3.523 ms | 4.099 ms |
| Mean raw contact reports | 146.0 | 87.8 |
| GPU utilization during physics | 65.38% | 65.45% |
| GPU power during physics | 110.51 W | 109.22 W |

The simulator and dependency versions match the reference: EERSim 0.1.3,
ovphysx 0.5.11, the same Ryzen 9 5900XT/RTX 4070 Ti, and driver 616.56. Start pose,
field-pack file hash, friction, gravity, noise, seed, and GPU selection also match.
Robot URDF and motor-profile hashes differ, as do workspace selection, the extra
preload, configured mode duration, and requested GUI render FPS. The scene now
has 4,459 shapes, 11 articulation links and eight DOFs, rather than the claw
scene's 4,514 shapes, 10 links and six DOFs.

There are **zero native initialization or competition ticks**. Placed-pin count
stays at its five initial placements, with no new placement. The zero-force fix
remains active: 73 regular-body rows are skipped and four submitted each tick;
69 contact-presence pairs are retained at the end of both runs. These counters
do not establish grasping, stacking, or motor-control accuracy at 32 iterations.

The previous claw capture averaged native steps of 25.133/19.159 ms and whole-loop
RTF of 0.138x/0.346x. Its shorter simulated coverage included different motion
and settling phases. Even over a common first 10 simulated seconds, estimated
interval RTF differs: 0.263x/0.508x for this neutral run versus 0.112x/0.222x for
the claw reference. Those common-window values use interpolated progress
observations and exclude earlier scene loading; they still compare different
workloads and cannot isolate the iteration reduction.

The user reports **Rocksmith 2014 running in the background** during this capture.
GPU utilization and power are device-wide; the archive cannot separate the
game's contribution from EERSim. Clocks remain at 2,805 MHz, with maximum
temperatures of 59 C/58 C. Native step time also combines CPU work, GPU execution
and synchronization, so utilization alone does not identify the remaining cost.

A controlled follow-up should reuse one exact saved claw configuration and PROS
workspace, including the preload and identical simulation durations, and change
only 100 versus 32 iterations. Hold background applications consistent or close
Rocksmith for both cases. Validate successful grasp, lift, release, placement and
stacking before drawing accuracy conclusions or adopting a lower default.
This capture supports positive idle-throughput evidence, not an isolated solver
speedup or a change to the current defaults.

## Run a matched claw comparison

The [paired capture helper](../tools/compare_claw_iterations/README.md) works with
the existing Windows 0.1.3 directory/installer EXE. It creates a fresh copy of the
bundled claw program and fixes the robot, field, preload, initial conditions and
30-second Skills Auto duration for both 100- and 32-iteration cases. It runs
headlessly and collects the two diagnostic reports in one shareable ZIP.
Close Rocksmith and other EERSim simulations before collecting a reference pair.
The output distinguishes full mode completion from limited-duration captures;
successful execution alone does not establish equivalent contact accuracy.
