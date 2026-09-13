# A100 validation for the 0.1.4 Windows preview

Version 0.1.4 reduces repeated CPU work in contact conversion, force transforms
and scoring observations. Actor contact metadata is reused, selected contacts
are materialized with fewer temporary allocations, center-of-mass transforms
are reused within each completed worker state, and wheel/goal observations are
reused within a synchronous calculation pass. The next state or pass sees fresh
values; runtime settings and movable custom goals remain observable.

Built and tested source: `b9d80de6ef6740eaed240db43f0d00ebb02881c7`.
Baseline: `5e9e09e515249bbeb12128ac2d465a68250500aa`.
The [private Windows preview](https://github.com/WVUR1/EERSim/releases/tag/v0.1.4-windows-preview.1)
contains the unsigned installer, checksum, build manifest and combined validation
record. This documentation was recorded after the source build; the release tag
identifies the exact built/tested source commit.

## Matched A100 results

| Case | Step RTF | Loop RTF excluding queries | Native ms | Contacts ms | Tires ms | Sensors/scoring ms | Checks |
|---|---:|---:|---:|---:|---:|---:|---|
| before-1 | 0.2911x | 0.2807x | 19.203 | 2.492 | 1.891 | 1.455 | passed |
| after-1 | 0.3088x | 0.2976x | 19.196 | 1.337 | 1.805 | 0.992 | passed |
| after-2 | 0.3020x | 0.2908x | 19.467 | 1.521 | 1.808 | 1.025 | passed |
| before-2 | 0.2906x | 0.2806x | 19.239 | 2.500 | 1.906 | 1.449 | passed |

All runs use two verified native CPU workers, GPU physics, 120Hz and 100 solver iterations. Values above are competition-only stage means; periodic activity queries are outside timed steps.

- before-1 to after-1: 6.07% step throughput change; 1.638ms saved per competition tick.
- before-2 to after-2: 3.91% step throughput change; 1.079ms saved per competition tick.

Across both repeats, combined step throughput increased from 0.2909x to 0.3053x (**4.98%**). This combines simulated duration divided by summed competition step time, rather than averaging realtime factors.

| Stage | Baseline mean ms | Candidate mean ms | Cost reduction |
|---|---:|---:|---:|
| Contact conversion total | 2.496 | 1.429 | 42.75% |
| Contact pair selection (within total) | 0.448 | 0.111 | 75.16% |
| Robot contact materialization (within total) | 1.051 | 0.253 | 75.95% |
| Tire calculations | 1.898 | 1.806 | 4.86% |
| Sensors and scoring | 1.452 | 1.008 | 30.56% |
| Native PhysX step | 19.221 | 19.331 | -0.57% |

Exact physical CSV, initial/final object, preload and native activity equality: **True**.
Independent physical acceptance and controlled configuration checks: **True**.
Evidence matches supplied checksum manifest: **True**.

Source changes: `eersim/__init__.py`, `eersim/physics.py`, `eersim/physx_contacts.py`, `eersim/physx_world.py`, `pyproject.toml`

All four runs lifted the preload 199.91mm and retained it in the intended red goal from 11.3167 through 30 simulated seconds, released from the robot. The 3,613 CSV rows per run, 170 initial and 170 final field-object records, and 32 sampled native activity observations match exactly. Robot/field assets, collision shape inventory (4,514), solver settings, and CPU worker count are unchanged.

These changes reduce CPU contact conversion and repeated control/observation calculations. The native PhysX step remains approximately 19.2-19.5ms and accounts for 70.5-71.1% of candidate step time; the result remains below realtime. Native timing includes CPU work, GPU execution and synchronization, so this measurement does not isolate individual GPU kernels.

This is one full claw fixture on a shared Linux A100 server, repeated in before/after/after/before order. It does not establish equivalent Windows GUI performance, broad physical calibration, or official game scoring. The JSON record contains all cumulative stage differences and per-check results. Nested contact stages must not be added to their parent.

## Windows artifact checks

The unsigned installer includes the GPU runtime, desktop PROS compiler and
default robot/field assets. Static auditing matched all eight checked frozen
modules, including the three optimized modules, to the source. All 10,353
checked files across the assets, native adapter, examples, NVIDIA runtimes and
compiler matched their build inputs. Runtime metadata is ovphysx 0.5.11 and
OVStage 0.1.1.355824; native helper ABI declarations match the bundled helper.

The packaged EXE passed `--headless --help`. Its packaged bridge compiled the
unchanged bundled claw workspace with the bundled compiler on a system-only
PATH, then completed 13 initialization protocol ticks with synthetic inputs
and shut down. These checks do not run a physics scene. No Windows GPU,
interactive GUI or installer lifecycle acceptance was performed for this build.
The installer retains the existing application ID and per-user data behavior.

- Installer SHA-256: `66fe1522fa5ea96778cb2b854c5aa82124897a9f927a2486820c2bdef51c743b`.
- Installer size: 370,900,541 bytes.
- Bundled EXE SHA-256: `4c8a1a718d8d2366b63bc8633685a1845d1b5ff7d377a6d97dadc0921d721c6c`.

## Tests and remaining failures

The focused contact, force-cache, observation, verification, diagnostics and
release-payload suite passed all 174 tests. Full
[CI for the built source](https://github.com/WVUR1/EERSim/actions/runs/34777057138)
reports 405 passed, 40 skipped and two failures. Both failures also occur at
the baseline commit with the same reported assertions:

- `test_authored_cup_pin_stack_settles_then_wakes_on_collision`
- `test_bounded_rest_goal_pin_sleeps_and_contact_wakes_it`

These Bullet tests expect exactly zero resting velocities. They remain
unresolved; the release does not claim an all-green suite. All 33 newly added
tests pass. Actual GPU physics acceptance was performed on the A100.

## Evidence and cleanup

[Portable comparison JSON](diagnostics-a100-0.1.4.json) contains per-run
timings, physical checks, source/archive hashes and comparison details.
The raw 20-file evidence archive was downloaded and its SHA-256 verified as
`32542deae59b2ff6166f073bd48ac2cc8f6c0a4b4ff04e1ecdb8c58e861d6c4f`
(7,079,964 bytes). Private host paths and device identifiers are omitted from
the published summary. The temporary server workspace, including source,
assets, environment, caches and outputs, was removed after checking no owned
benchmark processes remained; absence was verified.
