# EERSim 0.1.6 preview validation

This preview addresses viewport scheduling, repeated scene loading, and pins
trapped in the preplaced goals. It exposes PhysX CPU worker settings and makes
workspace compilation an explicit action. The fixed physics rate, solver
iterations, motor equations, sensor sampling, and PROS callback order are
preserved. Collision geometry changes are described separately below.

## Viewport and loading

The live loop no longer caps rendering near four FPS whenever physics backlog
exceeds 0.1 seconds. Render deadlines use current wall time and start-to-start
frame cadence. Physics batches yield at a frame deadline while returning every
unfinished fixed tick to the accumulator. A single slow physics step or render
call can still exceed that deadline; this is not a guarantee of 30 FPS.

Workbench queues a small notification and retains the newest frame, avoiding a
backlog of obsolete pixel arrays. Generation checks prevent an old scene or
replay notification from consuming a newer frame. Telemetry reports completed
live rendering FPS, which is separate from Qt paint events and display refresh.
Run completion forces a final status update even between periodic telemetry
samples, so the controls do not remain in the running state after execution ends.

Start reuses an unchanged prepared scene. A scene that has already advanced is
still reset by the run lifecycle. Applying only viewport FPS or configured
simulation speed keeps the world; physical changes still rebuild it. Explicit
unchanged Apply reloads external files edited in place. Loading the demo no
longer compiles C++; click **Build workspace**, then **Start**.

GPU replay loading now passes immutable recorded visual descriptors directly
to the existing VTK mesh cache. It no longer first creates all Bullet visual
bodies. A private CPU scene is created only when CPU rendering is selected or
GPU setup/drawing falls back. Fallback starts at the latest selected replay pose.
Load timings are included in replay renderer status. No actual GPU-render
loading speedup is claimed from the mock checks.

## Pin/goal contact correction

The official goal CAD has a horizontal annular shoulder at socket height
76.5 mm. The earlier contact mesh bridged it diagonally, narrowing the opening.
The earlier pin root convex hull also filled a concave fillet. Together they
created an initial intersection at all five official preplaced goal-pin poses.

The corrected socket retains that shoulder, refines the inner lip and repairs
cap winding. Four adjacent axial bands per root preserve the pin's concavity.
The official starting positions, visual meshes, mass, inertia, friction,
restitution and solver settings remain unchanged. No object is frozen or
teleported to suppress contact forces.

Static comparison finds zero strict mesh intersections at the original upright
and flipped seating frames. Across 216 heights and 192 radial directions, the
maximum sampled inward goal deviation from the recorded CAD mesh falls from
1.968 mm to 0.129 mm. These are sampled mesh measurements; the source visual
mesh itself declares a 0.25 mm tessellation tolerance. See
[asset provenance and regeneration](../assets/README.md).

The correction increases each pin from 56 to 64 exported convex shapes and each
goal socket from 1,408 to 5,312 triangles. The claw fixture has 38 pins including
its preload, adding 304 convex shapes. Each socket remains one static concave
mesh. This accuracy correction adds collision work.

In the isolated CPU PhysX test at 120 Hz, each of the five pins settled for one
second before a 2 N upward force was applied at its current center of mass for
0.25 seconds. All five baseline pins remained trapped, moving less than
0.003 mm upward. All five corrected pins cleared their goals, rising about
0.543 m. This is a controlled removability test, not a claw-grip or insertion
validation. The same baseline-trapped/corrected-free result passed at 240 Hz
on CPU PhysX and 120 Hz on A100 GPU PhysX. Full native PROS claw runs separately
passed on CPU and GPU PhysX.

## Native throughput and CPU workers

The Linux A100 server ran the unchanged full claw program for 30 competition
seconds at 120 Hz, with 100 position and one velocity TGS iteration. All runs
completed 3,613 ticks including initialization and passed the physical claw
outcome checks. No live frames were rendered during stepping.

| Scene | Physics | CPU workers | Step real-time factor |
| --- | --- | ---: | ---: |
| Previous collision geometry | PhysX CPU | 0, inline | 0.928× |
| Corrected geometry | PhysX CPU | 0, inline | 0.848× |
| Corrected geometry | PhysX CPU | 2 | 0.796× |
| Corrected geometry | PhysX CPU | 8 | 0.720× |
| Corrected geometry, replay recording enabled | PhysX A100 CUDA | 2 | 0.321× |

Step timing includes controller exchange and physics, and excludes scene
loading, compilation, live rendering and periodic diagnostic queries. These
are individual server runs, not statistical estimates or Windows benchmarks.
The GPU case also includes replay capture and is not a matched replay-off CPU
comparison. More workers slowed this workload, so the default remains two and
the setting is configurable rather than automatically using all 256 server
logical CPUs.

For this scene, try **PhysX · CPU**, **CPU workers → 0 · inline**, **120 Hz** and
**100 solver iterations**, with GPU graphics independently selected. An explicit
`EERSIM_PHYSX_THREADS` environment variable overrides the saved GUI setting.
The profile accepts 0–1024 workers; the GUI offers the local logical CPU count
and retains a larger valid saved count. CPU worker selection affects PhysX's
host tasks, including CUDA physics. It does not parallelize dependent physics
ticks or PROS callbacks.

TinyRenderer has no supported camera worker-count setting. Its synchronous
Python camera call can hold the GIL, so CPU rendering remains a limit even
with the scheduling fix. This preview does not introduce a separate CPU render
process or claim an all-core software renderer.

## Validation scope

Focused configuration, replay, engine and GUI checks passed, including exact
tick/native/replay order, retained pacing backlog, and final run-state delivery.
Eleven static collision/regeneration checks passed on Windows, covering mixed
Git line endings and unchanged binary meshes; Qt packaging notice checks passed.
Generated JSON and URDF use canonical LF line endings, with identical parsed
geometry and preserved OBJ/STL bytes. Native CPU replay checks passed for both Bullet and PhysX CPU:
recorded descriptors match private Bullet geometry, fallback pixels match the
ordinary CPU replay path, and playback leaves the live physics state unchanged.
Actual offscreen Qt checks passed for the six setup tabs, four modes, worker
controls, manual Build/Start workflow, replay controls and engine cleanup. Those
Qt checks use stub engines and a prerecorded image.

An unfiltered server suite reached its 1,200-second bound during the 60-second
Bullet Skills Auto claw test and was terminated with no remaining owned
processes. It is not a passing full-suite result. Two earlier failures in that
run exposed a telemetry-dependent assertion and Windows/Linux generator line
endings; both received focused fixes. The release validation attachment records
the subsequent checks and any residual failures. The measured 30-second native
PhysX claw runs above are separate acceptance cases.

The authorized A100 GPU was used for physics only. GPU rendering could not be
validated on that server's permitted display device. This work does not establish
interactive Windows frame rates, Windows CUDA throughput, or installer upgrade
behavior. The private Windows installer is unsigned.

The [academic/noncommercial license](../LICENSE) covers original EERSim material;
third-party terms remain separate. The installer includes the
[Qt/PySide notices](notices/qt-pyside6.md) and omits the unused Qt Virtual Keyboard
payload. Static bundle checks verify source/asset identity and these notices.
