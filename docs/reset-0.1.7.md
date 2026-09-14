# Reset behavior and 0.1.7 validation

Reset stops robot code and reconstructs the robot, field objects, and seeded
sensor state using the last applied experiment. It preserves the camera and
does not compile the workspace or apply unsubmitted edits. PhysX includes its
required bootstrap tick in the reset world's simulation time.

The button and viewport show **Resetting…** until a replacement frame is ready.
Repeat clicks, replay entry, and conflicting controls are blocked during this
operation. A reset error is reported without allowing a second error during
cleanup or observation to terminate the engine worker. Fix the indicated cause
and use **Apply settings**, or retry **Reset**, to recover.

Native cleanup escalates through bounded shutdown attempts. Resources that fail
to close remain owned, and further scene creation is blocked until retained
cleanup succeeds. A failed reset does not keep retrying a broken renderer.

## Checks performed

- 135 focused Python checks passed, with one existing comparison-harness test
  skipped. Two native cases were excluded from that mocked run and covered by
  the separate engine test run below.
- All four engine tests passed separately, including real Bullet scene
  configuration failure/recovery. No visible window was created by those tests.
- The final source opened a real Qt window with Bullet physics and Intel Iris
  Xe OpenGL rendering. After manual lift motion, Reset and Stop → Reset restored
  the original viewport pixels, robot pose, and joints exactly. Busy controls
  and rendered completion were checked. Resets took 11.01 s, 12.30 s; all four owned
  processes exited.
- A CPU PhysX / Intel OpenGL manual-motion check passed Stop → Reset in 97.98 s,
  restoring the original image, pose, and joints; all seven owned processes
  exited. This normal-path check preceded the final engine error-recovery patch.
  It did not run compiled autonomous code or CUDA physics.

## Limits

These checks demonstrate recovery and feedback, not faster scene construction.
The full repository suite was not rerun; the existing Bullet rest-state failures
reported for 0.1.6 have not been addressed. The reported desktop CUDA Reset crash
was not reproduced on this Intel-only laptop. Installer wizard/upgrade and
packaged Windows CUDA execution were not tested for this update. The release's
validation JSON records separate packaging and headless executable checks.
