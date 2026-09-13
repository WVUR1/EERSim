# Export a performance diagnostic

Use **Telemetry → Export performance diagnostics…** on the computer where the
simulator is slow. This exports a runnable script and experiment settings; it
does not start a simulation during export.

1. Choose an export location. EERSim creates a new diagnostic folder inside it.
2. Select the robot program mode. The dialog starts with the selected competition
   mode, or Skills Auto when no mode is selected. Driver modes use neutral input.
3. Choose a **Wall-time limit** of 60–600 seconds per case; the default is 120.
4. Leave **Compare 120 Hz and 240 Hz** enabled for two separate cases, or disable
   it to use the physics frequency currently shown in Field. Leave **Measure
   rendering separately** enabled to measure a stationary scene after physics.
5. Click **Export diagnostics**. The Telemetry tab shows the new folder and the
   command to run.

The export uses the controls currently shown, including edits you have not yet
applied to the running scene. Robot bindings, start position, noise, solver
iterations, physics backend, and graphics settings follow those controls.

On Windows, close EERSim before measuring so the interactive scene does not
compete for CPU/GPU resources. Open PowerShell in the exported folder and run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\run-diagnostics.ps1
```

Follow the instructions included in that folder for other supported platforms.
By default there are two cases with a 120-second wall-time budget each. Scene
loading, PROS compilation, and cleanup add time. A comparison of different step
rates is a performance experiment; it does not establish identical physical
accuracy. Both cases use the same starting configuration, program mode, and
solver iteration count.

Physics and native robot code run without display-rate pacing during measurement.
An optional separate render check then measures a stationary scene for up to
12 frames, stopping after a frame when 30 seconds have elapsed. It records first-frame and subsequent-frame costs and
the active or fallback renderer. It saves no images and opens no interactive GUI.
These measurements describe physics capacity and rendering cost separately;
they are not an end-to-end measurement of the interactive GUI's real-time factor.

Keep the exported files on the same computer: the configuration refers to local
robot, field, simulator, and PROS workspace paths. Keep those files in place until
the run finishes. The exported scripts use the installed simulator or the source
environment from which they were created.

The resulting report ZIP excludes raw PROS source and asset files. It may still
contain local paths, hardware information, configuration values, runtime
diagnostics, and messages printed by the robot program. Private build output and
the raw working folder are not included. Review the report before sharing it with
the team. Nothing is uploaded automatically.

## Compare solver iterations on the bundled claw demo

Use the [paired Windows capture helper](../tools/compare_claw_iterations/README.md)
to run the same claw experiment at 100 and 32 iterations with the installed EXE.
It prepares both profiles automatically, checks robot/workspace identity, and
produces one report ZIP. This avoids accidentally comparing the default idle
robot with the claw autonomy profile. See the
[32-iteration capture analysis](diagnostics-4070ti-iterations.md) for the reason
that a matched robot, program and background workload matter.

For server-side GPU validation, see the [A100 paired acceptance results](diagnostics-a100-iterations.md).
The headless harness validates lift and released goal placement as well as
performance. This can run over SSH without GUI forwarding; Windows captures
remain useful for Windows-specific renderer, driver and installer issues.

The [A100 activity and CPU worker diagnostics](diagnostics-a100-activity.md)
measure native sleeping bodies, contact conversion and verified worker counts.
They include a repeated full-claw regression and the opt-in source command for
further headless GPU testing.
