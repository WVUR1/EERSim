# Desktop feedback and troubleshooting

Version 0.1.9 adds a detachable console, setting help icons, visible errors and
loading feedback to the simulator and PID tuning utility.

## Console

Use **View → Console**, the **Console** button, or **Ctrl+Shift+C** to show the
console in the active window. It starts hidden each time. **Detach** moves it
into a separate window; **Dock** puts it back. Closing the console hides it and
keeps collecting messages. It does not stop the simulator or recording.

**Copy**, **Save…**, and **Clear** work on the console output. Messages have local
timestamps. The console batches screen updates and bounds its memory usage;
it is a recent-message buffer, not an unlimited recording. Save important logs
before a long session overwrites older output. Hardware capture exports and
performance diagnostic bundles remain the tools for complete measured data.

## Setting help

Hover over a **?** icon for an explanation, or click it to keep the explanation
visible. The icons also accept keyboard focus and expose their descriptions to
screen readers. Help explains units, physical effects, performance tradeoffs,
and whether a setting changes the physics, the view, or only a tuning test.
Robot motor tables include header help for port, joint, reversal, gear cartridge,
external ratio and torque. These explanations do not replace calibration against
your real robot.

## Errors and warnings

Failures show a popup with the action that failed and suggested next steps.
Detailed error text can be copied from the dialog and is retained in the console.
Popups do not stop Qt's event loop. Repeated identical errors are suppressed,
and other pending messages are bounded so one failing operation cannot cover
the screen in dialogs. Closing a warning does not retry the operation.

The **PROS CLI is required for physical firmware builds, uploads and terminal
recordings** in the tuning window. Choose the CLI executable in that window or
install PROS and make its command available in the environment. Simulator
workspace builds instead use the desktop C++ compiler bundled with the installer;
absence of PROS alone does not prevent simulation. Missing or incompatible
tools are reported when an operation needs them.

Unexpected Python desktop exceptions also report their traceback. A Windows
startup failure can show a native error dialog even when Qt is missing. Windowed
executable stdout/stderr uses the per-user `EERSim/logs/eersim.log` location
(under `%LOCALAPPDATA%` on Windows). A native library or graphics-driver crash
can terminate the process before any Python popup appears; use an exported
performance diagnostic bundle to investigate those failures.

## Loading feedback

A separate loading bar reports scene configuration, native workspace builds,
simulation startup, Reset, replay preparation and telemetry export. It does not
replace the competition time bar or replay timeline. Native scene creation and
compilation have unknown durations, so these stages use an animated bar and an
operation label. They do not display a fabricated completion percentage.

Scene and Reset operations stay busy until the new live view is ready. Replay
preparation stays busy while its view loads. The tuning utility uses an animated
bar for jobs of unknown length and measured fractions for the candidate search.
Failure or cancellation clears the busy state without presenting a successful
100% result. A loading indicator makes waiting visible; it does not itself make
physics initialization or asset loading faster. Some short file dialogs and
metadata operations finish without a meaningful progress estimate.

Workspace compilation remains an explicit **Build workspace** action. Applying
settings, Reset, replay and loading a robot do not trigger an automatic rebuild.

## Validation for 0.1.9

The focused regression selection passed **343 tests**, with **two platform-specific
skips**. It covers real offscreen Qt windows, hidden/floating console lifecycle,
setting help, queued-error suppression, missing-PROS feedback, laptop layouts,
progress stages, failed telemetry export recovery, and existing engine, replay,
tuning and native-bridge behavior. The supervised test process tree exited with
no remaining child processes.

These checks include mocks and CPU execution. They do not establish CUDA
performance, physical-robot tuning accuracy, or installation/upgrade behavior on
every machine. Native process crashes still require diagnostic logs. Windows
installer build and bundle checks are recorded separately with the local build.
