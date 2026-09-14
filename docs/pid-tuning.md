# PID tuning lab

Click **PID tuning…** beside **Build workspace** to open the separate tuning
window. It uses the selected PROS workspace and the robot's motor bindings.
Discovery, recording, response fitting, and candidate search do not rebuild the
workspace. Simulation recording uses the existing PROS bridge; your source needs
no logging calls.

![PID tuning lab showing candidate response curves](pid-tuning.png)

The screenshot uses illustrative response data, not a physical-robot capture.
See the [0.1.8 validation record](pid-tuning-0.1.8.md) for tested scope.

## Select a mechanism

Choose a **Target** and **Axis**, then check the displayed ports and source.
**Refresh workspace** rescans changed source. **Add group** and **Edit mapping**
let you configure a single motor, shared motor group, tank drivetrain, H-drive,
or X-drive. Manual mappings are remembered for that workspace on this computer.

| Topology | Motor assignments | Available axes |
| --- | --- | --- |
| Single motor / shared group | Motors moving in the same logical direction | shaft |
| Tank | left, right | forward, turn |
| H-drive | left, right, lateral | forward, turn, strafe |
| X-drive | front-left, front-right, back-left, back-right | forward, turn, strafe |

Enter the signed ports in each side or corner's field; every drivetrain field
accepts multiple motors. For example, an H-drive can use left `1, 2`, right
`-3, -4`, and lateral `5`. The dialog generates forward, turn, and strafe
coefficients automatically. Negative ports reverse the motor's logical
direction. A physical port cannot appear twice in one target; it may belong to
several selectable targets.

For a custom mixture, enable **Advanced: edit axis coefficients directly**.
This optional JSON editor has one coefficient per listed port: `1` follows an
axis, `-1` opposes it, and `0` excludes that motor from that axis. The ordinary
side/corner assignments do not require editing JSON.

Discovery recognizes PROS `Motor` and `MotorGroup` constructors, literal ports,
integer constant aliases and object-like `#define`s, literal port vectors, and
EERKit `TankDrive` / `XDrivetrain` group references. It preserves signed ports,
supported constructor reversal flags, namespaces, and source locations. Explicit
`eerkit::HDrive` / `HDrivetrain` declarations use a conventional left/right/lateral
order and are labelled for verification.

This is a source scan, not C++ execution. Factories, arbitrary wrappers, computed
ports, runtime changes, and conditional build configurations cannot be inferred
reliably. Unresolved definitions appear in the Setup notes. A source-resolved
target confirms the declaration's syntax, not the connected robot. Simulation
binding fallbacks and manual mappings are labelled separately.

## Capture, tune, and validate in simulation

1. Import the robot and map the selected motor ports to its joints in the main
   window, then **Apply settings**. Use **Reset simulation** in the tuning window
   when you need to return to the applied starting scene.
2. In **Setup**, choose position or velocity control, controller period, voltage
   limit, powered interval, travel limit, and validation step. Set the integral
   activation error and integral limit to match the controller you intend to use.
3. Click **Characterize in simulation**. This stops the robot program and manual
   joint commands, then drives the selected axis through mapped motor ports with
   a bounded voltage pulse. The capture includes a stopped baseline and coast
   interval. The test ends if a selected motor reaches its travel limit.
4. In **Tune / results**, use **Analyze response** to inspect the recording, then
   **Find candidate gains**. The search fits a delayed first-order velocity model
   and compares controllers across variations in model gain and response time.
5. Reset to the intended starting condition, then click **Validate in simulation**
   to exercise the candidate on the complete robot and field. This also stops
   robot code and owns the selected motor commands during the test. Compare the
   recorded response with the fitted-model prediction; they are different tests.
6. Use **Export gains / report…** to save the candidate, controller settings,
   model, metrics, and prediction as JSON.

To inspect existing robot code instead of running a generated pulse, choose
**Live capture → Record live simulation**, then run the normal competition
callbacks in the main window. Build changed source explicitly with **Build
workspace** before the run. **Stop / cancel** ends capture; during a tuning
experiment it also disables the simulated motors. A new trial needs a new
capture after Reset.

Identification needs a continuous recording containing a stationary baseline,
changing voltage, measurable motion, and coast-down. Simulation VEX velocity and
position commands do not expose actual electrical voltage, so those captures can
be inspected but cannot supply this voltage-response fit. Use voltage-driven code
or **Characterize in simulation**. Do not fit through an encoder tare, collision,
hard stop, wheel slip, or changing payload and expect one motor model to explain
the result.

## Record a physical robot

The hardware workflow adds passive telemetry to a separate copy of a standard
PROS 4 project with a global `initialize()` callback. Your original workspace is
unchanged. It requires a working PROS CLI and ARM toolchain; EERSim's desktop
robot compiler alone cannot build V5 firmware. An already-flashed binary cannot
acquire this logger without building and uploading the instrumented copy.
The CLI is discovered through `PATH`; use the optional executable **Browse…**
field when it is installed elsewhere. Hardware recording supports a minimum
20 ms sample period.

1. In **Live capture**, click **1 · Prepare logger copy…** to create a new logger
   workspace outside the original project. Preparation copies the project and
   adds logging for the selected physical ports; it does not overwrite an
   existing destination.
2. Click **2 · Build**. This explicitly builds the logger copy with PROS.
3. Select the V5 program slot and, if necessary, the serial port. Click
   **3 · Upload** to replace that slot with the instrumented program.
4. Click **4 · Connect / record**, start the uploaded program using the robot's
   normal controls, and perform the movement with its existing callbacks.
   Return the robot to its starting condition between trials.
5. Click **Stop / cancel**, inspect the capture, and use the same analysis and
   candidate-search controls as above.

The logger does not start a robot program or command motor motion. Stopping the
terminal recording does not stop the physical robot: its program and normal
controls remain responsible for motion. The utility does not automatically
replace PID constants or run candidate controllers on hardware. Physical V5
logging has not yet been tested on a connected robot.

Logging begins after the original `initialize()` returns. Hardware sampling is
20–1000 ms and reads motors sequentially; serial and CPU overhead can reduce the
achieved rate. Motor telemetry is converted to degrees without changing encoder
units or taring the robot. Unsupported callback layouts or legacy reversal APIs
are reported during preparation instead of rewriting them speculatively.

## Units and reusable captures

These gains are for an outer PID with **millivolt output**, error measured in
**projected motor degrees** or **projected motor RPM**, and time in **seconds**.
Drivetrain axes combine motor encoders; they are not odometry distance or IMU
heading. Do not copy the numbers directly into an inches/metres path controller,
a `move(-127…127)` controller, or VEX's onboard velocity/position controller.

The controller follows EERKit's derivative-of-error behavior. It accumulates
`error × dt` only while the nonzero absolute error is below the activation
threshold, otherwise resetting the integral. The configured integral limit
clamps the integral sum used in the output before multiplication by Ki, without
clamping the stored accumulator. Output is capped at the selected millivolt limit. Preserve these
settings and the actual controller period when transferring gains; simulation
periods are aligned to fixed physics ticks.

**Export capture…** saves JSON Lines (`.jsonl`) or CSV. **Import capture…** reads
the same formats so another computer can fit and compare a recorded trial.
CSV uses `time,source,capture_id,port,position_deg,velocity_rpm,voltage_mv`, with one
row per physical motor at each timestamp. Time is seconds; port identifiers in
the capture are positive physical ports, with reversal applied by the selected
target. Keep each file to one continuous trial and one capture source.

An accepted model and its simulated response provide candidate gains, not
hardware validation. Spec-sheet motor curves do not identify a particular
mechanism's friction, backlash, compliance, battery behavior, or load. Compare
physical recordings and repeat representative loads and directions before
relying on a candidate for the actual robot.
