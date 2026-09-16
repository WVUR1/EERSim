"""Offline walkthrough content for the PID lab, included in frozen builds."""

GUIDES = {
    "Simulation: step by step": """
<h2>1. Choose what the controller measures</h2>
<p>First apply the intended robot, field, start pose and motor mapping in the
main simulator and wait for the scene to be ready. Open <b>Setup</b>, select a
<b>Target</b> and its <b>Axis</b>, and check the
displayed ports. For the claw bot drivetrain, choose the <b>X-drive from robot
mapping</b> and start with <b>forward</b>. This mapping is inferred from the
imported robot; verify its directions in the viewport. A shared motor group
commands every member in the same logical direction. An X-drive target also
provides the different mixtures needed for turn and strafe.</p>
<p>Choose <b>Position</b> to control relative motor-shaft degrees or <b>Velocity</b>
to control motor RPM. Drivetrain feedback is a projection of wheel encoders;
it is not robot position in inches/metres or heading in IMU degrees. Tune one
axis at a time. Driving just one grounded wheel of an X-drive can scrub the
other wheels and change its load, so it may not fit a simple motor model.</p>
<h2>2. Prepare a repeatable scene</h2>
<p>In the main simulator, apply the intended robot, field, start pose and motor
cartridges/gearing. Choose a clear test area with room for travel and coast-down.
Click <b>Reset simulation</b> in the tuner before a repeat trial and wait for
reset to finish. Reset restores the applied starting scene. If the robot starts
against a goal, wall or game object, change its starting pose and apply it first.</p>
<h2>3. Set the experiment in Setup</h2>
<p>The <b>Characterization / output limit</b> is a motor-voltage pulse in mV,
not a PID gain. <b>Characterization pulse interval</b> controls its duration.
<b>Maximum travel from start</b> bounds each participating motor's encoder
displacement, including external gearing. Set this within the mechanism's
usable travel. The trial also records a baseline and coast response.</p>
<p>For the demo X-drive, click <b>Short drivetrain pulse</b> to set 2000 mV for
0.2 s, then verify the travel bound. This produced a usable capture in CPU
PhysX at 120 Hz with the default claw bot and a 180° bound. It is a starting
point, not a guarantee for every backend, robot or load. The button preserves
your travel bound and the separate <b>Validation interval</b>, which defaults
to 2 s so the candidate PID has time to settle.</p>
<p>For a trial that hits its travel bound, first shorten the pulse interval
(for example, 2 s to 0.5 s) or reduce the voltage (3000 mV to 1500 mV).
Increase travel only if the whole robot has clear space and the mechanism can
move that far. These are adjustment examples, not calibrated presets.
Read <b>Settings explained</b> for each parameter and its tradeoffs.</p>
<h2>4. Capture the response</h2>
<p>Click <b>Characterize in simulation</b>. It stops the simulated robot program
and drives the selected ports through the physics engine. No PROS build is
needed for this experiment. Watch <b>Live capture</b>: the voltage trace should
show a pulse and the response should move smoothly in the selected direction.
Check the completion message for an early travel-limit stop.</p>
<p>To inspect existing code instead, first build changed code explicitly with
<b>Build workspace</b> and wait for the build to finish. Then choose <b>Record
live simulation</b> and start the normal program from the main window. Building
after arming capture stops that recording. The claw demo uses <code>move_velocity</code>, which
does not expose applied electrical voltage in the simulator; those traces can
be inspected, but use <b>Characterize</b> for voltage-response fitting.</p>
<h2>5. Fit, then validate on the full scene</h2>
<p>In <b>Tune / results</b>, click <b>Analyze response</b> to inspect the capture,
then <b>Find candidate gains</b>. Read the model-quality details and warnings.
The plotted fitted-model prediction is a simplified model, not another physics
run. A successful fit does not prove the gains work on the complete robot.</p>
<p>Click <b>Reset simulation</b>, wait for completion, then <b>Validate in
simulation</b>. Validation uses the saved candidate settings and actually
drives the current robot in the field. Inspect its recorded response in
<b>Live capture</b> for overshoot, oscillation, settling and travel-limit stops.
Changing a tuning setting invalidates the candidate; reset alone preserves it.</p>
<h2>6. Save and repeat</h2>
<p><b>Export gains / report</b> saves gains, units, settings and model quality.
<b>Export capture</b> saves the raw recording for later analysis or a bug report.
Repeat with representative directions, payloads and starting conditions.
Physical-robot validation is a separate step; the simulator cannot measure
your actual friction, backlash, compliance or battery condition.</p>
""",
    "Physical robot: step by step": """
<h2>1. Select the real mechanism and controller units</h2>
<p>Select the actual PROS workspace, then verify the target's signed ports and
axis. A robot-mapping target describes the imported simulated robot; confirm
it matches the real wiring and reversals. The logger reads positive physical
ports and the target applies the reversal signs during analysis.</p>
<p>Choose Position (motor degrees) or Velocity (motor RPM), and match your
controller's loop period. Hardware logging has a minimum period of 20 ms;
capture and validate at the rate you intend to use. Millivolt output and time
in seconds are part of the gain units.</p>
<h2>2. Prepare the telemetry copy</h2>
<p>A working <b>PROS CLI and ARM toolchain</b> are required. In <b>Live capture</b>,
use <b>CLI…</b> if automatic discovery cannot find PROS. Connect the Brain by USB.
Click <b>1 · Prepare logger copy…</b> and choose a writable parent folder outside
the original project. This creates a new workspace with passive motor telemetry.
It preserves your original project and callbacks. Standard PROS 4 projects with
a global <code>initialize()</code> are supported.</p>
<h2>3. Build and upload explicitly</h2>
<p>Click <b>2 · Build</b>. Check the console if compilation fails. Choose the V5
program slot and optional serial port, then click <b>3 · Upload</b>. This replaces
that slot with the instrumented program; it does not start the program. The
desktop simulator compiler alone cannot produce V5 firmware. Already-uploaded
firmware cannot acquire the telemetry logger without this build/upload step.</p>
<h2>4. Record one controlled trial</h2>
<p>Place the robot at its intended start with enough room for the mechanism.
Click <b>4 · Connect / record</b>, then start the uploaded program with its normal
controls. Logging begins after the original <code>initialize()</code> returns.
Use the program's existing controls or routine to produce a stopped baseline,
a controlled move and a coast/stop response under consistent load.</p>
<p>The logger only reads telemetry; it does not create a test routine, reset
encoders or command motion. The simulator's voltage and powered-interval
settings do not drive the physical robot. If the program does not provide a
suitable repeatable movement, add one in the logger copy and explicitly rebuild
and upload it. Avoid taring encoders or restarting the program during a trial.</p>
<h2>5. Stop, inspect and fit</h2>
<p>Stop the robot using its normal controls, then click <b>Stop / cancel</b> to
end recording. <b>Stopping recording does not stop the physical robot.</b>
Export the capture, inspect it with <b>Analyze response</b>, then select
<b>Find candidate gains</b>. A poor fit needs a better experiment or a more
suitable mechanism model; do not assume a higher gain fixes a poor recording.</p>
<h2>6. Apply and verify on hardware</h2>
<p>The utility does not overwrite PID constants or run candidate controllers
on the physical robot. Apply the candidate in your own code only after checking
the feedback units, output units, loop period, integral behavior and travel
limits. Build/upload that change explicitly. Start with a modest target and
record its real response using the same workflow. Return the robot to its start
between trials and compare different directions and payloads.</p>
<p><b>Validate in simulation</b> runs the imported simulated robot, even when the
candidate came from a hardware recording. It is not physical validation.
The physical logger workflow has not yet been verified on a connected V5 robot.</p>
""",
    "Settings explained": """
<h2>Target, axis and direction</h2>
<p>A <b>single motor</b> or <b>shared group</b> has a shaft axis. Tank, H-drive and
X-drive targets mix motors for forward, turn and (where supported) strafe.
Negative port numbers reverse commands and feedback. Check directions before
fitting: motors fighting each other produce little movement or changing load.
Select the complete drivetrain when tuning its movement on the floor.</p>
<h2>Controller: Position or Velocity</h2>
<p>Position uses degrees relative to the capture's starting motor encoder
position. Velocity uses RPM. The same gains cannot be reused between these
modes. A wheel encoder degree differs from a degree of robot heading. External
gearing also means one motor revolution may be less than one wheel revolution.</p>
<h2>Characterization / output limit — millivolts</h2>
<p>This is both the simulation characterization pulse voltage and the candidate
controller's output cap. 3000 mV = 3 V; 12000 mV = 12 V. Too little voltage can
leave a loaded mechanism stuck; too much can reach an obstacle or travel bound
before useful data is collected. For weak motion, try 1500 → 3000 mV after
checking motor direction and load. For rapid travel, try 3000 → 1500 mV and a
shorter interval. Changing this setting requires finding a new candidate.</p>
<h2>Characterization pulse interval — simulation seconds</h2>
<p>This is the powered portion of characterization. The experiment
also includes a short baseline and coast interval. Increase it when motion is
still accelerating and travel permits; reduce it when a fast drivetrain uses
up the travel allowance. Slower rendering does not change simulation seconds.
Identification needs at least 0.6 s and 35 samples in the entire capture.</p>
<h2>Validation interval — simulation seconds</h2>
<p>This is how long to run the candidate PID step in the complete scene. The
model search evaluates for at least this long and can extend its prediction
for a slow response. Keep time for the robot to settle: a 0.2 s characterization
pulse does not imply a 0.2 s validation. The default is 2 s; extend it for a slow
position response. For velocity control, the robot keeps moving, so check the
travel allowance and use a shorter validation interval if necessary.</p>
<h2>Maximum travel from start — motor degrees</h2>
<p>Each active motor is checked separately against its starting position; a group
average cannot hide one motor exceeding the bound. This bounds displacement,
not total distance traveled during oscillation. The check is sampled at physics
ticks and disabling a motor is not an instantaneous physical stop. Leave room
for coast-down and mechanical limits.</p>
<p>180° is half a motor revolution. With the default claw bot's 2:1 reduction,
720 motor degrees is one wheel revolution. A larger bound can allow a useful
drivetrain recording, but would be unsuitable for some lifts or claws. Inspect
the mechanism and available space before increasing it.</p>
<h2>Controller period — milliseconds</h2>
<p>This sets how often the candidate PID updates, not the renderer FPS.
20 ms means 50 updates per second. The simulator rounds to whole physics ticks
and exports the actual interval. Smaller intervals need correspondingly sampled
data; the fitter refuses a controller faster than the recording supports.
Hardware logging has a 20 ms minimum. Match the actual interval in your code.</p>
<h2>Validation step — motor degrees or RPM</h2>
<p>This is the candidate's target, not the characterization pulse length.
For Position, 90 means a relative 90 motor-degree move; it must be inside the
travel bound. For Velocity, 60 means 60 motor RPM and travel continues throughout
the powered interval. Reduce target or interval if validation reaches its travel
limit. Negative values test the other direction; fit data for that direction
and load before relying on the result.</p>
<h2>EERKit integral activation error</h2>
<p>Integral accumulates only when nonzero absolute error is below this threshold;
otherwise it resets. Units match the error: degrees or RPM. A smaller threshold
restricts integral action to near the target; a large threshold enables it
throughout most of the move. For a 90° position target, a 10° threshold waits
until the remaining error is under 10° before accumulating.</p>
<h2>EERKit integral limit — error × seconds</h2>
<p>The PID clamps the integral value used in its output before multiplying by
Ki; the stored accumulator itself can keep growing. A limit of 100 degree·s
and Ki = 2 mV/(degree·s) caps that contribution at 200 mV. A larger limit can
correct sustained load but can increase windup and recovery time. When Ki = 0,
neither integral setting changes the output.</p>
<h2>Hardware controls</h2>
<p><b>Program slot</b> chooses the Brain slot replaced by Upload. <b>Serial</b>
selects a particular device (for example COM4); empty uses PROS discovery.
<b>CLI…</b> selects the installed PROS executable. Preparing, building,
uploading and recording are separate actions. None starts robot motion.</p>
""",
    "Troubleshooting and gain units": """
<h2>“Response does not fit reliably”</h2>
<p>The measured response does not match the model well enough to propose gains.
Read the error's fit metrics and suggested next steps. Common causes include
one wheel scrubbing against the other grounded wheels, a collision or hard
stop, changing load, inconsistent motor directions, or a very short pulse.</p>
<p>For the claw bot, select the full X-drive and one axis, reset in clear space,
and capture a fresh trial. Inspect the voltage and response traces. If the
travel limit stops it early, shorten the interval or reduce voltage; increase
travel only when the mechanism and field allow it. If motion is too small,
check port signs and obstruction before increasing voltage. If otherwise good
trials still fail, export the capture for diagnosis instead of repeatedly
changing gains.</p>
<h2>“Encoder position jumps” / discontinuity</h2>
<p>The encoder displacement disagrees with the recorded velocity at a particular
time. This is evidence of inconsistent data, not proof that you forgot to tare.
Possible causes include a tare/reset within the recording, feedback latency,
a sharp contact impulse or a simulator encoder fault. The error shows the
offending time and values. Do not tare in the middle of a recording.</p>
<p>Export the failing capture first. Reset before starting a new trial and keep
one uninterrupted run per capture. For the simulation, retry from clear space.
If the error repeats with no reset or collision, keep that capture and include
the backend, physics rate, robot and port mapping in the bug report.</p>
<h2>Too few samples, missing voltage or timing gaps</h2>
<p>A travel-limited trial may end before the required 0.6 s / 35 samples.
Adjust the pulse and travel allowance rather than extending a run into an
obstacle. Simulator position/velocity commands do not supply voltage telemetry;
use Characterize for fitting. For a physical capture, check USB/serial delivery
and record again if there are large gaps. Import one continuous trial per file.</p>
<h2>Why can Kp look large, with Ki and Kd equal to zero?</h2>
<p>The output is in <b>millivolts</b>. For Position, Kp has units mV/degree,
Ki has mV/(degree·s), and Kd has mV·s/degree. For Velocity, replace degree with
RPM. These numerical values are not directly comparable with gains for volts,
normalized power, inches/metres or IMU heading.</p>
<p>For example, <b>Kp = 202 mV/degree</b> asks for 2020 mV at 10° error.
At 90° it asks for 18180 mV, which clips to the configured limit. With a
3000 mV cap, the proportional term alone clips above about 14.9° error.
Expressed in volts rather than millivolts, that same proportional gain is
0.202 V/degree. All three gains and voltage-valued limits must use the same
output-unit conversion; feedback-unit changes require separate conversion.</p>
<p>A P-only candidate can be appropriate for a damped motor-position response:
integral can increase overshoot and derivative may not improve the selected
trials. The search may therefore choose zero I/D. That alone does not prove the
candidate is good or bad. Check fit quality, saturation, settling, steady-state
error and the full-scene validation trace. A gain that clips for most of a move
is poorly constrained by that trial; repeat with smaller steps and loads before
copying it to robot code.</p>
<h2>Save evidence and open the console</h2>
<p>Use <b>Export capture</b> for raw data and <b>Export gains / report</b> for a
candidate and its settings. <b>Console</b> shows operation details and can be
detached by dragging its title bar. Error dialogs include copyable details.
These files are more useful for diagnosis than a gain value alone.</p>
""",
}
