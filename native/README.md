# Native PROS adapter

EER Sim compiles the selected workspace's C++ sources on the desktop with a
replacement PROS header and runtime. Validation against a separate EERKit
checkout compiled its 11 C++ translation units without source edits, including
its serial OTOS task. Select any compatible external PROS workspace; EERKit is
not required for the bundled examples or the native fixture tests.
The original ARM api.h and pros headers are replaced only in a build staging
directory; robot sources and deployment files remain intact.

Supported APIs are declared in eersim_api.hpp: Motor and MotorGroup voltage,
velocity and position commands, gearsets, reversal, encoder units, brakes,
controller analog/digital input, LCD text, Imu heading/rotation/calibration,
Rotation, Distance, ADI DigitalOut/Pneumatics, Serial OTOS, Task, Mutex and
competition state. The C++ pros::c namespace has basic motor and delay wrappers.
Other methods fail compilation/linking. Unsupported runtime operations raise a
named error and stop the simulation. Sensor reads without bindings are errors.
C translation units, ARM binary libraries, arbitrary UART devices, vision,
GPS, optical devices, embedded LVGL and external ADI expanders are not supported.

## Scheduling and time

Each native PROS task has a host thread, but only one task runs at a time.
Workers run in stable creation order until they call pros::delay or block on a
simulated mutex/serial read. The host sends a hardware snapshot at each fixed
physics tick and receives actuator commands only after all due tasks yield.
Task priorities are currently cooperative; a nondefault priority emits a
diagnostic. This scheduling is deterministic, and does not emulate FreeRTOS
preemption or reproduce hardware data races. Delays round up to physics ticks.

No physics time elapses while robot code executes. A task which fails to yield
within two wall-clock seconds triggers a visible watchdog failure. Thus overloaded
hardware slows wall-clock playback instead of skipping physics steps. Physics
time is supplied explicitly as snapshot.time (seconds); millis and micros read
that clock. Robot code using wall time, standard threads or host I/O bypasses this
contract and should not be used for deterministic control.

Initialization runs initialize then competition_initialize. The initialized
flag becomes true when both return. Both callbacks can drive motors and ADI
outputs and receive updated sensor feedback; initialization advances physics
without consuming competition time. A mode transition cancels the old callback
at its yield point, clears motor commands and invokes the new callback. Background
tasks persist, as they do on the brain. Disabled mode suppresses motor output.
A fresh child process is required to reset globals and rerun initialization.
Skills Auto and Match Auto both invoke autonomous; Skills Driver and Driver Auto
invoke opcontrol. Match duration and skills selection belong to the host session.

## Hardware contract

Motor feedback uses radians and radians/second in physical port direction.
Commands return voltage in mV (clamped to +/-12000), velocity in RPM, or absolute
position in degrees. For position commands, velocity is the maximum profile RPM.
Commands also carry brake (coast, brake, hold) and gearset free RPM (100, 200, 600).
Signed port reversal and user encoder units are resolved by the native runtime.
Motor electrical telemetry is not fabricated: get_voltage in closed loop mode
reports an unsupported operation.

IMU snapshots provide heading and rotation in clockwise degrees. Rotation
sensors accept separate snapshot.rotation entries or a motor entry at their
bound port. Distance snapshots are millimeters. ADI command keys are 1..8,
corresponding to A..H; values are electrical output levels, including active-low
pneumatic configurations.

Serial implements EERKit's protocol: ASCII ! then a 20-byte, little-endian
SerialMessage (float type and a 16-byte union). Incoming type 1 contains the
snapshot.otos [x inches, y inches, heading degrees] verbatim. Transmitted type 4
changes the sensor coordinate offset; type 3 sets that offset directly. This
changes reported odometry, never physical robot pose. The bundled world maps
Bullet +X forward/+Y left/CCW yaw to OTOS [X inches, -Y inches, yaw degrees],
matching EERKit's heading negation. With noise disabled these are exact world
coordinates; no Jetson or external odometry process is necessary.

## Building

The builder detects CXX, clang++, g++, or MSVC cl in a Developer Command Prompt.
Windows MinGW/LLVM compiler DLL directories are persisted alongside the executable.
Builds live in the selected workspace's .eersim/build and are content-cached.
Optional eersim.build.json supplies lists of defines, include_dirs and additional
sources; every path must stay in the selected workspace. Firmware Makefile flags
and ARM libraries are not reused.

Run the integration tests with a host compiler available:

    python -m unittest discover -s tests -p test_bridge.py -v

These tests build a real C++ fixture and check source preservation, quoted-header
replacement, task determinism, sensor units, reversal, gearset limits, mode
changes, controller edges, position commands, active-low pneumatics and watchdogs.
