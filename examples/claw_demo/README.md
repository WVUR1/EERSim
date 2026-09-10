# X-drive claw demo

Use **Load claw scoring demo** in EER SIM to load the robot, Override field,
one red/yellow preload, and this source workspace. The button builds the program
and selects Skills Auto. Press Start to grip, lift, drive to the nearby red goal,
and release the pin.

Skills Driver uses the normal controller: left stick translates, right X turns,
R1 closes the claw, R2 opens, L1 raises the lift, L2 lowers. The import preview
can jog each motor independently before running the program.

Ports 1–4 are blue-cartridge drive motors with 2:1 external reductions.
Port 5 is the red-cartridge lift (.04 m/rev lead), port 6 the red-cartridge
claw (.02 m/rev lead), and port 7 carries the EERKit OTOS serial pose packet.

This example contains ordinary PROS source and the entry header needed by the
simulator's host builder. To deploy on hardware, place it in a normal PROS
project and configure/calibrate the physical robot and odometry accordingly.
