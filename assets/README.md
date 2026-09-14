The bundled robots are original, editable examples. The Override field uses
meshes converted from VEX Robotics' official STEP assembly; source hashes,
dimensions and transforms are recorded in `cad_manifest.json`. VEX retains its
geometry/design rights. The sample robot is not WVU's measured robot, and the
simulator is not a certified competition field. All URDF geometry uses metres,
kilograms and radians. JSON rotations use roll/pitch/yaw in radians. Experiment
`start_pose` is the exception: `[x_m, y_m, z_m, yaw_degrees]`.

The X-drive example binds the actual groups from `src/main.cpp`: front left
`15,17,-16`, front right `-8,-6,21`, back left `19,18,-20`, back right `-9,-1,10`,
and intake `11,-7,3,-14`. A negative port describes a physically reversed shaft.
The host adapter also implements PROS reversal: positive group motion therefore
produces the correct physical shaft commands and joint torques. ADI A/B/C move
the flap/lift/loader joints. The tank example uses ports 1/2 left, 3/4 right and
5 intake. Replace these assignments when selecting another workspace.

The sample robot has 10.73 kg total authored mass. Wheel diameter is 2 inches for
the X-drive, matching the repository. Mass distribution, gearing efficiency and
friction are examples requiring measurement. `generate_samples.py` regenerates
all supplied robot, object and game wrappers. Its offline goal/pin collider
generator requires NumPy and SciPy (the PhysX extra includes SciPy). Official
meshes are supplied separately and regenerated with `tools/import_vex_cad.py`.
Save your edited field or robot under another filename before running the sample
generator. From the repository root:

```sh
python assets/generate_samples.py
```

To generate only the corrected goal/pin collision assets into a separate review
directory, without overwriting source assets:

```sh
python tools/build_goal_pin_colliders.py --repository . --output-assets build/collision-assets --root-bands
python -m pytest tests/test_goal_pin_colliders.py tests/test_sample_collision_regeneration.py
```

Both generation paths retain the corrected goal lip and pin-root bands. The
original root OBJ/STL files remain as source profiles and negative-control test
geometry; the final pin URDFs reference the new axial-band STLs.

Robot binding files contain a `motors` array. Each row has a unique smart `port`
(1–21, optionally negative), a movable URDF `joint`, physical `cartridge`
(`red`, `green` or `blue`), optional `reversed`, legacy `rpm`,
`gear_ratio`, `stall_torque`, `max_power` and `efficiency`. Multiple ports may
drive one joint and their torques add. `gear_ratio` means motor revolutions per
joint revolution. For a prismatic joint, add `lead_m` in metres per screw
revolution; the same ratio then converts rotary motor torque to linear force.
All motor observations are physical shaft radians and radians/second, before the
host adapter converts them to PROS encoder units and applies configured reversal.

The V5 presets are red 36:1 (100 rpm, 2.1 N·m, 1800 ticks/revolution), green 18:1
(200 rpm, 1.05 N·m, 900 ticks/revolution), and blue 6:1 (600 rpm, 0.35 N·m,
300 ticks/revolution). Green/blue stall torque is inferred from the documented
red torque and cartridge reduction ratios. These are physical cartridges:
PROS `set_gearing()` changes software configuration and does not replace the
installed hardware. A mismatch produces one diagnostic while preserving the
physical cartridge. Bindings without `cartridge` infer it from `rpm`.

The full-voltage torque envelope is approximately digitized from
[VEX's motor performance graph](https://kb.vex.com/hc/en-us/articles/360044325872-Understanding-V5-Smart-Motor-11W-Performance).
It includes the current-limited low-speed plateau, torque at rated speed and
roughly 30% of the speed range above 11 W. VEX's specification table rates the
motor at 11 W; the performance graph shows about 12.75 W peak. `rated_power`
records 11 W and the default `max_power` is 12.75 W, keeping this distinction
explicit. `current_limit` defaults to 2.5 A and may be reduced. Current telemetry
is a torque-based estimate, not a winding/electrical model.

The nominal-speed governor, partial-voltage/back-EMF behavior and velocity/position
controller are simulation approximations. Motor targets update every 10 ms of
simulation time; Bullet's implicit velocity actuator enforces finite shaft
torque/current and speed limits without unstable explicit feedback on lightweight
shafts or screw joints. Position targets use adjustable `position_kp`. Battery sag, thermal
derating, inductance and V5's proprietary controller firmware are not modeled;
the simulator can sustain the graph peak without overheating. External gearing
multiplies shaft torque and divides shaft speed. `efficiency` defaults to 1 for
this external transmission; the published shaft torque already includes the
internal cartridge. Calibrate against acceleration, coast-down and loaded-speed
measurements. Source specifications:
[Understanding V5 Smart Motors](https://kb.vex.com/hc/en-us/articles/360060929971-Understanding-V5-Smart-Motors).

For `drive.type: "omni"`, every wheel row names its joint, radius and driven
`direction` in the base frame. Collision contacts provide normal load. A bounded
Coulomb force opposes actual contact slip along that direction, and much smaller
roller friction opposes transverse slip. Applying that force at the wheel
contact produces its reaction torque and pushes dynamic objects with an equal
opposite force. An inertia-based impulse bound stabilizes low-speed slip. No
traction is applied without a real upward-facing contact; the chassis pose and
velocity are never commanded directly. The tank preset uses Bullet's native
cylinder contacts and isotropic friction. General imported joints are actuated
through torque, velocity or position control, with URDF collision/limit response.

Every bundled game uses the same shared VEX foam-tile material and the same
experiment friction value. A new season does not silently change traction.
The default coefficient is a calibration starting point, not a published VEX
wheel/foam coefficient. Wheel compound, roller orientation, normal load and
the robot's mass still matter. The model permits slip when commanded contact
force exceeds available grip; calibrate grip once against the real robot and
carry that experiment setting between game packs.

`pneumatics` rows contain `port` (A–H or the equivalent incoming ADI number),
`joint`, `extended`, `retracted`, `force`, and `max_velocity`. Positions are
radians for revolute joints and metres for prismatic joints. This is a bounded
actuator model; reservoir pressure and airflow are not modeled.

`distance_sensors` rows contain `port`, `position` and `direction` in the robot
base frame, and `max_range` in metres. Put ray origins outside collision geometry.
Distance is a single ideal ray with optional noise, in millimetres; 9999 means no
return in range. Configure `sensors.otos` `x_sign`, `y_sign`, `heading_sign` and
`sensors.imu_heading_sign` for the code's coordinate convention. The default
Bullet frame is X forward, Y left and Z up. OTOS outputs X forward/Y right in
inches with counterclockwise heading in degrees; the repository negates that
heading. IMU heading is clockwise. Zero noise defaults give the odometry node
direct simulation coordinates. Noise changes sensor output only; `pose` remains
ground truth. Latency and sample-and-hold dropout use simulation time and the
experiment seed. Repeated reads/renders do not resample noise.

Game packs are ordinary JSON objects:

```json
{
  "schema_version": 1,
  "name": "My calibrated season field",
  "season": "2026-2027",
  "geometry_accuracy": "Describe measurement sources and remaining approximations",
  "field": {"size": [3.6576, 3.6576], "tile_size": 0.6096, "walls": true},
  "mode_durations": {"skills_driver": 60, "skills_auto": 60, "match_auto": 30, "driver_auto": 90},
  "objects": [
    {"name": "barrier", "shape": "box", "size": [0.1, 0.7, 0.2], "position": [0.6, 0, 0.1]},
    {"name": "ball", "shape": "sphere", "radius": 0.06, "mass": 0.12, "position": [0, 0, 0.061]},
    {"name": "custom_element", "urdf": "../objects/cup.urdf", "fixed": false, "position": [0.3, 0, 0.01]}
  ]
}
```

`box`, `sphere`, `cylinder` and `capsule` primitives support `mass`, `friction`,
`restitution`, `rolling_friction`, `position`, `rotation` and RGBA `color`.
Cylinders/capsules use `radius` and `height`. Zero mass means static.
`collision: false` creates a visual marking. URDF paths resolve relative to the
game JSON and support `fixed` and `scale`. `fixed: false` retains URDF masses;
an explicit `mass` overrides the base link mass. URDF mesh paths remain relative
to their URDF. Include mesh files alongside the exported robot or field asset.

The default Override pack uses the
[official 2026-27 field CAD](https://link.vex.com/docs/26-27/v5rc/field-cad) and
[manual](https://www.vexrobotics.com/override-manual), checked September 10, 2026.
The source assembly supplies exact positions and rotations for 9 goals,
4 loaders, 4 toggles, 36 cups and 37 pins. Match loads (20 cups, 22 pins) and
4 robot preloads remain inventory rather than being scattered onto the field.
The measured CAD grid pitch is 598.058534 mm. Pin overall dimensions are
80.293 by 76.793 by 165 mm: the large central flange joins two tapered ends.
Cups are 80 mm diameter and 164.5 mm high, with two hollow ends.

Pin dynamic collision uses two convex tapered ends, sampled rounded tips, a
beveled D-shaped flange, and four adjacent axial-ring convex bands per root.
The bands avoid filling the concave root fillet with one convex hull. Measured
cross-sections preserve the recessed local -Y face on both tapers and the tips'
2.25 mm D-flat. The 48-sided circular portions have under 0.09 mm chord error;
convexification within each leaf and omitted molded details add separate
approximation error. This is not a uniform 0.09 mm bound on the whole collision
model. Each pin has 13 URDF leaves; the PhysX exporter's exact subdivision for
GPU cooking limits produces 64 convex shapes per pin. Cups use hollow convex
sectors sampled from measured inner/outer profiles. The inner polygon circumscribes the circular opening (under 0.77 mm
angular error at the rim), avoiding false interference in the authored stacks.
Intermediate CAD samples preserve the tapered body and the true flared lip.
Their openings remain physically accessible.
Goals use 5,312-triangle hollow contact meshes. Their corrected inner profile
uses 64 radial samples per ring and preserves the CAD's horizontal annular
shoulder at socket z = 76.5 mm, instead of joining its edges with a diagonal
ramp. The original outer-wall vertices/faces are retained and the cap winding is
consistent. Molded ribs and fastener details remain omitted. Goal risers,
mounting plates and perimeter structural rails are box approximations.

`goal_profile.json` records the official `goal_socket.obj` source hash and sampled
inner/outer sections; `pin_root_bands.json` records the source root profiles and
band bounds. The source CAD definition is `276-9250-003_Master`, with original
STEP provenance in `cad_manifest.json`. Displayed CAD meshes have a declared
0.25 mm chord tolerance. An offline comparison at 216 axial heights and 192
angular rays measured a maximum inward goal-profile deviation of 0.129 mm and
outward deviation of 0.026 mm relative to that triangle mesh; these are sampled
measurements, not global analytic-surface error bounds. Static tests find no
strict pin/goal mesh intersections at the original five seated-pin poses.
Contact offsets, native cooking, insertion/extraction, and stacking still need
backend-specific validation. No performance or physical-accuracy guarantee
follows from those static checks. Official positions, visuals, mass, inertia,
friction, and solver settings were not changed by this collider correction.
The white panels use their exact exported CAD shape and installation positions.
Toggle visuals separate the actual fixed mounts and rotor at the authored axle;
rotor contact uses its conservative measured box envelope. These collision models
approximate contact mechanics, even though displayed shapes derive from official CAD.
CAD pins, cups and goals use a 0.1 mm collision margin; Bullet's default 1 mm
margin noticeably raised pins inside the narrow tapered socket. The small generated STL collision meshes avoid the nested compounds inserted
by Bullet's OBJ convex importer, so the margin reaches the actual hulls. OBJ
vertex copies remain alongside them for inspection. Field URDFs retain reduced
coordinates with native sleep and contact-driven wakeup enabled. For these
jointless reduced bodies, Bullet compares the sum of squared linear and angular
speed with `sleep_speed**2` (default 0.0001), a mixed-unit criterion sustained for
two seconds. A second numerical-rest check requires two seconds within a 0.25 mm
COM bounding-box diagonal and a 0.2-degree quaternion orientation range, actual
support contacts, and a resting contact group. Separate instantaneous impact
ceilings are 0.01 m/s linear and 0.1 rad/s angular speed; these permit bounded
angular jitter without reusing the native mixed-unit test. It excludes contacts with the
robot, articulated mechanisms and moving props. It removes only the bounded
residual kinetic energy before requesting native sleep; positions, masses and
collisions remain unchanged and impacts wake the pieces normally. Set experiment
`numerical_rest: false` to disable this additional check.

Persistent friction anchors are disabled by default because the installed
Bullet version can inject energy into rolling bodies when anchors are enabled.
A 2 mm contact-cache breaking threshold avoids a separate rolling-energy defect
observed at 1 mm; this is a cache setting, not collision-shape inflation. The
shared tile/wheel coefficient remains `friction: 0.9`, while CAD plastic parts use
`plastic_friction: 0.5`. Bullet multiplies body coefficients, giving plastic/plastic
0.25 and plastic/tile 0.45 by default. These are unmeasured calibration presets,
not official VEX material specifications. Per-object `friction`, `friction_anchor`,
`collision_margin` and `sleep_speed` remain available for controlled experiments.
Dense faceted socket contacts can settle several millimetres from authored CAD
assembly poses, produce high opposing normal forces, and remain expensive while
active. Physics slows wall-time progression without skipping fixed steps.

`override-route-planning.json` is an optional experiment derived by the sample
generator from the full field pack. It fixes all 36 cups and 37 field pins in
place to reduce CPU cost during route planning. Their poses, geometry and
materials match the full pack, but they act as immovable obstacles. Robot and
toggle joints retain their dynamics. Use the full pack for game-piece interaction
tests; separately configured experiment objects retain their own mass settings.

Pin and cup mass is set to 75 g each: an equal-split assumption based on the
[official kit's 150 g combined weight](https://www.vexrobotics.com/276-9255.html),
not published individual masses. Inertia and friction still need calibration.

The simple `xdrive_claw` robot uses four blue motors (ports 1-4), a red lift motor
(5, 40 mm screw lead), and a red sliding-jaw motor (6, 20 mm lead). Slim jaws
grip the pin's upper taper through friction; their faces tilt 0.208 radians to
match that taper, with the flange below the jaws.
The jaws never create attachment constraints. The prototype placement counter
requires an upright pin in a goal socket, actual pin-goal contact, and release
from the robot. This is a demonstration metric: official rules can count held
pins, and full scoring, color orientation, stacks and violations are not adjudicated.

Timers default to VEX U's 30-second autonomous and 90-second driver periods;
select 15/105 in the experiment for V5RC. Explicit experiment duration overrides
take priority over game-pack durations.
