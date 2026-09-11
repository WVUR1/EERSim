`audit_joints.py` checks saved joint poses and velocities without starting PhysX, a GUI or CUDA. Use the export environment, which supplies NumPy and `usd-core`; the ovphysx runtime environment is separate.

From an EERSim source checkout:

```sh
python tools/physx_prototype/audit_joints.py \
  --scene build/physx-field/field.usda \
  --poses build/physx-gpu/final_poses.npy \
  --velocities build/physx-gpu/final_velocities.npy \
  --rows build/physx-gpu/pose_rows.json \
  --output build/physx-gpu/joint-audit.json
```

Use the unchanged scene that produced the snapshots. Pose and velocity rows must come from the same timestep and match `pose_rows.json`. The scene must use metres. Poses contain world actor-frame positions and xyzw orientations. Velocities contain world centre-of-mass linear velocity followed by world angular velocity.

The audit reads the joint body targets and local frames from USD. Static targets retain their authored world transforms; recorded dynamic targets use their saved poses. It measures:

- Forbidden anchor translation: all directions for fixed/revolute joints, perpendicular to the allowed axis for prismatic joints.
- Forbidden rotation: hinge-axis misalignment for revolute joints, full relative frame rotation for fixed/prismatic joints.
- Forbidden anchor velocity, computed from COM velocity and angular velocity at each anchor. Prismatic checks account for rotation of the moving joint axis.
- Forbidden relative angular velocity: perpendicular to the hinge axis for revolute joints, all directions for fixed/prismatic joints.

Default pose tolerances are 1 mm and 0.5 degrees. Set `--translation-tolerance-m` and `--rotation-tolerance-deg` to choose explicit diagnostic criteria. These values are not an established hardware-accuracy standard. Velocity residuals are measurements without an arbitrary pass threshold.

The JSON records each joint, labels, measurements, selected tolerances and input SHA256 hashes. Exit code 0 means all configured pose checks passed; exit code 1 means at least one failed. Invalid inputs also fail, so inspect the JSON or error message rather than interpreting exit code alone. The output cannot overwrite an input file.

A pose-check pass does not validate joint limits, drives, damping, contact forces, energy, live native attachment or trajectory equivalence. Sleeping bodies can still hold incorrect joint poses, and broad world-position bounds can pass while hinges violate their constraints. Use this report alongside the cooked-geometry audit, live scene inventory and native warnings.
