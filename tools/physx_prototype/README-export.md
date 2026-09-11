This is an isolated physics-only field exporter for the exact `ovphysx==0.5.11` runtime. It makes no changes to EERSim's published source and performs no simulation steps or rendering. The generated scene is for contact-throughput research; it is not a replacement autonomous backend.

Use an activated EERSim Python environment and install the offline tools' pinned dependencies with `python -m pip install scipy==1.16.3 usd-core==25.11`. SciPy performs convex-hull analysis; usd-core provides independent USD parser validation. The scripts import dependencies from that environment and take the checkout/output locations as arguments. The compute host uses the separately pinned runtime environment described in `README-runner.md`.

Run validation with ordinary Python, without `-O`, `-OO` or `PYTHONOPTIMIZE`. The validator explicitly rejects optimized execution because Python optimization disables its assertion checks.

```text
python export_field.py --repository <EERSim-checkout> --output exported --convex-policy split-exact
python validate_export.py exported
```

The explicit precision experiment uses a larger raw coordinate space only for convex cooking:

```text
python export_field.py --repository <EERSim-checkout> --output exported-v4 --convex-policy split-exact --convex-cook-scale 100
python validate_export.py exported-v4
```

The default cooking scale remains 1. At scale 100, raw convex points are multiplied by 100 and each collider receives a compensating positive scale of 0.01. Actors, inertia, material coefficients, contact offsets, world units, triangle geometry and primitive dimensions remain unchanged. Pinned source inspection confirms the cooker retains the positive magnitude as `PxMeshScale`, applying only reflection signs to its raw input points. The actual cooked dump must confirm that behavior and the resulting precision. Validation of the first v3 scene checked all 4,449 mesh instances against v2 with a maximum world-coordinate change of 9.54 nanometres from float32 rounding.

This preconditioning reduces the cooker's absolute plane-tolerance floor from 0.7 mm to 7 micrometres in world units. It does not reduce its relative threshold term, which depends on half the sum of the raw hull's axis-aligned extents. Recentring vertices does not change those extents. No sub-10-micrometre cooked-geometry claim is justified until the runtime audit passes.

Copy the complete chosen output directory, `benchmark.py`, and the runner's required helper artifacts to a working directory on the compute host. Keep the output directory's relative `geometry/` references intact, and install the pinned runtime dependencies on that host instead of copying a local Python environment. See `README-runner.md` for the bounded CPU/GPU comparison command and live GPU verification.

The export loads the real claw-demo `World` once, records its initial body/link poses, masses, diagonal inertias, principal-axis frames, friction and restitution, and reads the actual URDF collision geometries. It retains the floor, perimeter, all 36 cups, 37 field pins, demo preload, goals, loaders, toggle hinges and passive robot links/joints. Display meshes are omitted. Static sockets remain triangle meshes and cup openings stay hollow.

The corrected `exported-v4/` scene retains Bullet's intra-URDF collision exclusions: the current World does not set `URDF_USE_SELF_COLLISION`, so all link pairs within each imported robot/toggle body are filtered. There are 49 such pairs. Treating the robot as independent PhysX actors without these exclusions would introduce collisions between overlapping nonadjacent mast/carriage components. Static-parent exclusions must be authored on the dynamic paddle, targeting its static parent's subtree: the pinned filter loader requires its first endpoint to resolve to an actual body/shape and descends children only for its second endpoint. Earlier v2/v3 variants placed those four exclusions on a plain static Xform, so they were ignored; they are historical diagnostic artifacts. Validation now checks the native-resolvable author, preserves all unordered pairs, and checks each joint's world-space anchor/axis while allowing the authored prismatic displacement along its permitted axis.

The v4/v3 audit can be reproduced with `python validate_export.py exported-v4 --equivalent-to exported-v3 --allow-filter-authorship-changes`. It found exactly four reversed filter directions and zero world-coordinate change across all 4,449 mesh instances; actor, mass, inertia, material, contact-offset and joint settings were unchanged. Actual constraint stability still requires the subsequent simulation audit.

`field.manifest.json` records input file hashes, generated mesh hashes, actor properties, collider counts, geometry audits, solver settings and unmapped features. `field.usda` references only adjacent `geometry/*.usda` files, with metres, kilograms and Z-up coordinates. The reference timestep is exactly 1/240 second. The scene explicitly requests PhysX GPU dynamics, GPU broadphase, TGS, 100 position iterations and one velocity iteration. These are reported settings, not a claim of numerical equivalence with Bullet's 100-iteration sequential-impulse solver.

The original pin convex leaves exceed PhysX's GPU limits; some also exceed its hard 255-vertex cooking limit. The default `reject` policy stops instead of silently simplifying them. `split-exact` recursively intersects each original convex polyhedron with an axial plane until each resulting piece has at most 64 vertices, 64 polygon planes and 32 vertices on a polygon. The split halfspaces share only their boundary, so their union retains the original hull. The 64 original cup components remain unchanged; each pin becomes 56 convex components. New internal seams can affect solver behavior and are explicitly a decomposition difference.

Each partition checks summed volume against the original, every output vertex against the original supporting halfspaces, and exterior support distances along 518 deterministic directions. Double-precision volume error must be below 1e-9 relative (or 1e-16 cubic metres absolute), and support/halfspace errors below 1e-9 metres. The authored `point3f` coordinates have a separate maximum quantization tolerance of 1e-7 metres. Every convex collider explicitly sets `hullVertexLimit=64` and `minThickness=0`, avoiding the schema's default 1 mm thickness inflation. Runtime cooking eligibility still needs verification through the runner's live PhysX inspection.

These exactness checks describe the authored partition, not the final cooked PhysX hull. The pinned PhysX cooker has a separate default plane tolerance that can remove vertices even below all size limits; GPU eligibility does not establish geometric fidelity. The `cooked_geometry.cpp` inspector is needed to compare actual cooked support distances against these source partitions before making accuracy claims. CPU/GPU throughput comparisons must use the same cooking settings and geometry.

Material static/dynamic friction numbers and restitution are copied from the loaded Bullet links. Both material combine modes are `multiply`, matching Bullet's pairwise coefficient products. Convex mesh rest offsets use the loaded Bullet collision margin, and contact offsets add 2 mm. This is a disclosed PhysX contact-model mapping, not an exact reproduction of Bullet margin/contact-cache semantics. Box and analytic primitive outer dimensions remain authored dimensions, with zero rest offset.

The comparison omits PROS execution, the voltage/torque motor model, external omni-wheel tire forces, and EERSim's numerical-rest helper. Robot and toggle constraints are passive regular PhysX joints, rather than Bullet reduced-coordinate articulations. Bullet rolling/spinning friction is recorded but has no direct USD material equivalent here. Damping coefficients are copied from EERSim's setup, but the integration rules differ. Native PhysX sleeping is used. Wheel Cylinder primitives may be cooked into convex geometry; the live shape report must identify their resulting type. GPU dynamics alone does not prove every contact is generated on the GPU.

Pinned primary references:

- [PhysX 0.5.11 collision authoring](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/docs/simulation_setup/collision.md)
- [Material coefficients and combine modes](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/docs/simulation_setup/rigid_bodies.md)
- [Convex cooking hard limits](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/physx/include/cooking/PxConvexMeshDesc.h)
- [GPU compatibility checks](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/physx/include/geometry/PxConvexMesh.h)
- [Schema cooking and solver defaults](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/schemas/physx/source/physxSchema/generatedSchema.usda)

The input field geometry is derived from official VEX CAD and retains its existing source attribution. No new licensing claim is made by this temporary conversion.
