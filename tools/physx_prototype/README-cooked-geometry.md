`cooked_geometry.cpp` inspects the actual convex hulls held by a live PhysX scene. It is read-only and uses the public headers from the exact `ovphysx-0.5.11` tag. This checks a different boundary from the exporter's geometric audit: cooking may change an exactly exported hull.

Compile on the Linux benchmark host with the same compiler and headers as `scene_inspect.cpp`:

```sh
g++ -std=c++17 -O2 -DNDEBUG -fPIC -shared -I physx-include cooked_geometry.cpp -o cooked_geometry.so
```

No PhysX or CUDA libraries are linked. Obtain the live scene pointer through the already validated `ovphysx_get_physx_ptr` flow, then call after initialization or a synchronous step has completed, while the scene remains alive:

```python
import ctypes
import json
from pathlib import Path

library = ctypes.CDLL(str(Path("cooked_geometry.so").resolve()))
dump = library.eersim_cooked_convex_geometry
dump.argtypes = [ctypes.c_void_p]
dump.restype = ctypes.c_char_p
report = json.loads(dump(pointer))
if "error" in report:
    raise RuntimeError(report["error"])
Path("cooked-geometry.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
```

The returned string belongs to thread-local helper storage and is valid until the next call on that thread. `ctypes.c_char_p` copies it into Python bytes. A null pointer returns an error record; callers must supply a valid, matching-version scene pointer for inspection.

`meshes` contains each distinct `PxConvexMesh` once: vertices, polygon indices, polygon planes, vertex/polygon counts and GPU compatibility. `instances` maps those mesh IDs to actor and shape names, shape-local position/quaternion, native mesh scale, scale rotation and scale matrix, plus contact/rest offsets. Mesh IDs are scoped to one report. Null names remain null; actor/shape ordinals aid diagnosis but are not stable USD identifiers. Articulation links are enumerated as well as regular static/dynamic actors.

Vertices use the cooked mesh's own coordinate frame. Convert a column vertex `v` to actor-local coordinates using `R_local @ (S_mesh @ v) + p_local`. `mesh_scale_matrix_columns` contains the three columns of `S_mesh`; transpose the outer array when constructing an ordinary row-major NumPy matrix. Local quaternions are `(x,y,z,w)`. Compare against the authored collider after applying its USD transform into the same actor-local frame. The helper intentionally avoids reading global actor poses because DirectGPU host actor state can be stale.

Match `shape_name` to the exported `/World/.../collider_<collision>_<part>` prim and its referenced `geometry/*.usda` points. Check every mapping and shape count before interpreting discrepancies. Compare hull volume, bounds and bidirectional surface distance; the cooked polygon faces are included for this purpose. Deterministic sampled support directions are useful diagnostics, but their largest discrepancy is not a proven maximum over all directions. Keep mesh-surface error separate from contact/rest offsets, which affect contact behaviour without moving the stored vertices.

`audit_cooked_geometry.py` performs that comparison using NumPy, SciPy 1.16.3 and usd-core 25.11. It opens the composed USD stage, requires absolute actor/shape names that resolve unambiguously, and checks that every authored convex Mesh collider is covered. Analytic primitives such as cooked wheel Cylinders are listed separately. Missing names, unresolved paths, unexpected meshes and unmatched convex colliders fail coverage instead of invoking geometry guesses.

```sh
python audit_cooked_geometry.py --scene exported-v2/field.usda --dump field-gpu/cooked_geometry.json --output cooked-audit.json
```

The report separates authored geometry lost during cooking from cooked geometry extending outside the source. It computes convex-set Hausdorff distance from the maximum vertex distance to the other closed convex polyhedron, using analytic point-to-triangle distances in double precision. This is a per-collider leaf measurement; it does not compute the exterior error of a union of neighboring compound leaves. A 1e-12 m numerical inside band and 12-decimal coordinate cache keys are explicit; no CAD accuracy is inferred from them. Repeated equivalent leaves reuse comparisons. The report also includes volumes, bounds and sampled support diagnostics. `--max-error-m` optionally applies a caller-chosen acceptance threshold; without one, `passed` means complete successful measurement, not that the measured geometry is accurate enough.

The pinned source confirms a likely source of simplification:

- [`PxCookingParams::planeTolerance`](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/physx/include/cooking/PxCooking.h#L207) defaults to `0.0007`; points sufficiently close to the current hull can be dropped. Setting it to zero admits all points but can cause hull-merging failures.
- [`ConvexMeshCookingTask.cpp`](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/ovruntime/source/omni.physx.cooking/plugins/tasks/ConvexMeshCookingTask.cpp#L112) recomputes a convex hull and uses default cooking parameters. [`CookingTask.cpp`](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/ovruntime/source/omni.physx.cooking/plugins/service/CookingTask.cpp#L930) changes GPU-data/triangle-adjacency flags, leaving that tolerance intact.
- [`QuickHull`](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/physx/source/geomutils/src/cooking/GuCookingQuickHullConvexHullLib.cpp#L855) uses `max(planeTolerance * (dx + dy + dz) / 2, planeTolerance)`. For small vertices authored in metres, the default floor is therefore 0.7 mm. This is an algorithm threshold, not a verified Hausdorff error bound.
- The pinned [`PhysxConvexHullCollisionAPI`](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/schemas/physx/source/physxSchema/generatedSchema.usda#L462) exposes `hullVertexLimit` and `minThickness`; neither controls this threshold. The public `ConvexMeshCookingParams` and inspected cooking settings also expose no plane-tolerance override.

Native `PxCookingParams.planeTolerance` is the explicit control, but the inspected ovphysx USD cooking path does not surface it. A custom native cooking path or supported runtime extension would need to set and validate a tighter tolerance, then recheck GPU eligibility. Authoring an unrecognized USD attribute would not establish that cooking changed. First measure the present cooked hulls; fewer vertices alone neither establishes nor quantifies significant geometric error.

The retrieved A100 runs now provide measurements for two exports. Both audits resolved every one of the 4,432 authored convex Mesh collider instances by its USD actor/shape name. They compared 258 and 264 distinct geometry/transform combinations, respectively; neither report has missing mappings or uncovered authored convex meshes.

| Measured per-collider leaf result | v2: original coordinates | v3: points ×100, collider scale ÷100 |
| --- | --- | --- |
| Maximum pin-leaf hull distance | 699.096 µm inward | 20.4195 µm inward |
| Maximum cup-leaf hull distance | 697.458 µm inward | 0.003785 µm (about 3.8 nm) |
| Largest individual leaf volume loss | 12.1643% | 0.0118821% |
| Largest cooked outward excess | Within the 1e-12 m numerical band | 0.003571 µm (about 3.6 nm) |
| Requested 10 µm acceptance threshold | No threshold was requested for this measurement | **Not met** |

The v2 largest-distance pin leaf was `/World/rigid_100_base/collider_4_10` (`pin_cluster_3_east/pin`): 48 source hull vertices became nine cooked vertices and lost 10.3214% of that leaf's volume. The v3 largest-distance pin leaf was `/World/rigid_87_base/collider_1_3` (`pin_cluster_0_west/pin`): 40 source hull vertices became 35 and lost 0.00558739% volume. The v3 worst cup distance occurred at `/World/rigid_121_base/collider_63_0` (`cup_diagonal_3/cup`). Maximum distance and maximum volume loss occur on different leaves.

The reproducible v3 audit command includes the original target and therefore exits with failure while retaining complete measurements:

```sh
python audit_cooked_geometry.py --scene exported-v3/field.usda --dump gpu-v3/cooked_geometry.json --output gpu-v3/cooked-audit.json --max-error-m 0.00001
```

The JSON records `coverage_complete: true`, `within_requested_threshold: false` and `passed: false`. The v2 report is `gpu-v2/cooked-audit.json`. These are distances between authored USD convex leaves and live cooked leaves; the nanometre-scale cup numbers do not establish original CAD accuracy, material accuracy or physical contact accuracy.

Preconditioning works because this cooking path consumes raw mesh points and only applies the sign of mesh scaling. Positive scale magnitude remains in `PxConvexMeshGeometry.scale`: the retrieved v3 instances report approximately `0.01` on each axis. The exporter multiplies convex point coordinates by 100 and divides the collider's scale by 100, keeping actor poses, mass/inertia, joints and contact/rest offsets unchanged. Its separate composed-USD comparison found at most 9.537 nm change in world-space mesh coordinates. The [cooking task](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/ovruntime/source/omni.physx.cooking/plugins/tasks/ConvexMeshCookingTask.cpp#L83) and [USD loading path](https://github.com/NVIDIA-Omniverse/PhysX/blob/ovphysx-0.5.11/ovphysx/ovruntime/source/omni.physx/plugins/usdLoad/Collision.cpp#L633) explain that separation.

This reduces the fixed world-space tolerance floor from 0.7 mm to 7 µm. The shape-size-dependent term remains, so increasing the factor does not guarantee arbitrarily smaller error; the measured 20.4 µm pin result reflects why the 10 µm target cannot be claimed. v3 remains a bounded performance experiment. No packaged backend changed, and contact behaviour, compound-union exterior accuracy and native PROS operation still require independent validation.
