"""Compare live cooked convex hulls with composed USD meshes; never modifies a scene.

Requires NumPy, scipy==1.16.3 and usd-core==25.11. No simulator or GPU is launched.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import ConvexHull
from pxr import Usd, UsdGeom, UsdPhysics


NUMERICAL_BAND_M = 1e-12


def quaternion_matrix(values):
    values = np.asarray(values, dtype=np.float64)
    if values.shape != (4,) or not np.isfinite(values).all():
        raise ValueError("Invalid local quaternion")
    if abs(float(values @ values) - 1) > 1e-5:
        raise ValueError("Local quaternion is not unit length")
    x, y, z, w = values / np.linalg.norm(values)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def point_array(value, name):
    result = np.asarray(value, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3 or len(result) < 4:
        raise ValueError(f"{name} must contain at least four XYZ vertices")
    if not np.isfinite(result).all():
        raise ValueError(f"{name} contains nonfinite coordinates")
    return result


def cooked_actor_points(mesh, instance):
    points = point_array(mesh["vertices"], "Cooked mesh")
    scale = np.asarray(instance["mesh_scale_matrix_columns"], dtype=np.float64).T
    position = np.asarray(instance["local_position"], dtype=np.float64)
    if scale.shape != (3, 3) or position.shape != (3,):
        raise ValueError("Invalid native mesh scale/local position")
    if not np.isfinite(scale).all() or not np.isfinite(position).all():
        raise ValueError("Nonfinite native instance transform")
    if abs(np.linalg.det(scale)) < 1e-18:
        raise ValueError("Singular native mesh scale")
    rotation = quaternion_matrix(instance["local_quaternion_xyzw"])
    return points @ scale.T @ rotation.T + position


def authored_actor_points(stage, cache, shape_name, actor_name):
    if not isinstance(shape_name, str) or not shape_name.startswith("/"):
        raise ValueError("Missing absolute USD shape name; geometry matching is not inferred")
    if not isinstance(actor_name, str) or not actor_name.startswith("/"):
        raise ValueError("Missing absolute USD actor name; actor frame is not inferred")
    shape = stage.GetPrimAtPath(shape_name)
    actor = stage.GetPrimAtPath(actor_name)
    if not shape or not actor:
        raise ValueError("Named shape or actor does not resolve in the composed USD stage")
    if not shape.GetPath().HasPrefix(actor.GetPath()):
        raise ValueError("Named actor is not an ancestor of the named collider")
    if not shape.IsA(UsdGeom.Mesh):
        if shape.GetTypeName() not in {"Cylinder", "Sphere", "Cube", "Capsule", "Cone"}:
            raise ValueError(f"Named cooked collider is neither a mesh nor a supported analytic primitive: {shape.GetTypeName()}")
        return None, shape.GetTypeName()
    points = point_array(UsdGeom.Mesh(shape).GetPointsAttr().Get(), "Authored USD mesh")
    # Gf matrices multiply row vectors; shape-to-world then world-to-actor.
    matrix = cache.GetLocalToWorldTransform(shape) * cache.GetLocalToWorldTransform(actor).GetInverse()
    values = np.asarray(matrix, dtype=np.float64)
    transformed = np.column_stack((points, np.ones(len(points)))) @ values
    if not np.isfinite(transformed).all() or np.any(abs(transformed[:, 3]) < 1e-15):
        raise ValueError("Invalid authored collider-to-actor transform")
    return transformed[:, :3] / transformed[:, 3, None], "Mesh"


def directed_hull_distance(points, target_points, target_hull):
    """Convex-set directed distance: max vertex distance to the other closed hull.

    Outside-point distance is the minimum analytic distance to its triangulated
    boundary. The maximum of distance-to-a-convex-set occurs at a source vertex.
    Computation uses double precision; the stated roundoff band is not CAD tolerance.
    """
    violations = points @ target_hull.equations[:, :3].T + target_hull.equations[:, 3]
    outside = np.max(violations, axis=1) > NUMERICAL_BAND_M
    distances = np.zeros(len(points))
    triangles = target_points[target_hull.simplices]
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac = b - a, c - a
    normals = np.cross(ab, ac)
    norm2 = np.einsum("ij,ij->i", normals, normals)
    valid = norm2 > np.finfo(float).tiny
    if not valid.all():
        raise ValueError("Degenerate triangle in computed convex hull")
    d00 = np.einsum("ij,ij->i", ab, ab)
    d01 = np.einsum("ij,ij->i", ab, ac)
    d11 = np.einsum("ij,ij->i", ac, ac)
    denominator = d00*d11 - d01*d01
    if np.any(denominator <= 0):
        raise ValueError("Numerically degenerate convex-hull face")
    for index in np.flatnonzero(outside):
        point = points[index]
        ap = point - a
        plane_numerator = np.einsum("ij,ij->i", ap, normals)
        projection = point - normals * (plane_numerator / norm2)[:, None]
        projected_a = projection - a
        d20 = np.einsum("ij,ij->i", projected_a, ab)
        d21 = np.einsum("ij,ij->i", projected_a, ac)
        beta = (d11*d20 - d01*d21) / denominator
        gamma = (d00*d21 - d01*d20) / denominator
        inside = (beta >= 0) & (gamma >= 0) & (beta + gamma <= 1)
        squared = np.where(inside, plane_numerator**2 / norm2, np.inf)
        for start, end in ((a, b), (b, c), (c, a)):
            edge = end - start
            length2 = np.einsum("ij,ij->i", edge, edge)
            fraction = np.clip(np.einsum("ij,ij->i", point - start, edge) / length2, 0, 1)
            delta = point - (start + fraction[:, None] * edge)
            squared = np.minimum(squared, np.einsum("ij,ij->i", delta, delta))
        distances[index] = math.sqrt(float(squared.min()))
    return float(distances.max()), float(max(0, violations.max()))


def compare_hulls(authored, cooked):
    authored_hull, cooked_hull = ConvexHull(authored), ConvexHull(cooked)
    authored_vertices = authored[authored_hull.vertices]
    cooked_vertices = cooked[cooked_hull.vertices]
    inward, authored_outside = directed_hull_distance(authored_vertices, cooked, cooked_hull)
    outward, cooked_outside = directed_hull_distance(cooked_vertices, authored, authored_hull)
    random = np.random.default_rng(7).normal(size=(512, 3))
    random /= np.linalg.norm(random, axis=1)[:, None]
    directions = np.vstack((np.eye(3), -np.eye(3), random,
                            authored_hull.equations[:, :3], cooked_hull.equations[:, :3]))
    differences = (authored_vertices @ directions.T).max(axis=0) - (cooked_vertices @ directions.T).max(axis=0)
    return {
        "authored_point_count": len(authored), "authored_hull_vertex_count": len(authored_vertices),
        "cooked_vertex_count": len(cooked),
        "authored_volume_m3": float(authored_hull.volume), "cooked_volume_m3": float(cooked_hull.volume),
        "relative_volume_change": float(cooked_hull.volume / authored_hull.volume - 1),
        "authored_bounds_m": [authored.min(axis=0).tolist(), authored.max(axis=0).tolist()],
        "cooked_bounds_m": [cooked.min(axis=0).tolist(), cooked.max(axis=0).tolist()],
        "max_authored_outside_cooked_m": inward,
        "max_cooked_outside_authored_m": outward,
        "convex_hausdorff_distance_m": max(inward, outward),
        "max_authored_violation_of_cooked_planes_m": authored_outside,
        "max_cooked_violation_of_authored_planes_m": cooked_outside,
        "sampled_support_direction_count": len(directions),
        "max_sampled_authored_minus_cooked_support_m": float(max(0, differences.max())),
        "max_sampled_cooked_minus_authored_support_m": float(max(0, -differences.min())),
    }


def audit(scene, dump_path, maximum_error=None):
    raw_dump = dump_path.read_bytes()
    dump = json.loads(raw_dump)
    if "error" in dump:
        raise ValueError(f"Native geometry inspection failed: {dump['error']}")
    if dump.get("schema_version") != 1:
        raise ValueError("Unsupported cooked-geometry schema")
    stage = Usd.Stage.Open(str(scene.resolve()))
    if not stage:
        raise ValueError("Cannot open composed USD stage")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1:
        raise ValueError("Audit requires metresPerUnit=1; other stage units need explicit conversion")
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    meshes = {entry["id"]: entry for entry in dump["meshes"]}
    if len(meshes) != len(dump["meshes"]):
        raise ValueError("Duplicate cooked mesh IDs")
    comparisons, memo, errors, non_mesh, matched_names = [], {}, [], [], set()
    for index, instance in enumerate(dump["instances"]):
        name, actor = instance.get("shape_name"), instance.get("actor_name")
        identity = {"instance_index": index, "shape_name": name, "actor_name": actor,
                    "mesh_id": instance.get("mesh_id")}
        try:
            expected, kind = authored_actor_points(stage, cache, name, actor)
            if expected is None:
                non_mesh.append({**identity, "usd_type": kind,
                                 "reason": "Analytic primitive cooking is outside this mesh audit"})
                continue
            if name in matched_names:
                raise ValueError("Multiple cooked instances share one authored mesh prim; union audit required")
            actual = cooked_actor_points(meshes[instance["mesh_id"]], instance)
            # Dedupe equivalent repeated leaves, including harmless transform roundoff.
            # Both point arrays enter the key; distinct scale/pose instances are not reused.
            key = (expected.shape, actual.shape, hashlib.sha256(
                np.round(expected, 12).tobytes() + np.round(actual, 12).tobytes()).hexdigest())
            if key not in memo:
                result = compare_hulls(expected, actual)
                result.update({"comparison_id": len(comparisons), "instances": []})
                memo[key] = result
                comparisons.append(result)
            memo[key]["instances"].append(identity)
            matched_names.add(name)
        except Exception as error:
            errors.append({**identity, "error": f"{type(error).__name__}: {error}"})
    expected_names = set()
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            continue
        if UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get() == "convexHull":
            expected_names.add(str(prim.GetPath()))
    missing = sorted(expected_names - matched_names)
    unexpected = sorted(matched_names - expected_names)
    largest = max((item["convex_hausdorff_distance_m"] for item in comparisons), default=None)
    complete = bool(comparisons) and not errors and not missing and not unexpected
    within = None if maximum_error is None or largest is None else largest <= maximum_error
    comparisons.sort(key=lambda entry: entry["convex_hausdorff_distance_m"], reverse=True)
    return {
        "schema_version": 1, "passed": complete and within is not False,
        "coverage_complete": complete, "maximum_error_threshold_m": maximum_error,
        "within_requested_threshold": within,
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "dump_sha256": hashlib.sha256(raw_dump).hexdigest(),
        "scene_meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "units_note": "Distances are authored stage units; this exporter requires metresPerUnit=1",
        "numerical_inside_band_m": NUMERICAL_BAND_M,
        "deduplication_coordinate_rounding_decimals": 12,
        "distance_method": "Bidirectional maximum source-vertex distance to target closed convex polyhedron; analytic point-to-triangle distance in double precision",
        "sampled_support_note": "Support samples are diagnostics and are not a maximum-error proof",
        "scope": "Per-collider convex leaf geometry; does not compute the union of neighboring compound leaves. Excludes contact/rest offsets, triangle meshes, analytic primitives and full-contact simulation",
        "native_convex_instance_count": len(dump["instances"]),
        "expected_convex_mesh_prim_count": len(expected_names),
        "matched_convex_mesh_instances": len(matched_names),
        "unique_comparisons": len(comparisons),
        "max_convex_hausdorff_distance_m": largest,
        "missing_authored_convex_meshes": missing,
        "unexpected_mesh_instances": unexpected,
        "unassessed_analytic_instances": non_mesh, "errors": errors, "comparisons": comparisons,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--dump", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-error-m", type=float, help="Optional explicit acceptance threshold; no default accuracy claim")
    args = parser.parse_args()
    if args.max_error_m is not None and (not math.isfinite(args.max_error_m) or args.max_error_m < 0):
        parser.error("max-error-m must be finite and nonnegative")
    scene = args.scene.resolve(strict=True)
    dump = args.dump.resolve(strict=True)
    output = args.output.resolve()
    if output in (scene, dump):
        parser.error("Output must not overwrite the scene or cooked-geometry input")
    result = audit(scene, dump, args.max_error_m)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: result[key] for key in (
        "passed", "coverage_complete", "matched_convex_mesh_instances", "unique_comparisons",
        "max_convex_hausdorff_distance_m", "within_requested_threshold")}, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
