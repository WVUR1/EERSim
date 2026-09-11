"""Offline joint constraint audit of saved world poses and COM velocities.

Requires NumPy and usd-core; does not load ovphysx or initialize CUDA.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics


def rotation(quaternion):
    """Return a column-vector rotation matrix from an xyzw quaternion."""
    q = np.asarray(quaternion, dtype=float)
    length = np.linalg.norm(q)
    if not np.isfinite(q).all() or abs(length - 1) > 1e-4:
        raise ValueError("Expected a finite unit quaternion")
    x, y, z, w = q / length
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def usd_rotation(value):
    return rotation([*value.GetImaginary(), value.GetReal()])


def norm(vector):
    return float(np.linalg.norm(vector))


def angle_degrees(cosine):
    return math.degrees(math.acos(float(np.clip(cosine, -1, 1))))


def joint_metrics(kind, frame0, frame1, axis_index):
    """Frames contain world anchor, joint rotation, anchor velocity and omega."""
    p0, r0, v0, w0 = frame0
    p1, r1, v1, w1 = frame1
    delta, relative_omega = p1 - p0, w1 - w0
    axis = r0[:, axis_index]
    displacement = float(np.dot(delta, axis))
    if kind == "PhysicsPrismaticJoint":
        forbidden_translation = delta - axis * displacement
        # A translating joint's allowed axis rotates with body0. Differentiate
        # delta = displacement * axis before removing axial translation speed.
        relative_velocity = v1 - v0 - np.cross(w0, delta)
        forbidden_velocity = relative_velocity - axis * np.dot(relative_velocity, axis)
    else:
        forbidden_translation = delta
        forbidden_velocity = v1 - v0
    if kind == "PhysicsRevoluteJoint":
        forbidden_angle = angle_degrees(np.dot(axis, r1[:, axis_index]))
        forbidden_omega = relative_omega - axis * np.dot(relative_omega, axis)
    else:
        forbidden_angle = angle_degrees((np.trace(r0.T @ r1) - 1) / 2)
        forbidden_omega = relative_omega
    return {
        "anchor0_world_m": p0.tolist(), "anchor1_world_m": p1.tolist(),
        "anchor_separation_m": norm(delta),
        "forbidden_translation_m": norm(forbidden_translation),
        "forbidden_rotation_deg": forbidden_angle,
        "forbidden_anchor_velocity_m_s": norm(forbidden_velocity),
        "forbidden_angular_velocity_rad_s": norm(forbidden_omega),
        "axial_displacement_m": displacement,
    }


def audit(scene, pose_file, velocity_file, rows_file, translation_tolerance, rotation_tolerance):
    stage = Usd.Stage.Open(str(scene))
    if stage is None:
        raise ValueError("Could not open USD scene")
    if abs(UsdGeom.GetStageMetersPerUnit(stage) - 1) > 1e-12:
        raise ValueError("This benchmark audit requires metersPerUnit=1")
    poses = np.load(pose_file, allow_pickle=False)
    velocities = np.load(velocity_file, allow_pickle=False)
    rows = json.loads(rows_file.read_text(encoding="utf-8"))
    if poses.shape != (len(rows), 7) or velocities.shape != (len(rows), 6):
        raise ValueError("Pose/velocity shapes must match pose_rows.json")
    if not np.isfinite(poses).all() or not np.isfinite(velocities).all():
        raise ValueError("Nonfinite pose or velocity")
    by_path = {}
    for index, row in enumerate(rows):
        if row["row"] != index or row["path"] in by_path:
            raise ValueError("Rows must be unique and in recorded tensor order")
        if not stage.GetPrimAtPath(row["path"]):
            raise ValueError(f"Tensor prim missing from scene: {row['path']}")
        by_path[row["path"]] = index
    cache = UsdGeom.XformCache()
    body_cache = {}

    def body(path):
        if path in body_cache:
            return body_cache[path]
        if not path:
            return np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), "world"
        prim = stage.GetPrimAtPath(path)
        if not prim:
            raise ValueError(f"Joint target does not exist: {path}")
        if path in by_path:
            index = by_path[path]
            position, orient = poses[index, :3], rotation(poses[index, 3:])
            com_value = prim.GetAttribute("physics:centerOfMass").Get()
            if com_value is None or not np.isfinite(com_value).all():
                raise ValueError(f"Authored finite centerOfMass required: {path}")
            com = position + orient @ np.asarray(com_value, dtype=float)
            linear, angular = velocities[index, :3], velocities[index, 3:]
            label = rows[index].get("label", path)
        else:
            # An unrecorded dynamic target/ancestor must never silently become static.
            ancestor = prim
            while ancestor and not ancestor.IsPseudoRoot():
                if str(ancestor.GetPath()) in by_path or ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                    raise ValueError(f"Joint target needs its own dynamic tensor row: {path}")
                ancestor = ancestor.GetParent()
            matrix = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float)
            position, orient = matrix[3, :3], matrix[:3, :3].T
            if not np.allclose(orient.T @ orient, np.eye(3), atol=1e-6):
                raise ValueError(f"Scaled/sheared joint target unsupported: {path}")
            com, linear, angular, label = position, np.zeros(3), np.zeros(3), path
        result = position, orient, com, linear, angular, label
        body_cache[path] = result
        return result

    results = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        kind = prim.GetTypeName()
        if kind not in ("PhysicsFixedJoint", "PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
            raise ValueError(f"Unsupported joint type: {kind}")
        if prim.GetAttribute("physics:jointEnabled").Get() is False:
            continue
        frames, paths, labels = [], [], []
        for side in (0, 1):
            targets = prim.GetRelationship(f"physics:body{side}").GetTargets()
            if len(targets) > 1:
                raise ValueError(f"Multiple body{side} targets: {prim.GetPath()}")
            path = str(targets[0]) if targets else ""
            position, orient, com, linear, angular, label = body(path)
            local_position = np.asarray(prim.GetAttribute(f"physics:localPos{side}").Get(), dtype=float)
            local_rotation = usd_rotation(prim.GetAttribute(f"physics:localRot{side}").Get())
            anchor = position + orient @ local_position
            anchor_velocity = linear + np.cross(angular, anchor - com)
            frames.append((anchor, orient @ local_rotation, anchor_velocity, angular))
            paths.append(path)
            labels.append(label)
        axis = prim.GetAttribute("physics:axis").Get() or "X"
        values = joint_metrics(kind, *frames, axis_index="XYZ".index(str(axis)))
        violations = []
        if values["forbidden_translation_m"] > translation_tolerance:
            violations.append("forbidden_translation")
        if values["forbidden_rotation_deg"] > rotation_tolerance:
            violations.append("forbidden_rotation")
        results.append({"path": str(prim.GetPath()), "type": kind, "bodies": paths,
                        "labels": labels, "passed_configured_pose_checks": not violations,
                        "violations": violations, **values})
    if not results:
        raise ValueError("No supported enabled joints found")
    return {
        "schema_version": 1,
        "passed_configured_pose_checks": all(x["passed_configured_pose_checks"] for x in results),
        "joint_count": len(results),
        "failed_joint_count": sum(not x["passed_configured_pose_checks"] for x in results),
        "translation_tolerance_m": translation_tolerance,
        "rotation_tolerance_deg": rotation_tolerance,
        "max_forbidden_translation_m": max(x["forbidden_translation_m"] for x in results),
        "max_forbidden_rotation_deg": max(x["forbidden_rotation_deg"] for x in results),
        "max_forbidden_anchor_velocity_m_s": max(x["forbidden_anchor_velocity_m_s"] for x in results),
        "max_forbidden_angular_velocity_rad_s": max(x["forbidden_angular_velocity_rad_s"] for x in results),
        "notes": [
            "Offline audit; no simulation, CUDA initialization or source modification.",
            "Poses are world actor frames; linear velocities are world COM velocities, followed by world angular velocities.",
            "Unrecorded static targets use authored USD transforms; recorded dynamics use saved tensor poses.",
            "Thresholds are explicit diagnostic criteria, not established accuracy requirements or trajectory-equivalence proof.",
            "Velocity residuals are measurements only; no velocity pass threshold is applied.",
            "Joint limits, drives, damping, contacts, energy and live native joint attachment are not validated here.",
            "Snapshots must come from the same timestep and unchanged authored scene; hashes preserve input provenance.",
        ],
        "inputs": {name: {"filename": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                   for name, path in (("scene", scene), ("poses", pose_file),
                                      ("velocities", velocity_file), ("rows", rows_file))},
        "joints": results,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("scene", "poses", "velocities", "rows", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--translation-tolerance-m", type=float, default=0.001)
    parser.add_argument("--rotation-tolerance-deg", type=float, default=0.5)
    args = parser.parse_args()
    if not (math.isfinite(args.translation_tolerance_m) and args.translation_tolerance_m >= 0
            and math.isfinite(args.rotation_tolerance_deg) and args.rotation_tolerance_deg >= 0):
        parser.error("Tolerances must be finite and nonnegative")
    inputs = [getattr(args, key).resolve(strict=True) for key in ("scene", "poses", "velocities", "rows")]
    output = args.output.resolve()
    if output in inputs:
        parser.error("Output must not overwrite an input")
    result = audit(*inputs, args.translation_tolerance_m, args.rotation_tolerance_deg)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key not in ("joints", "inputs", "notes")}))
    return 0 if result["passed_configured_pose_checks"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
