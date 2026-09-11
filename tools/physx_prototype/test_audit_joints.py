"""Offline joint-audit checks; no physics runtime, simulation or CUDA."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from audit_joints import audit, joint_metrics, rotation


class JointAuditTests(unittest.TestCase):
    def setUp(self):
        self.zero = np.zeros(3)
        self.identity = np.eye(3)
        self.base = (self.zero, self.identity, self.zero, self.zero)

    def test_fixed_joint_coincident_frames_have_no_error(self):
        result = joint_metrics("PhysicsFixedJoint", self.base, self.base, 0)
        for metric in ("forbidden_translation_m", "forbidden_rotation_deg",
                       "forbidden_anchor_velocity_m_s", "forbidden_angular_velocity_rad_s"):
            self.assertEqual(result[metric], 0)

    def test_revolute_joint_allows_hinge_rotation_and_speed(self):
        orientation = rotation([np.sin(0.3), 0, 0, np.cos(0.3)])
        moving = (self.zero, orientation, self.zero, np.array([3., 0, 0]))
        result = joint_metrics("PhysicsRevoluteJoint", self.base, moving, 0)
        self.assertEqual(result["forbidden_rotation_deg"], 0)
        self.assertEqual(result["forbidden_angular_velocity_rad_s"], 0)
        # The identical orientation change is forbidden for a fixed joint.
        fixed = joint_metrics("PhysicsFixedJoint", self.base, moving, 0)
        self.assertAlmostEqual(fixed["forbidden_rotation_deg"], np.degrees(0.6))
        self.assertEqual(fixed["forbidden_angular_velocity_rad_s"], 3)

    def test_prismatic_velocity_accounts_for_rotation_of_allowed_axis(self):
        omega = np.array([0., 0, 2.])
        displacement = np.array([0.25, 0, 0])
        slide_velocity = np.array([1.5, 0, 0])
        parent = (self.zero, self.identity, self.zero, omega)
        child = (displacement, self.identity,
                 np.cross(omega, displacement) + slide_velocity, omega)
        result = joint_metrics("PhysicsPrismaticJoint", parent, child, 0)
        self.assertEqual(result["forbidden_translation_m"], 0)
        self.assertEqual(result["forbidden_anchor_velocity_m_s"], 0)
        self.assertEqual(result["forbidden_angular_velocity_rad_s"], 0)
        self.assertEqual(result["axial_displacement_m"], 0.25)

    def test_prismatic_joint_detects_transverse_motion(self):
        child = (np.array([0.25, 0.01, 0]), self.identity,
                 np.array([0., 2, 0]), self.zero)
        result = joint_metrics("PhysicsPrismaticJoint", self.base, child, 0)
        self.assertAlmostEqual(result["forbidden_translation_m"], 0.01)
        self.assertEqual(result["forbidden_anchor_velocity_m_s"], 2)

    def test_usd_joint_frames_use_saved_poses_and_world_com_velocities(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene = root / "scene.usda"
            stage = Usd.Stage.CreateNew(str(scene))
            UsdGeom.SetStageMetersPerUnit(stage, 1)
            UsdGeom.Xform.Define(stage, "/World")
            local_coms = ([0.25, 0, 0], [-0.1, 0, 0])
            paths = ["/World/parent", "/World/child"]
            for path, com in zip(paths, local_coms):
                body = UsdGeom.Xform.Define(stage, path)
                UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
                UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateCenterOfMassAttr(Gf.Vec3f(*com))
                # Authored initial transforms intentionally differ from saved poses.
                body.AddTranslateOp().Set(Gf.Vec3d(-10, -20, -30))
            joint = UsdPhysics.FixedJoint.Define(stage, "/World/fixed")
            joint.CreateBody0Rel().SetTargets([paths[0]])
            joint.CreateBody1Rel().SetTargets([paths[1]])
            joint.CreateLocalPos0Attr(Gf.Vec3f(1, 0, 0))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            local_rotation = Gf.Quatf(float(np.cos(np.pi / 8)), Gf.Vec3f(0, 0, float(np.sin(np.pi / 8))))
            joint.CreateLocalRot0Attr(local_rotation)
            joint.CreateLocalRot1Attr(local_rotation)
            stage.GetRootLayer().Save()

            q = np.array([0., 0, np.sqrt(0.5), np.sqrt(0.5)])
            positions = np.array([[1., 2, 3], [1., 3, 3]])
            poses = np.column_stack((positions, np.tile(q, (2, 1))))
            omega = np.array([0., 0, 2.])
            anchor = np.array([1., 3, 3.])
            anchor_velocity = np.array([1., 2, 3.])
            velocities = []
            for position, com in zip(positions, local_coms):
                world_com = position + rotation(q) @ com
                linear = anchor_velocity - np.cross(omega, anchor - world_com)
                velocities.append(np.concatenate((linear, omega)))
            velocities = np.array(velocities)
            pose_file, velocity_file, rows_file = (root / name for name in ("poses.npy", "velocities.npy", "rows.json"))
            np.save(pose_file, poses, allow_pickle=False)
            np.save(velocity_file, velocities, allow_pickle=False)
            rows_file.write_text(json.dumps([{"row": index, "path": path} for index, path in enumerate(paths)]), encoding="utf-8")
            result = audit(scene, pose_file, velocity_file, rows_file, 1e-6, 1e-4)
            self.assertTrue(result["passed_configured_pose_checks"])
            self.assertLess(result["max_forbidden_anchor_velocity_m_s"], 1e-7)
            np.testing.assert_allclose(result["joints"][0]["anchor0_world_m"], anchor, atol=1e-12)

            # Replacing COM velocity with actor-origin velocity corrupts the
            # hinge-speed check even though both saved poses remain valid.
            velocities[1, :3] = anchor_velocity
            np.save(velocity_file, velocities, allow_pickle=False)
            changed = audit(scene, pose_file, velocity_file, rows_file, 1e-6, 1e-4)
            self.assertTrue(changed["passed_configured_pose_checks"])
            self.assertAlmostEqual(changed["max_forbidden_anchor_velocity_m_s"], 0.2, places=7)


if __name__ == "__main__":
    unittest.main()
