import itertools
import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from audit_cooked_geometry import audit, compare_hulls, main


def cube():
    return np.array(list(itertools.product((-0.5, 0.5), repeat=3)), dtype=float)


class CookedGeometryAuditTests(unittest.TestCase):
    def test_identical_reordered_hulls(self):
        result = compare_hulls(cube(), cube()[::-1])
        self.assertLess(result["convex_hausdorff_distance_m"], 1e-12)
        self.assertAlmostEqual(result["relative_volume_change"], 0)

    def test_corner_distance_exceeds_plane_violation(self):
        # A diagonal translation has true corner distance sqrt(3)*d, although
        # each axis-aligned supporting-plane violation is only d.
        result = compare_hulls(cube(), cube() + 0.1)
        self.assertAlmostEqual(result["max_authored_outside_cooked_m"], 3**0.5 * 0.1)
        self.assertAlmostEqual(result["max_cooked_outside_authored_m"], 3**0.5 * 0.1)
        self.assertAlmostEqual(result["max_authored_violation_of_cooked_planes_m"], 0.1)

    def test_shrinkage_direction_and_volume(self):
        result = compare_hulls(cube(), cube() * 0.8)
        self.assertAlmostEqual(result["max_cooked_outside_authored_m"], 0)
        self.assertAlmostEqual(result["max_authored_outside_cooked_m"], 3**0.5 * 0.1)
        self.assertAlmostEqual(result["relative_volume_change"], 0.8**3 - 1)

    def fixture(self, root):
        scene = root / "scene.usda"
        stage = Usd.Stage.CreateNew(str(scene))
        UsdGeom.SetStageMetersPerUnit(stage, 1)
        world = UsdGeom.Xform.Define(stage, "/World")
        actor = UsdGeom.Xform.Define(stage, "/World/Robot")
        UsdPhysics.RigidBodyAPI.Apply(actor.GetPrim())
        actor.AddTranslateOp().Set(Gf.Vec3d(7, 4, 2))
        actor.AddRotateZOp().Set(37)
        mesh = UsdGeom.Mesh.Define(stage, "/World/Robot/collider")
        mesh.CreatePointsAttr([Gf.Vec3f(*row) for row in cube()])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        mesh.AddTranslateOp().Set(Gf.Vec3d(0.13, -0.07, 0.2))
        mesh.AddRotateYOp().Set(90)
        mesh.AddScaleOp().Set(Gf.Vec3d(2, 0.5, 1.5))
        stage.GetRootLayer().Save()
        native = {
            "schema_version": 1,
            "meshes": [{"id": 0, "vertices": cube().tolist()}],
            "instances": [{"actor_name": "/World/Robot", "shape_name": "/World/Robot/collider",
                           "mesh_id": 0, "local_position": [0.13, -0.07, 0.2],
                           "local_quaternion_xyzw": [0, 2**-0.5, 0, 2**-0.5],
                           "mesh_scale_matrix_columns": np.diag([2, 0.5, 1.5]).tolist()}],
        }
        dump = root / "dump.json"
        dump.write_text(json.dumps(native), encoding="utf-8")
        return scene, dump, native

    def test_real_usd_actor_frame_rotation_and_mesh_scale(self):
        with tempfile.TemporaryDirectory() as temporary:
            scene, dump, _ = self.fixture(Path(temporary))
            result = audit(scene, dump, maximum_error=1e-9)
            self.assertTrue(result["passed"], result["errors"])
            self.assertEqual(result["matched_convex_mesh_instances"], 1)
            self.assertLess(result["max_convex_hausdorff_distance_m"], 1e-12)

    def test_null_shape_name_fails_without_geometry_guess(self):
        with tempfile.TemporaryDirectory() as temporary:
            scene, dump, native = self.fixture(Path(temporary))
            native["instances"][0]["shape_name"] = None
            dump.write_text(json.dumps(native), encoding="utf-8")
            result = audit(scene, dump)
            self.assertFalse(result["passed"])
            self.assertEqual(result["missing_authored_convex_meshes"], ["/World/Robot/collider"])
            self.assertIn("not inferred", result["errors"][0]["error"])

    def test_explicit_geometry_threshold_detects_changed_cooking(self):
        with tempfile.TemporaryDirectory() as temporary:
            scene, dump, native = self.fixture(Path(temporary))
            native["meshes"][0]["vertices"] = (cube() * 0.9).tolist()
            dump.write_text(json.dumps(native), encoding="utf-8")
            result = audit(scene, dump, maximum_error=0.001)
            self.assertTrue(result["coverage_complete"])
            self.assertFalse(result["passed"])
            self.assertGreater(result["max_convex_hausdorff_distance_m"], 0.1)

    def test_output_alias_cannot_overwrite_either_input(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            scene, dump, _ = self.fixture(root)
            originals = {path: path.read_bytes() for path in (scene, dump)}
            (root / "output").mkdir()
            for target in (scene, dump):
                with self.subTest(target=target.name):
                    alias = root / "output" / ".." / target.name
                    argv = ["audit_cooked_geometry.py", "--scene", str(scene),
                            "--dump", str(dump), "--output", str(alias)]
                    with patch("sys.argv", argv), patch("audit_cooked_geometry.audit") as analyze:
                        with contextlib.redirect_stderr(io.StringIO()) as stderr:
                            with self.assertRaises(SystemExit) as raised:
                                main()
                        self.assertEqual(raised.exception.code, 2)
                        self.assertIn("must not overwrite", stderr.getvalue())
                        analyze.assert_not_called()
                    for path, original in originals.items():
                        self.assertEqual(path.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
