"""Provenance and assembly-frame invariants for the derived official CAD."""
from __future__ import annotations

from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SIMULATOR = Path(__file__).resolve().parents[1]
ASSETS = SIMULATOR / "assets"
spec = importlib.util.spec_from_file_location("vex_cad_import", SIMULATOR / "tools" / "import_vex_cad.py")
cad = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cad)


class OfficialCadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads((ASSETS / "cad_manifest.json").read_text(encoding="utf-8"))

    def test_asset_checksums_match_provenance(self):
        source = self.manifest["source"]
        self.assertEqual(source["publisher"], "VEX Robotics")
        self.assertEqual(len(source["step_sha256"]), 64)
        self.assertEqual(len(source["archive_sha256"]), 64)
        for key, asset in self.manifest["assets"].items():
            with self.subTest(asset=key):
                path = ASSETS / asset["mesh"]
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), asset["mesh_sha256"])
                self.assertTrue(path.with_suffix(".mtl").is_file())

    def test_original_assembly_counts_and_normalized_placement_roundtrip(self):
        field = self.manifest["field_placements"]
        count = Counter(row["kind"] for row in field["instances"] if row["in_field"])
        self.assertEqual(count, {"pin": 37, "cup": 36, "goal_alliance": 4,
                                 "goal_neutral": 4, "goal_center": 1,
                                 "toggle": 4, "loader": 4,
                                 "white_panel": 8, "white_panel_short": 4})
        for row in field["instances"]:
            asset = self.manifest["assets"][row.get("asset_key", row["kind"])]
            offset = [value * .001 for value in asset["normalization_offset_oriented_mm"]]
            # Mesh coordinates of the original CAD assembly origin are -offset.
            reprojected = cad.matvec(row["rotation_matrix"], [-v for v in offset])
            reprojected = [a + b for a, b in zip(reprojected, row["position"])]
            for actual, expected in zip(reprojected, row["assembly_origin_m"]):
                self.assertAlmostEqual(actual, expected, places=12)

    def test_solid_halves_and_toggle_partition_retain_all_triangles(self):
        assets = self.manifest["assets"]
        for key in ("pin", "cup"):
            self.assertEqual(assets[key]["triangles"],
                             assets[key + "_upper"]["triangles"] + assets[key + "_lower"]["triangles"])
            for half in ("upper", "lower"):
                self.assertEqual(assets[key]["normalization_offset_oriented_mm"],
                                 assets[key + "_" + half]["normalization_offset_oriented_mm"])
        self.assertEqual(assets["toggle"]["triangles"],
                         assets["toggle_fixed"]["triangles"] + assets["toggle_rotor"]["triangles"])
        mechanism = self.manifest["mechanisms"]["toggle"]
        self.assertAlmostEqual(sum(value * value for value in mechanism["axis"]), 1.0)
        for pivot, origin in zip(mechanism["pivot_m"], mechanism["visual_origin_from_joint_m"]):
            self.assertAlmostEqual(pivot, -origin)

    def test_real_pin_flange_and_socket_units(self):
        assets = self.manifest["assets"]
        self.assertAlmostEqual(assets["pin"]["size_m"][0], .080293018, places=7)
        self.assertAlmostEqual(assets["pin"]["size_m"][2], .165, places=7)
        self.assertAlmostEqual(assets["cup"]["size_m"][2], .1645, places=7)
        self.assertAlmostEqual(assets["goal_socket"]["size_m"][2], .08253265, places=7)
        profile = self.manifest["measured_profiles"]["goal_socket"]["sections"]
        self.assertGreater(profile[-1]["minimum_surface_radius_m"],
                           2 * profile[0]["minimum_surface_radius_m"])

    def test_cross_section_reports_segment_radius_minimum(self):
        # A vertical triangle cuts z=0 along x=-1..1,y=1. Its closest
        # point to the axis is inside the segment, not one of its endpoints.
        with tempfile.TemporaryDirectory() as temporary:
            mesh = Path(temporary) / "slice.obj"
            mesh.write_text("v -2 1 -1\nv 2 1 -1\nv 0 1 1\nf 1 2 3\n")
            section = cad.mesh_sections(mesh, [0])[0]
            self.assertAlmostEqual(section["minimum_surface_radius_m"], 1)
            self.assertAlmostEqual(section["maximum_surface_radius_m"], 2 ** .5)


if __name__ == "__main__":
    unittest.main()
