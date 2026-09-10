"""URDF/field authoring validation and import relocation, using only the stdlib."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from eersim.importers import (
    create_robot_config, import_field, inspect_robot_urdf,
    load_field_import, materialize_field_import,
)


URDF = """<?xml version="1.0"?>
<robot name="test_bot">
  <link name="base">
    <collision><geometry><box size=".3 .3 .1"/></geometry></collision>
  </link>
  <link name="wheel">
    <visual><geometry><mesh filename="meshes/wheel.obj"/></geometry></visual>
  </link>
  <joint name="wheel_spin" type="continuous">
    <parent link="base"/><child link="wheel"/><axis xyz="0 1 0"/>
    <limit effort="2.1" velocity="20"/>
  </joint>
</robot>
"""


class ImporterTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="eersim-import-")
        self.root = Path(self.temp.name)
        (self.root / "meshes").mkdir()
        (self.root / "meshes" / "wheel.obj").write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
        self.urdf = self.root / "robot.urdf"
        self.urdf.write_text(URDF)

    def tearDown(self):
        self.temp.cleanup()

    def write_field(self, data):
        path = self.root / "field.json"
        path.write_text(json.dumps(data))
        return path

    def test_standard_urdf_inspection_and_unassigned_motor_patch(self):
        info = inspect_robot_urdf(self.urdf)
        self.assertEqual(info["name"], "test_bot")
        self.assertEqual(info["root_link"], "base")
        self.assertEqual(info["joints"], [{"name": "wheel_spin", "type": "continuous",
                         "lower": None, "upper": None, "effort": 2.1, "velocity": 20.0}])
        self.assertTrue(any("mass/inertia" in warning for warning in info["warnings"]))
        config = create_robot_config(self.urdf)
        self.assertEqual(config["robot_urdf"], str(self.urdf.resolve()))
        self.assertEqual(config["robot_config"], "")
        self.assertEqual(config["motors"], [])
        self.assertEqual(self.urdf.read_text(), URDF)

    def test_sidecar_preserves_multiple_signed_motors_on_one_joint(self):
        motors = [{"port": 1, "joint": "wheel_spin", "rpm": 600},
                  {"port": -2, "joint": "wheel_spin", "rpm": 600}]
        sidecar = self.urdf.with_suffix(".json")
        sidecar.write_text(json.dumps({"schema_version": 1, "urdf": self.urdf.name, "motors": motors}))
        config = create_robot_config(self.urdf)
        self.assertEqual(config["motors"], motors)
        self.assertEqual(config["robot_config"], str(sidecar))
        motors[1]["port"] = -1
        sidecar.write_text(json.dumps({"motors": motors}))
        with self.assertRaisesRegex(ValueError, "duplicate motor port"):
            create_robot_config(self.urdf)

    def test_unrelated_adjacent_json_is_not_used_as_robot_bindings(self):
        self.urdf.with_suffix(".json").write_text(json.dumps({"field": {}, "objects": []}))
        self.assertEqual(create_robot_config(self.urdf)["robot_config"], "")
        self.urdf.with_suffix(".json").write_text("{invalid")
        info = inspect_robot_urdf(self.urdf)
        self.assertTrue(any("could not be read" in warning for warning in info["warnings"]))
        self.assertEqual(create_robot_config(self.urdf)["motors"], [])

    def test_missing_mesh_and_unresolved_package_have_actionable_errors(self):
        (self.root / "meshes" / "wheel.obj").unlink()
        with self.assertRaisesRegex(ValueError, "mesh file is missing"):
            inspect_robot_urdf(self.urdf)
        self.urdf.write_text(URDF.replace("meshes/wheel.obj", "package://test/meshes/wheel.obj"))
        with self.assertRaisesRegex(ValueError, "unresolved ROS package URI"):
            inspect_robot_urdf(self.urdf)

    def test_local_package_uri_is_normalized_without_editing_source(self):
        package = self.root / "robot_description"
        (package / "urdf").mkdir(parents=True)
        (package / "meshes").mkdir()
        mesh = package / "meshes" / "wheel.obj"
        mesh.write_text((self.root / "meshes" / "wheel.obj").read_text())
        robot = package / "urdf" / "export.urdf"
        original = URDF.replace("meshes/wheel.obj", "package://robot_description/meshes/wheel.obj")
        robot.write_text(original)
        # Avoid an intentionally ambiguous same-relative-path test fixture.
        (self.root / "meshes" / "wheel.obj").unlink()
        robot.with_suffix(".json").write_text(json.dumps(
            {"urdf": robot.name, "motors": [{"port": 2, "joint": "wheel_spin"}]}))
        with patch("eersim.importers.Path.cwd", return_value=self.root):
            info = inspect_robot_urdf(robot)
            self.assertTrue(any("package assets resolved" in warning for warning in info["warnings"]))
            config = create_robot_config(robot)
            prepared = Path(config["robot_urdf"])
            self.assertNotEqual(prepared, robot)
            self.assertTrue(prepared.is_relative_to(self.root / ".eersim" / "imports"))
            mesh_reference = ET.parse(prepared).find(".//mesh").get("filename")
            self.assertFalse(Path(mesh_reference).is_absolute())
            self.assertEqual((prepared.parent / mesh_reference).resolve(), mesh)
            self.assertEqual(config["robot_config"], str(robot.with_suffix(".json")))
            self.assertEqual(config["motors"][0]["port"], 2)
            field = import_field(robot, self.root / "field-imports")
            wrapper_path = Path(field["path"])
            wrapper = json.loads(wrapper_path.read_text())
            self.assertFalse(Path(wrapper["objects"][0]["urdf"]).is_absolute())
            self.assertEqual((wrapper_path.parent / wrapper["objects"][0]["urdf"]).resolve(), prepared)
            self.assertEqual(create_robot_config(robot)["robot_urdf"], str(prepared))
        self.assertEqual(robot.read_text(), original)

    def test_ambiguous_local_package_assets_require_an_explicit_choice(self):
        (self.root / "robot_description" / "meshes").mkdir(parents=True)
        (self.root / "robot_description" / "meshes" / "wheel.obj").write_text("v 0 0 0\n")
        self.urdf.write_text(URDF.replace("meshes/wheel.obj", "package://robot_description/meshes/wheel.obj"))
        with self.assertRaisesRegex(ValueError, "package URI.*ambiguous"):
            inspect_robot_urdf(self.urdf)

    def test_json_field_with_package_urdf_gets_a_normalized_wrapper(self):
        self.urdf.write_text(URDF.replace("meshes/wheel.obj", "package://robot_description/meshes/wheel.obj"))
        source = self.write_field({"field": {}, "objects": [{"urdf": self.urdf.name}]})
        original = source.read_bytes()
        with patch("eersim.importers.Path.cwd", return_value=self.root):
            result = import_field(source)
        self.assertNotEqual(result["path"], str(source))
        wrapper_path = Path(result["path"])
        data = json.loads(wrapper_path.read_text())
        self.assertNotEqual(data["objects"][0]["urdf"], str(self.urdf))
        self.assertFalse(Path(data["objects"][0]["urdf"]).is_absolute())
        self.assertTrue((wrapper_path.parent / data["objects"][0]["urdf"]).is_file())
        self.assertEqual(source.read_bytes(), original)

    def test_malformed_xml_and_non_urdf_fail_before_physics_loading(self):
        self.urdf.write_text("<robot>")
        with self.assertRaisesRegex(ValueError, "Cannot read URDF XML"):
            inspect_robot_urdf(self.urdf)
        self.urdf.write_text("<mujoco/>")
        with self.assertRaisesRegex(ValueError, "<robot> root"):
            inspect_robot_urdf(self.urdf)

    def test_joint_tree_and_limits_are_checked(self):
        self.urdf.write_text(URDF.replace('child link="wheel"', 'child link="missing"'))
        with self.assertRaisesRegex(ValueError, "existing parent and child"):
            inspect_robot_urdf(self.urdf)
        self.urdf.write_text(URDF.replace('type="continuous"', 'type="revolute"'))
        with self.assertRaisesRegex(ValueError, "needs lower and upper limits"):
            inspect_robot_urdf(self.urdf)
        self.urdf.write_text(URDF.replace("</robot>", '<link name="detached"/></robot>'))
        with self.assertRaisesRegex(ValueError, "one connected root"):
            inspect_robot_urdf(self.urdf)
        self.urdf.write_text(URDF.replace('child link="wheel"', 'child link="base"'))
        with self.assertRaisesRegex(ValueError, "joint cycle"):
            inspect_robot_urdf(self.urdf)

    def test_invalid_urdf_primitive_dimensions_fail_before_loading(self):
        self.urdf.write_text(URDF.replace('size=".3 .3 .1"', 'size=".3 0 .1"'))
        with self.assertRaisesRegex(ValueError, "box.size must be positive"):
            inspect_robot_urdf(self.urdf)

    def test_single_urdf_field_wraps_without_duplicate_floor_or_walls(self):
        data = load_field_import(self.urdf)
        self.assertFalse(data["field"]["floor"])
        self.assertFalse(data["field"]["walls"])
        self.assertTrue(data["objects"][0]["fixed"])
        self.assertEqual(data["objects"][0]["urdf"], str(self.urdf))
        output_dir = self.root / "workspace" / ".eersim" / "imports"
        result = import_field(self.urdf, output_dir)
        path = Path(result["path"])
        self.assertTrue(path.is_file())
        self.assertEqual(path.parent, output_dir)
        self.assertEqual(result["metadata"]["name"], "test_bot")
        self.assertEqual(result["metadata"]["object_count"], 1)
        self.assertEqual(import_field(self.urdf, output_dir)["path"], str(path))
        self.assertEqual(self.urdf.read_text(), URDF)

    def test_json_import_resolves_urdf_paths_and_preserves_source(self):
        data = {"schema_version": 1, "name": "My game", "field": {"size": [4, 5]},
                "objects": [{"urdf": "robot.urdf", "fixed": True}],
                "mode_durations": {"match_auto": 15}, "author": "test"}
        source = self.write_field(data)
        original = source.read_bytes()
        normalized = load_field_import(source)
        self.assertEqual(normalized["objects"][0]["urdf"], str(self.urdf))
        self.assertEqual(normalized["author"], "test")
        result = import_field(source)
        self.assertEqual(result["path"], str(source))
        target = self.root / "elsewhere" / "saved.json"
        self.assertEqual(materialize_field_import(source, target), target)
        self.assertEqual(json.loads(target.read_text())["objects"][0]["urdf"], "../robot.urdf")
        self.assertEqual(load_field_import(target)["objects"][0]["urdf"], str(self.urdf))
        self.assertEqual(source.read_bytes(), original)

    def test_generated_package_field_and_meshes_survive_moving_asset_tree(self):
        self.urdf.write_text(URDF.replace("meshes/wheel.obj", "package://robot_description/meshes/wheel.obj"))
        original_robot = self.urdf.read_bytes()
        source = self.write_field({"field": {}, "objects": [{"urdf": self.urdf.name}]})
        original_field = source.read_bytes()
        output = self.root / "experiments" / "portable.field.json"
        with patch("eersim.importers.Path.cwd", return_value=self.root):
            materialize_field_import(source, output)
        saved = json.loads(output.read_text())
        self.assertEqual(saved["import_metadata"]["source"], "../field.json")
        self.assertTrue(all(not Path(value).is_absolute()
                            for value in saved["import_metadata"]["normalized_urdfs"]))
        self.assertEqual(source.read_bytes(), original_field)
        self.assertEqual(self.urdf.read_bytes(), original_robot)
        with tempfile.TemporaryDirectory(prefix="eersim-relocated-") as destination:
            relocated = Path(destination) / "moved asset tree"
            self.root.rename(relocated)
            self.assertFalse(self.root.exists())
            loaded = load_field_import(relocated / "experiments" / output.name)
            prepared = Path(loaded["objects"][0]["urdf"])
            self.assertTrue(prepared.is_relative_to(relocated))
            mesh_reference = ET.parse(prepared).find(".//mesh").get("filename")
            self.assertEqual((prepared.parent / mesh_reference).resolve(), relocated / "meshes" / "wheel.obj")
            self.assertEqual(inspect_robot_urdf(prepared)["name"], "test_bot")
            self.assertEqual((relocated / "robot.urdf").read_bytes(), original_robot)
            self.assertEqual((relocated / "field.json").read_bytes(), original_field)

    def test_rematerialized_field_metadata_keeps_its_original_source(self):
        first = self.root / "first" / "field.json"
        second = self.root / "second" / "nested" / "field.json"
        materialize_field_import(self.urdf, first)
        materialize_field_import(first, second)
        metadata = json.loads(second.read_text())["import_metadata"]
        self.assertFalse(Path(metadata["source"]).is_absolute())
        self.assertEqual((second.parent / metadata["source"]).resolve(), self.urdf)
        self.assertEqual(load_field_import(second)["objects"][0]["urdf"], str(self.urdf))

    def test_explicit_field_destination_and_source_overwrite_protection(self):
        destination = self.root / "selected-name.json"
        result = import_field(self.urdf, destination)
        self.assertEqual(result["path"], str(destination))
        with self.assertRaisesRegex(ValueError, "preserve the source"):
            materialize_field_import(destination, destination)
        with self.assertRaisesRegex(ValueError, "different from the source"):
            import_field(destination, destination)

    def test_invalid_field_schema_geometry_and_nonfinite_values_are_rejected(self):
        cases = [
            ({"motors": []}, "field.*objects"),
            ({"schema_version": 2, "field": {}, "objects": []}, "schema_version"),
            ({"field": {"size": [0, 1]}, "objects": []}, "positive"),
            ({"field": {}, "objects": [{"shape": "mesh"}]}, "unsupported"),
            ({"field": {}, "objects": [{"position": [0, float("nan"), 1]}]}, "finite"),
            ({"field": {}, "objects": [{"mass": -1}]}, "at least 0"),
            ({"field": {}, "objects": [{"color": [2, 0, 0, 1]}]}, "between 0 and 1"),
            ({"field": {}, "objects": [{"urdf": "missing.urdf"}]}, "file is missing"),
        ]
        for data, message in cases:
            with self.subTest(data=data):
                with self.assertRaisesRegex(ValueError, message):
                    load_field_import(self.write_field(data))


if __name__ == "__main__":
    unittest.main()
