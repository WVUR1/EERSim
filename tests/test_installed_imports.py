"""Custom assets must load without writing to an installed application folder."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from eersim.config import default_config, load_config, save_config
from eersim.importers import create_robot_config, default_import_directory, import_field
from eersim.physics import World
from eersim import runtime_paths


_MESH = """v -.05 -.05 -.05
v .05 -.05 -.05
v .05 .05 -.05
v -.05 .05 -.05
v -.05 -.05 .05
v .05 -.05 .05
v .05 .05 .05
v -.05 .05 .05
f 1 3 2
f 1 4 3
f 5 6 7
f 5 7 8
f 1 2 6
f 1 6 5
f 2 3 7
f 2 7 6
f 3 4 8
f 3 8 7
f 4 1 5
f 4 5 8
"""
_LINK = """<inertial><mass value=".1"/><inertia ixx=".001" iyy=".001" izz=".001" ixy="0" ixz="0" iyz="0"/></inertial>
<visual><geometry><mesh filename="package://custom_model/meshes/box.obj"/></geometry><material name="custom"><color rgba=".3 .7 .9 1"/></material></visual>
<collision><geometry><box size=".1 .1 .1"/></geometry></collision>"""


@pytest.mark.parametrize("field_format", ["json", "urdf"])
def test_external_assets_and_profile_work_with_read_only_install_cwd(monkeypatch, tmp_path, field_format):
    installed = tmp_path / "read-only installation"
    installed.mkdir()
    external = tmp_path / "user assets" / "custom_model"
    (external / "urdf").mkdir(parents=True)
    (external / "meshes").mkdir()
    (external / "meshes" / "box.obj").write_text(_MESH)
    robot = external / "urdf" / "robot.urdf"
    robot.write_text(f'<robot name="external_robot"><link name="base">{_LINK}</link>'
                     f'<link name="rotor">{_LINK}</link><joint name="rotor_spin" type="continuous">'
                     '<parent link="base"/><child link="rotor"/><origin xyz="0 0 .15"/>'
                     '<axis xyz="0 0 1"/><limit effort="2.1" velocity="63"/></joint></robot>')
    bindings = {"motors": [{"port": 6, "joint": "rotor_spin", "cartridge": "green"}]}
    robot.with_suffix(".json").write_text(json.dumps(bindings))
    field = external / "urdf" / "field.urdf"
    field.write_text(f'<robot name="external_field"><link name="field_base">{_LINK}</link></robot>')
    game = external / "field.json"
    game.write_text(json.dumps({"name": "External test field", "field": {"size": [2, 2], "walls": False},
                               "objects": [{"name": "external_obstacle", "urdf": "urdf/field.urdf", "fixed": True,
                                            "position": [.5, 0, .1]}]}))
    originals = {path: path.read_bytes() for path in external.rglob("*") if path.is_file()}
    user_data = tmp_path / "user data"
    monkeypatch.chdir(installed)
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(runtime_paths, "user_data_dir", lambda: user_data)
    monkeypatch.setenv("EERSIM_RENDERER", "cpu")
    mkdir = Path.mkdir

    def reject_install_writes(path, *args, **kwargs):
        # Windows does not enforce directory write protection through chmod.
        # Enforce that boundary while performing real imports and Bullet loads.
        if path.resolve().is_relative_to(installed):
            raise PermissionError("The installed application directory is read-only")
        return mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", reject_install_writes)
    patch = create_robot_config(robot)
    imported_field = import_field(game if field_format == "json" else field)
    assert Path(patch["robot_urdf"]).is_relative_to(user_data / "imports")
    assert Path(imported_field["path"]).is_relative_to(user_data / "imports")
    assert patch["robot_config"] == str(robot.with_suffix(".json"))
    assert patch["motors"] == bindings["motors"]
    for prepared in (user_data / "imports").glob("*.urdf"):
        reference = ET.parse(prepared).find(".//mesh").get("filename")
        assert (prepared.parent / reference).resolve() == external / "meshes" / "box.obj"

    config = default_config()
    config.update(patch, game_pack=imported_field["path"], start_pose=[0, 0, .25, 0],
                  gravity=0, fixed_base=True, renderer="cpu")
    profile = tmp_path / "user experiments" / "external.json"
    save_config(config, profile)
    saved = json.loads(profile.read_text())
    assert not Path(saved["robot_urdf"]).is_absolute()
    assert not Path(saved["game_pack"]).is_absolute()
    with World(load_config(profile)) as world:
        assert [joint["name"] for joint in world.joints] == ["rotor_spin"]
        assert world.motor_bindings[0]["port"] == 6
        for _ in range(12):
            snapshot = world.step({"motors": {"6": {"mode": "voltage", "value": 6000}}})
        assert abs(snapshot["joints"][0]["position"]) > .001
        frame = world.render(160, 120, distance=2.5)
        assert frame.shape == (120, 160, 4) and np.ptp(frame[:, :, :3]) > 30
        assert len(world.field_objects) >= 1
    assert not list(installed.iterdir())
    assert all(path.read_bytes() == original for path, original in originals.items())


def test_import_destination_retains_explicit_workspace_and_source_behavior(monkeypatch, tmp_path):
    monkeypatch.chdir(tmp_path)
    monkeypatch.delattr(sys, "frozen", raising=False)
    assert default_import_directory() == tmp_path / ".eersim" / "imports"
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    workspace = tmp_path / "selected workspace"
    assert default_import_directory(workspace) == workspace / ".eersim" / "imports"


def test_frozen_save_dialog_defaults_are_writable_and_preserve_user_choices(monkeypatch, tmp_path):
    from eersim.gui import _dialog_default

    installed = tmp_path / "installed application"
    installed.mkdir()
    monkeypatch.chdir(installed)
    user = tmp_path / "per-user data"
    monkeypatch.setattr(runtime_paths, "user_data_dir", lambda: user)
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    profile = Path(_dialog_default("experiment.json", "experiments"))
    telemetry = Path(_dialog_default("telemetry.csv", "telemetry"))
    assert profile == user / "experiments" / "experiment.json"
    assert telemetry == user / "telemetry" / "telemetry.csv"
    assert profile.parent.is_dir() and telemetry.parent.is_dir()
    chosen = tmp_path / "selected experiments" / "run.json"
    workspace = tmp_path / "selected workspace"
    assert _dialog_default("experiment.json", "experiments", chosen, workspace) == str(chosen)
    assert _dialog_default("experiment.json", "experiments", workspace=workspace) == str(workspace / "experiment.json")
    assert not list(installed.iterdir())
    monkeypatch.delattr(sys, "frozen", raising=False)
    assert _dialog_default("experiment.json", "experiments") == "experiment.json"
