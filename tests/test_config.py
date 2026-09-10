"""Saved experiments survive relocation without embedding installation paths."""
from copy import deepcopy
import json
from pathlib import Path
import shutil

import pytest

from eersim import config


@pytest.fixture
def assets(tmp_path, monkeypatch):
    root = tmp_path / "installation" / "assets"
    for name in ("robots/xdrive.urdf", "robots/xdrive.json",
                 "games/override-2026-27.json", "objects/pin.urdf"):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}")
    monkeypatch.setattr(config, "ASSETS", root)
    return root


@pytest.mark.parametrize("as_dataclass", [False, True])
def test_bundled_assets_reroot_on_another_installation(tmp_path, monkeypatch, assets, as_dataclass):
    data = config.default_config()
    data.update(notes={"purpose": "repeatable trial"},
                field_objects=[{"name": "preload", "urdf": str(assets / "objects/pin.urdf")}])
    original = config.SimConfig.from_dict(data) if as_dataclass else data
    before = deepcopy(original.to_dict() if as_dataclass else original)
    profile = tmp_path / "profiles" / "trial.json"
    config.save_config(original, profile)
    saved = json.loads(profile.read_text())
    assert saved["robot_urdf"] == "asset://robots/xdrive.urdf"
    assert saved["robot_config"] == "asset://robots/xdrive.json"
    assert saved["game_pack"] == "asset://games/override-2026-27.json"
    assert saved["field_objects"][0]["urdf"] == "asset://objects/pin.urdf"
    assert (original.to_dict() if as_dataclass else original) == before

    installed = tmp_path / "another-installation" / "share" / "eersim" / "assets"
    shutil.copytree(assets, installed)
    monkeypatch.setattr(config, "ASSETS", installed)
    loaded = config.load_config(profile)
    assert Path(loaded["robot_urdf"]) == installed / "robots/xdrive.urdf"
    assert Path(loaded["robot_config"]) == installed / "robots/xdrive.json"
    assert Path(loaded["game_pack"]) == installed / "games/override-2026-27.json"
    assert Path(loaded["field_objects"][0]["urdf"]) == installed / "objects/pin.urdf"
    assert loaded["notes"] == before["notes"]
    assert Path(config.default_config()["robot_urdf"]).is_relative_to(installed)


def test_profile_and_user_assets_can_move_together(tmp_path, assets):
    bundle = tmp_path / "original"
    relative = {"workspace": "robot-code", "robot_urdf": "robots/custom.urdf",
                "robot_config": "robots/custom.json", "game_pack": "fields/game.json"}
    data = config.default_config()
    for key, name in relative.items():
        target = bundle / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.mkdir() if key == "workspace" else target.write_text("{}")
        data[key] = str(target)
    data["field_objects"] = [{"name": "custom", "urdf": "../robots/custom.urdf"}]
    data["camera"] = {"yaw": 42, "target": [1, 2, 0]}
    before = deepcopy(data)
    profile = bundle / "experiments" / "trial.json"
    config.save_config(data, profile)
    saved = json.loads(profile.read_text())
    assert {key: saved[key] for key in relative} == {key: "../" + name for key, name in relative.items()}
    assert data == before

    moved = tmp_path / "relocated"
    shutil.copytree(bundle, moved)
    loaded = config.load_config(moved / "experiments/trial.json")
    assert all(Path(loaded[key]) == moved / name for key, name in relative.items())
    assert all(Path(loaded[key]).exists() for key in relative)
    # Embedded user URDFs retain the existing game-pack-relative convention.
    embedded = Path(loaded["game_pack"]).parent / loaded["field_objects"][0]["urdf"]
    assert embedded.resolve() == moved / "robots/custom.urdf"
    assert loaded["camera"] == before["camera"]
    config.save_config(loaded, moved / "experiments/trial.json")
    assert json.loads((moved / "experiments/trial.json").read_text()) == saved


@pytest.mark.parametrize("reference", ["asset://../secret", "asset:///absolute", "asset://C:/secret",
    "asset://robots/../../secret", "asset://robots\\..\\secret", "asset://", "asset://robots//file"])
def test_asset_references_cannot_escape_the_asset_root(tmp_path, assets, reference):
    profile = tmp_path / "invalid.json"
    profile.write_text(json.dumps({"robot_urdf": reference}))
    with pytest.raises(ValueError, match="asset reference"):
        config.load_config(profile)
    with pytest.raises(ValueError, match="asset reference"):
        config.save_config({"robot_urdf": reference}, tmp_path / "invalid-save.json")


def test_cross_drive_relative_path_failure_keeps_an_absolute_reference(tmp_path, monkeypatch, assets):
    robot = tmp_path / "external" / "custom.urdf"
    data = config.default_config()
    data["robot_urdf"] = str(robot)

    def different_volume(*args):
        raise ValueError("path is on a different Windows drive")

    monkeypatch.setattr(config.os.path, "relpath", different_volume)
    profile = tmp_path / "profile.json"
    config.save_config(data, profile)
    saved = json.loads(profile.read_text())
    assert saved["robot_urdf"] == robot.as_posix()
    assert Path(config.load_config(profile)["robot_urdf"]) == robot


def test_legacy_bom_absolute_paths_and_empty_bindings_remain_supported(tmp_path, assets):
    profile = tmp_path / "legacy.json"
    data = config.default_config()
    data.update(robot_config="", workspace="", custom_option={"enabled": True})
    profile.write_text(json.dumps(data), encoding="utf-8-sig")
    loaded = config.load_config(profile)
    assert loaded == data
    config.save_config(loaded, profile)
    assert config.load_config(profile) == data
