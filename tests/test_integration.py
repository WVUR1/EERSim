"""Compile the actual club robot and drive the contact physics through PROS."""
import os
from pathlib import Path

import pytest

from eersim.bridge import _compiler, build_workspace, BuildError
from eersim.config import default_config
from eersim.runner import Session


def test_repository_driver_moves_robot_and_disables_at_time_limit(tmp_path):
    configured_workspace = os.environ.get("EERSIM_EERKIT_WORKSPACE")
    if not configured_workspace:
        pytest.skip("Optional EERKit integration requires EERSIM_EERKIT_WORKSPACE")
    workspace = Path(configured_workspace).expanduser().resolve()
    if not (workspace / "src/main.cpp").is_file() or not (workspace / "include").is_dir():
        pytest.fail(f"EERSIM_EERKIT_WORKSPACE must name an EERKit checkout containing src/main.cpp and include/: {workspace}")
    try:
        _compiler()
    except BuildError:
        pytest.skip("Native C++ compiler required for repository integration")
    executable = build_workspace(workspace)
    config = default_config()
    config["duration_overrides"] = {"skills_driver": 0.75}
    session = Session(config)
    try:
        session.start("skills_driver", executable)
        session.controller = {"analog": {"left_y": 80}, "digital": {}}
        start_x = session.world.snapshot()["pose"][0]
        while session.running:
            session.step()
        state = session.state()
        assert state["initialized"]
        assert state["mode_time"] == pytest.approx(0.75)
        assert state["pose"][0] > start_x + 0.1
        assert session.runtime is None
        assert state["phase"] == "disabled"
        assert all(command["value"] == 0 for command in session.world._commands["motors"].values())
        path = tmp_path / "run.csv"
        session.export_csv(path)
        assert len(path.read_text().splitlines()) == state["telemetry_samples"] + 1
    finally:
        session.close()


def test_native_claw_demo_grips_lifts_and_places_its_preload():
    """The preset must complete a real PROS routine, not merely draw a claw."""
    try:
        _compiler()
    except BuildError:
        pytest.skip("Native C++ compiler required for claw integration")
    from eersim.demos import claw_demo_config
    import pybullet as bullet

    config = claw_demo_config()
    executable = build_workspace(Path(config["workspace"]))
    logs = []
    session = Session(config, log=logs.append)
    try:
        session.start("skills_auto", executable)
        preload = next(item["id"] for item in session.world.field_objects if item["name"] == "demo_preload")
        initial_height = bullet.getBasePositionAndOrientation(preload, physicsClientId=session.world.client)[0][2]
        highest = initial_height
        while session.running:
            state = session.step()
            if session.elapsed_ticks % 24 == 0:
                highest = max(highest, bullet.getBasePositionAndOrientation(
                    preload, physicsClientId=session.world.client)[0][2])
        assert highest > initial_height + .15, "The claw must physically lift its preload"
        assert any(item["pin"] == "demo_preload" and item["alliance"] == "red"
                   for item in state["scoring"]["placements"]), "The demo preload must remain in a red goal"
        assert any("Placement complete" in line for line in logs)
        assert not any("Route timeout" in line for line in logs)
        assert session.runtime is None
        assert state["phase"] == "disabled"
    finally:
        session.close()
