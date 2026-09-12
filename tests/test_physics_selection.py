"""Backend selections survive profiles and reach sessions without GUI startup."""
import copy
import json
import sys
from types import SimpleNamespace

import pytest

from eersim import config, runner
from eersim.__main__ import main


@pytest.mark.parametrize("backend", ["auto", "bullet", "physx_gpu", "physx_cpu"])
def test_backend_experiment_round_trip(tmp_path, backend):
    original = config.default_config()
    original.update(physics_backend=backend, physics_device=3, physics_solver_iterations=64,
                    renderer="cpu", physics_hz=480)
    before = copy.deepcopy(original)
    path = tmp_path / "experiment.json"
    config.save_config(original, path)
    loaded = config.load_config(path)
    assert loaded == original
    assert original == before
    assert json.loads(path.read_text())["physics_device"] == 3


@pytest.mark.parametrize("field,value", [
    ("physics_backend", "gpu"), ("physics_backend", None),
    ("physics_device", -1), ("physics_device", True), ("physics_device", 0.0),
    ("physics_device", "0"), ("physics_solver_iterations", 0),
    ("physics_solver_iterations", 256), ("physics_solver_iterations", True),
    ("physics_solver_iterations", 100.0),
])
def test_invalid_backend_settings_rejected(field, value):
    with pytest.raises(ValueError, match=field):
        config.SimConfig.from_dict({field: value})


def test_old_profiles_receive_explicit_backend_defaults(tmp_path):
    path = tmp_path / "old.json"
    path.write_text('{"physics_hz": 240, "renderer": "gpu"}')
    loaded = config.load_config(path)
    assert loaded["physics_backend"] == "auto"
    assert loaded["physics_device"] == 0
    assert loaded["physics_solver_iterations"] == 100
    assert loaded["renderer"] == "gpu"


def test_cli_backend_overrides_do_not_change_renderer_or_step_rate(tmp_path, monkeypatch):
    path = tmp_path / "experiment.json"
    config.save_config({"physics_backend": "bullet", "physics_device": 0,
                        "physics_solver_iterations": 100, "physics_hz": 480,
                        "renderer": "cpu"}, path)
    received = []
    monkeypatch.setitem(sys.modules, "eersim.gui", SimpleNamespace(
        run=lambda selected, workspace: received.append(selected) or 0))
    assert main(["--config", str(path), "--physics-backend", "physx_gpu",
                 "--physics-device", "2", "--solver-iterations", "32"]) == 0
    assert len(received) == 1
    selected = received[0]
    assert (selected["physics_backend"], selected["physics_device"],
            selected["physics_solver_iterations"]) == ("physx_gpu", 2, 32)
    assert selected["renderer"] == "cpu"
    assert selected["physics_hz"] == 480
    assert config.load_config(path)["physics_backend"] == "bullet"


@pytest.mark.parametrize("option,value", [("--physics-device", "-1"),
                                          ("--solver-iterations", "256")])
def test_cli_rejects_invalid_device_or_iterations_before_world_creation(monkeypatch, capsys, option, value):
    def unexpected_session(*args, **kwargs):
        pytest.fail("Invalid physics configuration must not create a World")
    monkeypatch.setattr(runner, "Session", unexpected_session)
    assert main(["--headless", option, value]) == 1
    assert "must be" in capsys.readouterr().err


class SelectedWorld:
    joints = []
    motor_bindings = []
    renderer_status = {"backend": "cpu", "device": "TinyRenderer",
                       "hardware_accelerated": False}

    def __init__(self, selected):
        self.selected = copy.deepcopy(selected)
        self.physics_status = {"backend": "physx", "device": "CUDA device 2",
                               "hardware_accelerated": True, "reason": "Validated CUDA scene"}

    def snapshot(self):
        return {"time": 0, "pose": [0, 0, 0, 0]}

    def disable(self):
        pass

    def close(self):
        pass


def test_default_session_routes_factory_and_reports_actual_physics_independently(monkeypatch):
    calls, logs = [], []

    def create_world(selected):
        calls.append(copy.deepcopy(selected))
        return SelectedWorld(selected)

    monkeypatch.setitem(sys.modules, "eersim.physics_backend", SimpleNamespace(create_world=create_world))
    selected = config.default_config()
    selected.update(physics_backend="physx_gpu", physics_device=2, renderer="cpu")
    session = runner.Session(selected, logs.append, runtime_factory=lambda _: None)
    try:
        assert calls == [selected]
        state = session.state()
        assert state["physics"]["hardware_accelerated"] is True
        assert state["renderer"]["hardware_accelerated"] is False
        assert state["physics"]["device"] == "CUDA device 2"
        assert any("Physics: physx" in entry and "CUDA device 2" in entry for entry in logs)
        state["physics"]["device"] = "consumer edit"
        assert session.state()["physics"]["device"] == "CUDA device 2"
    finally:
        session.close()


def test_injected_world_factory_remains_supported_without_backend_module(monkeypatch):
    monkeypatch.setitem(sys.modules, "eersim.physics_backend", None)
    session = runner.Session({"physics_hz": 240}, world_factory=SelectedWorld,
                             runtime_factory=lambda _: None)
    try:
        assert isinstance(session.world, SelectedWorld)
    finally:
        session.close()


@pytest.mark.parametrize("failure", ["runtime_missing", "cuda_unavailable"])
def test_auto_fallback_reason_survives_resets_and_explicit_bullet_clears_it(tmp_path, monkeypatch, failure):
    """Exercise real World.reset with one fixed link; no GPU, steps, or meshes."""
    pytest.importorskip("pybullet")
    from eersim import physics_backend

    robot = tmp_path / "single-link.urdf"
    robot.write_text('''<robot name="single-link"><link name="base_link">
      <inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01"
        ixy="0" ixz="0" iyz="0"/></inertial>
      <collision><geometry><box size=".1 .1 .1"/></geometry></collision>
      </link></robot>''')
    field = tmp_path / "empty-field.json"
    field.write_text(json.dumps({"field": {"size": [2, 2], "floor": False, "walls": False}, "objects": []}))
    selected = {"physics_backend": "auto", "robot_urdf": str(robot), "robot_config": "",
                "game_pack": str(field), "fixed_base": True, "motors": []}
    before = copy.deepcopy(selected)
    monkeypatch.setattr(physics_backend, "find_spec", lambda _: None if failure == "runtime_missing" else object())
    monkeypatch.setattr(physics_backend, "cuda_device_info",
                        lambda _: {"available": False, "reason": "No CUDA device available for this test"})
    world = physics_backend.create_world(selected)
    try:
        reason = world.physics_status["reason"]
        assert reason
        if failure == "runtime_missing":
            assert "not installed" in reason
        else:
            assert reason == "No CUDA device available for this test"
        assert world.config.physics_backend == "auto"
        assert world.physics_status["backend"] == "bullet"
        assert not world.physics_status["hardware_accelerated"]
        for settings in (None, copy.deepcopy(selected)):
            world.reset(settings)
            assert world.physics_status["reason"] == reason
            assert world.snapshot()["physics"]["reason"] == reason
        world.reset({**selected, "physics_backend": "bullet"})
        assert world.config.physics_backend == "bullet"
        assert not world.physics_status.get("reason")
        assert selected == before
    finally:
        world.close()


def test_failed_auto_physx_initialization_restores_auto_selection_before_bullet_fallback(monkeypatch):
    """The failed attempt's temporary physx_gpu selection must not leak into resets."""
    from eersim import physics_backend

    attempts, fallbacks = [], []

    def fail(selected):
        attempts.append(copy.deepcopy(selected))
        raise RuntimeError("GPU scene cooking rejected the fixture")

    class FallbackWorld:
        def __init__(self, selected):
            fallbacks.append(copy.deepcopy(selected))
            self.config = config.SimConfig.from_dict(selected)
            self.physics_status = {"backend": "bullet", "hardware_accelerated": False}

    monkeypatch.setattr(physics_backend, "find_spec", lambda _: object())
    monkeypatch.setattr(physics_backend, "cuda_device_info", lambda _: {"available": True})
    monkeypatch.setitem(sys.modules, "eersim.physx_world", SimpleNamespace(PhysXWorld=fail))
    monkeypatch.setattr(physics_backend, "World", FallbackWorld)
    selected = {"physics_backend": "auto", "physics_device": 2, "physics_solver_iterations": 64}
    before = copy.deepcopy(selected)
    world = physics_backend.create_world(selected)
    assert len(attempts) == len(fallbacks) == 1
    assert attempts[0]["physics_backend"] == "physx_gpu"
    assert fallbacks[0]["physics_backend"] == world.config.physics_backend == "auto"
    assert fallbacks[0]["physics_device"] == 2
    assert fallbacks[0]["physics_solver_iterations"] == 64
    assert world.physics_status["reason"] == "PhysX initialization failed: GPU scene cooking rejected the fixture"
    assert selected == before


def test_explicit_physx_failure_does_not_silently_fall_back_to_bullet(monkeypatch):
    from eersim import physics_backend

    def fail(selected):
        raise RuntimeError("GPU scene cooking rejected the fixture")

    monkeypatch.setattr(physics_backend, "find_spec", lambda _: object())
    monkeypatch.setattr(physics_backend, "cuda_device_info", lambda _: {"available": True})
    monkeypatch.setitem(sys.modules, "eersim.physx_world", SimpleNamespace(PhysXWorld=fail))
    monkeypatch.setattr(physics_backend, "World", lambda _: pytest.fail("Explicit PhysX selection must fail visibly"))
    with pytest.raises(RuntimeError, match="GPU scene cooking rejected"):
        physics_backend.create_world({"physics_backend": "physx_gpu"})
