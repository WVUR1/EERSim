"""GUI method tests with plain Python stand-ins, without any Qt application."""
from copy import deepcopy
from types import MethodType, SimpleNamespace

import pytest

from eersim import gui, engine


@pytest.fixture(autouse=True)
def no_application_created():
    before = gui.QApplication.instance()
    yield
    assert gui.QApplication.instance() is before


class Field:
    def __init__(self, value=0): self.data = value; self.enabled = True
    def text(self): return str(self.data)
    def value(self): return self.data
    def setText(self, value): self.data = value
    def setValue(self, value): self.data = value
    def setEnabled(self, value): self.enabled = value
    def setVisible(self, value): self.visible = value
    def setMaximum(self, value): self.maximum = value
    def setCurrentIndex(self, value): self.data = value
    def findData(self, value): self.selected = value; return 0
    def currentData(self): return getattr(self, "selected", self.data)
    def setChecked(self, value): self.checked = value
    def showMessage(self, value): self.message = value


def widgets():
    obj = SimpleNamespace(config={}, _scan_joints=lambda *args: None,
        _bindings_selected=lambda *args, **kwargs: None, _describe_game=lambda *args: None,
        _fill_motors=lambda *args: None, _read_motors=lambda: [], _workspace_key=lambda: "",
        scene=SimpleNamespace(yaw=1, pitch=-2, distance=3, target=[0, 0, 0]),
        noise_fields={}, duration_fields={key: Field(30) for key, *_ in gui.MODES},
        pose_fields=[Field() for _ in range(4)])
    for name in ("urdf_path", "binding_path", "game_path", "physics_hz", "physics_select",
                 "physics_device", "physics_iterations", "physics_cpu_threads", "render_fps",
                 "renderer_select", "gravity", "friction", "plastic_friction", "restitution",
                 "seed", "experiment_notes", "workspace"):
        setattr(obj, name, Field())
    return obj


def test_cpu_worker_field_round_trips_inline_and_preserves_larger_saved_values(monkeypatch):
    monkeypatch.setattr(gui.os, "cpu_count", lambda: 8)
    obj = widgets()
    gui.Workbench._fill_widgets(obj, {"physics_cpu_threads": 48, "physics_backend": "physx_cpu"})
    assert obj.physics_cpu_threads.value() == 48
    assert obj.physics_cpu_threads.maximum == 48 and obj.physics_cpu_threads.enabled
    config = gui.Workbench._collect_config(obj)
    assert config["physics_cpu_threads"] == 48
    gui.Workbench._fill_widgets(obj, {"physics_cpu_threads": 0, "physics_backend": "bullet"})
    assert obj.physics_cpu_threads.value() == 0
    assert obj.physics_cpu_threads.maximum == 8 and not obj.physics_cpu_threads.enabled
    assert gui.Workbench._collect_config(obj)["physics_cpu_threads"] == 0


def test_start_requests_unchanged_reuse_and_never_compiles(monkeypatch):
    events = []
    obj = SimpleNamespace(_live_action_allowed=lambda: True, _built_workspace="source",
        _workspace_key=lambda: "source", _mode="match_auto",
        _apply_config=lambda **kw: events.append(("apply", kw)) or True,
        engine=SimpleNamespace(start=lambda mode: events.append(("start", mode))),
        scene=SimpleNamespace(setFocus=lambda: None), _on_failed=lambda message: pytest.fail(message))
    gui.Workbench._start(obj)
    assert events == [("apply", {"reuse_unchanged": True}), ("start", "match_auto")]
    obj._built_workspace = None
    monkeypatch.setattr(gui, "_local_feedback", lambda window, title, message, **kwargs:
                        events.append(("warning", message, kwargs)))
    gui.Workbench._start(obj)
    assert events[-1][0] == "warning" and "Build" in events[-1][1]
    assert events[-1][2] == {"warning": True}
    assert len(events) == 3


def test_demo_load_is_explicit_build_required_not_implicit_compilation(monkeypatch):
    from eersim import demos
    calls = []
    config = {"workspace": "bundled-example"}
    monkeypatch.setattr(demos, "claw_demo_config", lambda: deepcopy(config))
    obj = SimpleNamespace(_live_action_allowed=lambda: True,
        _fill_widgets=lambda value: calls.append(("fill", value)),
        _apply_config=lambda: calls.append(("apply",)) or True,
        _select_mode=lambda mode: calls.append(("mode", mode)),
        mode_buttons={"skills_auto": Field()}, _build=lambda: pytest.fail("Implicit demo build"),
        _built_workspace="previous", start_button=Field(), build_status=Field(),
        _log=lambda message: calls.append(("log", message)), _on_failed=lambda message: pytest.fail(message))
    gui.Workbench._load_demo(obj)
    assert obj.config == config and obj._built_workspace is None
    assert not obj.start_button.enabled
    assert "Build workspace" in obj.build_status.text()
    assert calls[:3] == [("fill", config), ("apply",), ("mode", "skills_auto")]


def test_apply_default_reload_and_start_opt_in_are_forwarded(tmp_path):
    robot = tmp_path / "robot.urdf"
    robot.write_text("fixture", encoding="utf-8")
    game = tmp_path / "field.json"
    game.write_text("{}", encoding="utf-8")
    config = {"robot_urdf": str(robot), "robot_config": "", "game_pack": str(game)}
    calls = []
    status = Field()
    obj = SimpleNamespace(_live_action_allowed=lambda: True, _collect_config=lambda: deepcopy(config),
        _neutral_controller=lambda: None, _pending_scene_applies=0, _scene_apply_ready=False,
        reset_button=Field(),
        _logo_event=lambda event: None, _on_failed=lambda message: pytest.fail(message),
        engine=SimpleNamespace(configure=lambda value, **kw: calls.append((value, kw)),
                               set_camera=lambda *args: None),
        scene=SimpleNamespace(yaw=1, pitch=-2, distance=3, target=[0, 0, 0]), statusBar=lambda: status)
    assert gui.Workbench._apply_config(obj)
    assert gui.Workbench._apply_config(obj, reuse_unchanged=True)
    assert calls == [(config, {"reuse_unchanged": False}), (config, {"reuse_unchanged": True})]
    assert obj._pending_scene_applies == 2 and status.message == gui.APPLY_STATUS
    assert not obj.reset_button.enabled


def test_frame_acknowledgement_happens_even_when_pending_view_blocks_display():
    mailbox = engine._LatestFrame()
    obj = SimpleNamespace(_view_generation=0, _replay_pending="enter", _replay={},
        engine=SimpleNamespace(take_frame=mailbox.take), _replay_blocked=lambda: True,
        scene=SimpleNamespace(frames=[]))
    obj.scene.set_frame = obj.scene.frames.append
    old = mailbox.publish("live", 0, "old")
    gui.Workbench._on_live_frame(obj, old)
    assert mailbox._packet is None and obj.scene.frames == []
    replay = mailbox.publish("replay", 1, "new")
    obj._view_generation = 1
    obj._replay = {"active": True}
    obj._replay_pending = None
    gui.Workbench._on_live_frame(obj, old)
    gui.Workbench._on_replay_frame(obj, replay)
    assert obj.scene.frames == ["new"]
    assert mailbox._packet is None


def test_busy_gui_receives_only_newest_frame_for_each_generation():
    mailbox = engine._LatestFrame()
    obj = SimpleNamespace(_view_generation=2, _replay_pending=None, _replay={},
        engine=SimpleNamespace(take_frame=mailbox.take), _replay_blocked=lambda: False,
        scene=SimpleNamespace(frames=[]))
    obj.scene.set_frame = obj.scene.frames.append
    token = mailbox.publish("live", 2, "initial")
    for index in range(100): mailbox.publish("live", 2, index)
    gui.Workbench._on_live_frame(obj, token)
    assert obj.scene.frames == [99]
    gui.Workbench._on_live_frame(obj, token)
    assert obj.scene.frames == [99]
