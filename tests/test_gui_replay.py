"""Replay GUI behavior without constructing Qt objects, a world, or a window."""
from copy import deepcopy
from types import MethodType, SimpleNamespace

import pytest

from eersim import gui


class Control:
    def __init__(self):
        self.text = ""
        self.enabled = True
        self.visible = True
        self.blocked = False
        self.value = 0
        self.events = []

    def setText(self, text): self.text = text
    def setToolTip(self, text): self.tooltip = text
    def setEnabled(self, value): self.enabled = value
    def setVisible(self, value): self.visible = value
    def setDown(self, value): self.down = value
    def display(self, *values): self.axes = values
    def showMessage(self, text): self.text = text
    def blockSignals(self, value):
        previous, self.blocked = self.blocked, value
        return previous
    def setRange(self, *values): self.events.append(("range", values, self.blocked))
    def setValue(self, value):
        self.value = value
        self.events.append(("value", value, self.blocked))
    def setSingleStep(self, value): self.single_step = value
    def setPageStep(self, value): self.page_step = value
    def cancel_seek(self): self.seek_cancelled = True


class Timer:
    def __init__(self): self.active, self.starts = False, 0
    def start(self): self.active, self.starts = True, self.starts + 1
    def stop(self): self.active = False
    def isActive(self): return self.active


class Engine:
    def __init__(self): self.calls = []
    def __getattr__(self, name):
        return lambda *args: self.calls.append((name, deepcopy(args)))


def bench(*, active=False, playing=False):
    scene = SimpleNamespace(scene_name="Current field", yaw=10, pitch=-20, distance=4,
        target=[0, 0, 0], frames=[], cameras=[], driving=True, update=lambda: None)
    scene.set_frame = scene.frames.append
    scene.set_camera = lambda *args, **kwargs: scene.cameras.append((args, kwargs))
    status = Control()
    obj = SimpleNamespace(
        engine=Engine(), scene=scene, _state={}, _replay={"available": True, "active": active,
            "playing": playing, "generation": 0, "duration_s": 12.345, "position_s": 0,
            "sample_count": 372, "effective_hz": 30, "mode": "match_auto", "termination": "completed"},
        _replay_pending=None, _replay_pending_generation=-1, _view_generation=0,
        _replay_dragging=False, _replay_resume_after_seek=False, _pending_replay_seek=None,
        _replay_seek_timer=Timer(), _replay_reason="", _building=False, _pending_scene_applies=0,
        _built_workspace="program", _workspace_key=lambda: "program", _gamepad=None, _pygame=None,
        _keys=set(), _controller={"analog": dict.fromkeys(gui.ANALOG_NAMES, 0),
                                   "digital": dict.fromkeys(gui.DIGITAL_NAMES, False)},
        statusBar=lambda: status, logs=[], _controller_timer=Timer(),
    )
    for name in ("replay_button", "replay_play_button", "replay_live_button", "replay_timeline", "replay_time",
                 "replay_status", "progress", "scene_subtitle", "telemetry_caption", "status_badge",
                 "start_button", "pause_button", "stop_button", "step_button", "reset_button", "apply_button",
                 "build_button", "demo_button", "joint_button", "workspace", "load_profile_button",
                 "save_profile_button", "left_stick", "right_stick"):
        setattr(obj, name, Control())
    obj.metric_captions = {name: Control() for name in ("time", "remaining", "rtf", "pose")}
    obj.metric_values = {name: Control() for name in obj.metric_captions}
    obj.digital_buttons = {name: Control() for name in gui.DIGITAL_NAMES}
    obj.mode_buttons = {name: Control() for name, *_ in gui.MODES}
    pages = [Control() for _ in range(6)]
    obj.tabs = SimpleNamespace(count=lambda: len(pages), widget=pages.__getitem__)
    obj._log = obj.logs.append
    # Bind normal Python methods only; never instantiate a Qt widget.
    for name, member in vars(gui.Workbench).items():
        if name.startswith("_") and callable(member) and name not in ("__init__", "_log", "_workspace_key"):
            setattr(obj, name, MethodType(member, obj))
    obj.live_updates = []
    def observe_live(state):
        obj.live_updates.append(state)
        obj._state = state
        obj._update_live_controls()
        obj._refresh_replay_display()
    obj._on_state = observe_live
    return obj


def test_source_switch_acknowledgement_discards_queued_frames_without_replacing_live_state():
    obj = bench()
    live = {"running": True, "paused": True, "pose": [9, 8, 7, 0]}
    obj._state = live
    obj._keys.add(gui.Qt.Key.Key_W)
    obj._controller["analog"]["left_y"] = 127
    obj._replay_enter()
    assert obj._replay_pending == "enter" and not obj.scene.driving
    assert obj.engine.calls[0][0] == "set_controller"
    assert not any(obj.engine.calls[0][1][0]["analog"].values())
    assert obj.engine.calls[1] == ("replay_enter", ())
    obj._on_live_frame({"generation": 0, "pixels": "queued live"})
    obj._on_replay_state({**obj._replay, "generation": 0})
    assert obj._replay_pending == "enter" and not obj.scene.frames
    recorded = {**obj._replay, "active": True, "generation": 1, "pose": [1, 2, 3, .2],
                "camera": {"yaw": 45, "pitch": -30, "distance": 2, "target": [1, 2, 3]}}
    obj._on_replay_state(recorded)
    assert obj._state is live and obj.live_updates == [live]
    assert obj.scene.target == [1, 2, 3] and not obj.scene.cameras
    obj._on_live_frame({"generation": 1, "pixels": "wrong source"})
    obj._on_replay_frame({"generation": 0, "pixels": "old replay"})
    obj._on_replay_frame({"generation": 1, "pixels": "historical pose"})
    assert obj.scene.frames == ["historical pose"]
    obj._replay_live()
    obj._on_replay_frame({"generation": 1, "pixels": "queued replay"})
    obj._on_replay_state({**recorded, "active": False, "generation": 2,
                          "camera": {"yaw": 10, "pitch": -20, "distance": 4, "target": [0, 0, 0]}})
    obj._on_live_frame({"generation": 1, "pixels": "old live"})
    obj._on_live_frame({"generation": 2, "pixels": "paused live"})
    assert obj.scene.frames == ["historical pose", "paused live"]
    assert obj._state is live and live["paused"] and obj.scene.driving
    assert not any(name == "resume" for name, _ in obj.engine.calls)
    assert obj.scene.scene_name == "Current field"
    obj.scene.scene_name = "Newly imported field"
    obj._refresh_replay_display()
    assert obj.scene.scene_name == "Newly imported field"


@pytest.mark.parametrize("pending", [None, "enter", "live"])
def test_every_controller_and_manual_joint_path_is_inert_during_replay(monkeypatch, pending):
    obj = bench(active=True)
    obj._replay_pending = pending
    before = deepcopy(obj._controller)
    # A blocked gamepad poll must return before touching any Qt/native state.
    monkeypatch.setattr(gui, "QApplication", SimpleNamespace())
    for method, args in (("_key", (gui.Qt.Key.Key_W, True)), ("_key", (gui.Qt.Key.Key_W, False)),
                         ("_stick", ("left", 127, -127)), ("_digital", ("a", True)),
                         ("_send_controller", ()), ("_connect_gamepad", ()), ("_poll_gamepad", ()),
                         ("_move_joint", ())):
        getattr(obj, method)(*args)
    assert not obj.engine.calls and obj._controller == before and not obj._keys


@pytest.mark.parametrize("name,args", [
    ("_apply_config", ()), ("_choose_workspace", ()), ("_build", ()), ("_start", ()),
    ("_pause_resume", ()), ("_stop", ()), ("_reset", ()), ("_single_step", ()),
    ("_urdf_selected", ("missing.urdf",)), ("_accept_robot_import", ({}, {})),
    ("_edit_robot_setup", ()), ("_import_field", ("missing.json",)),
    ("_robot_preset", ("other",)), ("_game_preset", ("other",)),
    ("_load_profile", ()), ("_save_profile", ()), ("_load_demo", ())])
def test_live_mutation_handlers_cannot_bypass_disabled_controls(name, args):
    obj = bench()
    obj._replay_pending = "enter"
    getattr(obj, name)(*args)
    assert not obj.engine.calls
    assert "Return to Live" in obj.statusBar().text


@pytest.mark.parametrize("playing", [False, True])
def test_drag_coalesces_requests_keeps_thumb_local_and_resumes_only_prior_playback(playing):
    obj = bench(active=True, playing=playing)
    obj._replay_seek_started()
    for value in (.1, .2, .3, .5):
        obj._queue_replay_seek(value)
    assert obj._replay_seek_timer.starts == 1
    assert obj.engine.calls == [("replay_pause", ())]
    obj._on_replay_state({**obj._replay, "playing": False, "position_s": .1})
    assert not obj.replay_timeline.events  # Incoming frames cannot snap a held thumb backwards.
    obj._flush_replay_seek()
    assert obj.engine.calls[-1] == ("replay_seek", (.5,))
    obj._queue_replay_seek(4)
    obj._queue_replay_seek(6)
    obj._replay_seek_finished(7.25)
    expected = [("replay_pause", ()), ("replay_seek", (.5,)), ("replay_seek", (7.25,))]
    if playing:
        expected.append(("replay_play", ()))
    assert obj.engine.calls == expected and not obj._replay_seek_timer.active
    assert not obj._replay_dragging and obj._pending_replay_seek is None


def test_programmatic_updates_block_slider_signals_and_label_historical_time_distinctly():
    obj = bench(active=True, playing=True)
    obj._replay.update(position_s=9.5, mode_time=8.2, pose=[.2, -.3, .4, 0], scene_name="Recorded field")
    obj._refresh_replay_display()
    assert all(event[2] for event in obj.replay_timeline.events)
    assert not obj.replay_timeline.blocked and obj.replay_timeline.value == 9500
    assert obj.replay_time.text == "00:09.500 / 00:12.345"
    assert obj.metric_captions["rtf"].text == "PLAYBACK"
    assert obj.metric_values["remaining"].text == "8.20 s"
    assert "LIVE TELEMETRY" in obj.telemetry_caption.text and "PAUSED" in obj.telemetry_caption.text
    assert not obj.progress.visible and obj.scene.scene_name == "Recorded field"
    assert not obj.engine.calls


def test_failed_entry_unlocks_live_without_resuming_or_overwriting_pose():
    obj = bench()
    obj._replay_enter()
    obj._on_replay_state({**obj._replay, "active": False, "generation": 1, "reason": "Missing recorded mesh"})
    assert not obj._replay_blocked() and obj.scene.driving
    assert "Missing recorded mesh" in obj.statusBar().text
    assert not any(name == "resume" for name, _ in obj.engine.calls)
    assert obj.logs == ["Replay: Missing recorded mesh"]
    obj._on_replay_state(dict(obj._replay))
    assert len(obj.logs) == 1


def test_source_acknowledgement_with_default_camera_target_none_is_valid():
    obj = bench()
    obj._on_replay_state({**obj._replay, "generation": 1, "camera": {"target": None}})
    assert obj.scene.target == [0, 0, 0] and obj.replay_timeline.seek_cancelled


@pytest.mark.parametrize("condition", ["unavailable", "building", "applying"])
def test_entry_waits_for_a_retained_clip_and_idle_configuration(condition):
    obj = bench()
    if condition == "unavailable": obj._replay["available"] = False
    elif condition == "building": obj._building = True
    else: obj._pending_scene_applies = 1
    obj._replay_enter()
    assert not obj.engine.calls and obj._replay_pending is None


def test_camera_closeup_uses_recorded_pose_and_leaves_both_physics_states_untouched():
    obj = bench(active=True)
    obj._state = {"pose": [9, 8, 7, 0]}
    obj._replay["pose"] = [1, 2, 3, .5]
    before = deepcopy((obj._state, obj._replay))
    obj._camera_preset(3)
    assert obj.scene.cameras[-1][1]["target"] == [1, 2, 3]
    assert (obj._state, obj._replay) == before
    obj._replay["active"] = False
    obj._camera_preset(3)
    assert obj.scene.cameras[-1][1]["target"] == [9, 8, 7]


def test_controls_lock_entire_setup_but_leave_live_telemetry_readable():
    obj = bench(active=True)
    obj._state = {"running": True, "paused": True}
    obj._update_live_controls()
    assert not obj.pause_button.enabled and not obj.reset_button.enabled and not obj.joint_button.enabled
    assert all(not button.enabled for button in obj.mode_buttons.values())
    assert all(not obj.tabs.widget(index).enabled for index in range(5))
    assert obj.tabs.widget(5).enabled
    obj._replay["active"] = False
    obj._update_live_controls()
    assert obj.pause_button.enabled and obj.pause_button.text == "Resume"
    assert obj.step_button.enabled and all(obj.tabs.widget(i).enabled for i in range(6))


def test_close_cancels_queued_seek_without_resuming_live_or_replay():
    obj = bench(active=True)
    obj._queue_replay_seek(4)
    accepted = []
    gui.Workbench.closeEvent(obj, SimpleNamespace(accept=lambda: accepted.append(True)))
    assert accepted == [True] and obj.engine.calls == [("close", ())]
    assert not obj._replay_seek_timer.active and obj._pending_replay_seek is None


@pytest.mark.parametrize("x,reverse,expected", [(-10, False, 0), (10, False, 0), (60, False, 500),
    (110, False, 1000), (999, False, 1000), (10, True, 1000), (110, True, 0)])
def test_timeline_mapping_clamps_to_endpoints_and_respects_direction(x, reverse, expected):
    assert gui._timeline_position(x, 10, 100, 0, 1000, reverse) == expected


def test_click_and_drag_emit_ordered_scrub_events_without_widget_construction():
    calls = []
    values = {"value": 0}
    signal = lambda name: SimpleNamespace(emit=lambda *args: calls.append((name, args)))
    slider = SimpleNamespace(_seeking=False, isEnabled=lambda: True, setFocus=lambda: None,
        setSliderDown=lambda down: calls.append(("down", (down,))),
        seek_started=signal("begin"), seek_requested=signal("seek"), seek_finished=signal("finish"),
        setValue=lambda value: values.update(value=value), value=lambda: values["value"],
        _at_pointer=lambda event: event.position_ms)
    event = lambda position: SimpleNamespace(button=lambda: gui.Qt.MouseButton.LeftButton,
        position_ms=position, accept=lambda: None)
    gui.ReplayTimeline.mousePressEvent(slider, event(5000))
    gui.ReplayTimeline.mouseMoveEvent(slider, event(7000))
    gui.ReplayTimeline.mouseReleaseEvent(slider, event(7500))
    assert calls == [("down", (True,)), ("begin", ()), ("seek", (5.,)), ("seek", (7.,)),
                     ("down", (False,)), ("finish", (7.5,))]
    assert not slider._seeking


def test_test_module_does_not_create_a_qt_application():
    assert gui.QApplication.instance() is None
