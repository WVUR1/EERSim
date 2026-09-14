"""Reset feedback follows worker completion, not queued old telemetry."""
from types import MethodType

import pytest

from eersim import gui
from test_engine_presentation import run_worker
from test_gui_replay import bench


def reset_bench():
    obj = bench()
    obj.events = []
    obj._logo_event = obj.events.append
    obj.tabs.setCurrentIndex = lambda index: obj.events.append(("tab", index))
    obj._collect_config = lambda: pytest.fail("Reset must not silently apply pending edits")
    obj.config = {"start_pose": [1., 2., .1, 30.]}
    obj.pose_fields = [object() for _ in range(4)]
    return obj


def test_reset_stays_busy_until_worker_ack_even_with_old_telemetry():
    obj = reset_bench()
    obj._state = {"running": True, "paused": False, "tick": 10}
    obj._controller["analog"]["left_y"] = 127
    obj._reset()
    assert obj._reset_pending
    assert obj.reset_button.text == "Resetting…" and not obj.reset_button.enabled
    assert obj.status_badge.text == "RESETTING" and obj.scene.status == "RESETTING"
    assert obj.statusBar().text == gui.RESET_STATUS
    assert not obj.apply_button.enabled and not obj.start_button.enabled
    assert not obj.replay_button.enabled and not obj.scene.driving
    assert [name for name, _ in obj.engine.calls] == ["set_controller", "reset"]
    assert not any(obj.engine.calls[0][1][0]["analog"].values())

    # Telemetry already waiting in Qt's queue cannot acknowledge reset.
    obj._on_state({"running": False, "paused": False, "tick": 0})
    assert obj._reset_pending and not obj.reset_button.enabled
    obj._reset()
    obj._single_step()
    obj._stick("left", 127, 127)
    obj._digital("a", True)
    obj._replay_enter()
    assert [name for name, _ in obj.engine.calls] == ["set_controller", "reset"]

    state = {"running": False, "paused": False, "tick": 0, "pose": [1., 2., .1, 30.]}
    obj._on_reset_completed(state)
    assert not obj._reset_pending and obj.reset_button.enabled
    assert obj.reset_button.text == "Reset" and obj.apply_button.enabled
    assert obj._state == state and obj.scene.driving
    assert obj.statusBar().text == "Scene reset to the last applied starting state."
    assert obj.logs[-1] == obj.statusBar().text
    assert obj.config == {"start_pose": [1., 2., .1, 30.]}


def test_failed_reset_releases_busy_controls_and_keeps_error_visible():
    obj = reset_bench()
    obj._reset()
    obj._on_failed("Could not restore the scene")
    assert not obj._reset_pending
    assert obj.reset_button.enabled and obj.reset_button.text == "Reset"
    assert obj.status_badge.text == "ATTENTION"
    assert obj.statusBar().text == "Could not restore the scene"
    assert "error" in obj.events
    obj._on_reset_completed({"running": False, "tick": 0})
    assert obj.statusBar().text == "Could not restore the scene"


def test_worker_ack_contains_successful_reset_state_and_no_compiler_call(monkeypatch):
    phase = [0]
    renders_at_reset = []
    publications = []
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if phase[0] == 0 and session.steps >= 5:
            emit_frame = worker.frame.emit
            emit_ack = worker.reset_completed.emit
            worker.frame.emit = lambda state: (publications.append("frame"), emit_frame(state))
            worker.reset_completed.emit = lambda state: (publications.append("ack"), emit_ack(state))
            original = session.reset
            def reset(self):
                original()
                self.steps = 0
                self.paused = False
                renders_at_reset.append(len(self.renders))
            session.reset = MethodType(reset, session)
            worker._send("reset")
            phase[0] = 1
        elif phase[0] and worker.reset_completed.values:
            assert session.resets == 1
            assert len(session.renders) > renders_at_reset[0]
            assert session.renders[-1][1] == 0
            assert worker.frame.values[-1]["pixels"] == 0
            state = worker.reset_completed.values[0]
            assert state["tick"] == 0 and not state["running"]
            assert state["physics_backlog_s"] == state["real_time_factor"] == 0
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control)
    assert len(run.worker.reset_completed.values) == 1
    assert publications == ["frame", "ack"]


def test_worker_does_not_acknowledge_reset_that_raises(monkeypatch):
    phase = [0]
    def control(worker, sessions, timer, closing):
        if phase[0] == 0 and sessions[0].steps >= 5:
            def fail():
                raise RuntimeError("Reset fixture failed")
            sessions[0].reset = fail
            worker._send("reset")
            phase[0] = 1
        elif worker.failed.values:
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control, expected_failures=1)
    assert run.worker.failed.values == ["Reset fixture failed"]
    assert run.worker.reset_completed.values == []


@pytest.mark.parametrize("empty_frame", [False, True])
def test_failed_first_reset_render_never_acknowledges_success(monkeypatch, empty_frame):
    phase = [0]
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if phase[0] == 0 and session.steps >= 5:
            original = session.reset
            def reset(self):
                original()
                self.steps = 0
                def render(*args, **kwargs):
                    if empty_frame:
                        return None
                    raise RuntimeError("Reset renderer fixture failed")
                self.render = render
            session.reset = MethodType(reset, session)
            worker._send("reset")
            phase[0] = 1
        elif worker.failed.values:
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control, expected_failures=1)
    assert run.sessions[0].resets == 1
    assert run.worker.reset_completed.values == []
    assert "renderer" in run.worker.failed.values[0]


@pytest.mark.parametrize("applying, awaiting_state", [(1, False), (0, True)])
def test_reset_waits_for_all_scene_applies_and_their_state(applying, awaiting_state):
    obj = reset_bench()
    obj._pending_scene_applies = applying
    obj._scene_apply_ready = awaiting_state
    obj._update_live_controls()
    obj._reset()
    assert not obj.reset_button.enabled
    assert not getattr(obj, "_reset_pending", False)
    assert obj.engine.calls == []
    assert obj.statusBar().text == gui.APPLY_STATUS
