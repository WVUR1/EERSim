"""Reset presentation checks with no Qt application or native simulation."""
from types import MethodType

import pytest

from eersim import engine
from test_engine_presentation import FakeReplay, run_worker
from test_gui_replay import bench


@pytest.mark.parametrize("paused", [False, True])
def test_reset_acknowledges_new_view_and_replaces_queued_prereset_pixels(monkeypatch, paused):
    # Match the real ReplayController.leave generation behavior; the general
    # worker fixture normally does not model source transitions.
    def leave(replay):
        replay.events.append("leave")
        replay.active = False
        replay.generation += 1
    monkeypatch.setattr(FakeReplay, "leave", leave)
    mailbox = engine._LatestFrame()
    seen = {}
    obj = bench()
    phase = [0]

    def control(worker, sessions, timer, closing):
        session = sessions[0]
        worker._latest_frame = mailbox
        if phase[0] == 0 and session.steps >= 5:
            # The GUI has not yet taken this live frame. Reset must invalidate
            # that token and still deliver a replacement while physics is idle.
            seen["old_token"] = worker.frame.values[-1]
            seen["old_generation"] = worker.replay_state.values[-1]["generation"]
            original_reset = session.reset
            def reset(self):
                original_reset()
                self.steps = 0
                self.paused = False
                self.manual_joints.clear()
            session.reset = MethodType(reset, session)
            if paused:
                worker._send("pause")
            worker._send("reset")
            phase[0] = 1
        elif phase[0] == 1 and session.resets:
            new_token = worker.frame.values[-1]
            if new_token.get("generation", -1) <= seen["old_generation"]:
                return
            obj.engine.take_frame = mailbox.take
            obj._on_replay_state(worker.replay_state.values[-1])
            obj._on_live_frame(seen["old_token"])
            assert obj.scene.frames == []
            obj._on_live_frame(new_token)
            assert obj.scene.frames == [0]
            assert not session.running and not session.paused
            assert worker.state.values[-1]["tick"] == 0
            assert worker.state.values[-1]["physics_backlog_s"] == 0
            seen["new_generation"] = new_token["generation"]
            phase[0] = 2
            closing.set()

    run = run_worker(monkeypatch, wait_hook=control)
    assert phase[0] == 2
    assert run.sessions[0].resets == 1
    assert seen["new_generation"] > seen["old_generation"]
