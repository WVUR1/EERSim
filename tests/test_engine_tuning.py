"""Pure engine-worker tuning orchestration; no QObject or native scene runs."""
from copy import deepcopy

import pytest

from eersim import engine
from test_engine_presentation import FakeReplay, run_worker


BASE = {"physics_hz": 120, "render_fps": 30, "renderer": "cpu"}
TARGET = {"id": "arm", "ports": [1, -2], "axes": {"shaft": [1, 1]}}


def fake_tuners(monkeypatch):
    instances = []

    class FakeTuner:
        """Model only the public lifecycle contract, never a PID/physics model."""
        def __init__(self, sample, status):
            self.emit_sample, self.emit_status = sample, status
            self.active = self.capturing = False
            self.target = None
            self.events = []
            self.sessions = []
            self.generation = 0
            self.capture_id = None
            self.raw_step = None
            self.request = None
            self.step_calls = 0
            instances.append(self)

        def capture(self, session, enabled, target=None):
            assert not self.active
            self.capturing, self.target = bool(enabled), deepcopy(target)
            self.generation += 1
            self.capture_id = f"capture-{self.generation}"
            self.events.append(("capture", enabled, self.capture_id, session.running, session.steps))
            # The real Session invokes its observer during PROS stepping. Add
            # that behavior to the existing worker fixture, without native code.
            if not getattr(session, "_tuning_observer_fixture", False):
                original_step = session.step
                def step():
                    observed = original_step()
                    if session.motor_observer is not None:
                        session.motor_observer(observed)
                    return observed
                session.step = step
                session._tuning_observer_fixture = True
            session.motor_observer = self.observe if enabled else None
            self.emit_status({"state": "capturing" if enabled else "idle", "capture_id": self.capture_id})

        def observe(self, observed):
            self.emit_sample({"capture_id": self.capture_id, "tick": observed["tick"], "source": "simulation"})

        def experiment(self, session, request):
            if request.get("reject"):
                raise ValueError("test target is not mapped")
            session.stop()
            session.manual_joints.clear()
            session.paused = False
            session.motor_observer = None
            self.target = deepcopy(request["target"])
            self.active = self.capturing = True
            self.request = request
            self.sessions.append(session)
            self.raw_step = session.step
            self.events.append(("experiment", session.steps, session.closed))
            self.emit_status({"state": "running"})

        def step(self, session):
            assert self.active and session is self.sessions[-1]
            assert not session.running and not session.paused and not session.manual_joints
            self.step_calls += 1
            if self.request.get("fail_step"):
                raise RuntimeError("injected tuning step failure")
            if session.steps >= self.request.get("complete_after", 1000):
                # Like SimulationTuner, completion can return an observation
                # without issuing one more native integration step.
                self.cancel(session, "Experiment complete", state="completed")
                return session.state()
            observed = self.raw_step()
            self.emit_sample({"tick": observed["tick"], "source": "simulation"})
            return observed

        def cancel(self, session=None, message="Tuning stopped", *, state="cancelled"):
            self.events.append(("cancel", message, state,
                                None if session is None else session.steps,
                                None if session is None else session.closed))
            was_active = self.active
            self.active = self.capturing = False
            if session is not None:
                session.motor_observer = None
                if was_active:
                    session.paused = True
            self.emit_status({"state": state, "message": message})

    monkeypatch.setattr(engine, "SimulationTuner", FakeTuner)
    return instances


def replay_methods(monkeypatch, events):
    def enter(replay, session, renderer):
        events.append(("replay_enter", session.steps, session.paused))
        session.paused = True
        replay.active = True
        replay.generation += 1
    monkeypatch.setattr(FakeReplay, "enter", enter, raising=False)
    monkeypatch.setattr(FakeReplay, "advance", lambda *args: None, raising=False)
    monkeypatch.setattr(FakeReplay, "render", lambda *args: "replay pixels", raising=False)


def experiment(**options):
    return "tuning_experiment", ({"target": TARGET, "axis": "shaft", **options},)


def test_completed_tuning_never_steps_idle_physics_or_records_pros_replay(monkeypatch):
    tuners = fake_tuners(monkeypatch)
    idle_waits = []
    def stop_after_idle(worker, sessions, timer, closing):
        if worker.tuning_status.values and worker.tuning_status.values[-1]["state"] == "completed":
            idle_waits.append(sessions[0].steps)
            if len(idle_waits) == 20:
                closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), experiment(complete_after=3)],
                     wait_hook=stop_after_idle)
    assert run.sessions[0].steps == 3
    assert tuners[0].step_calls == 4  # The final call checks completion only.
    assert idle_waits == [3] * 20
    assert [row["tick"] for row in run.worker.tuning_sample.values] == [1, 2, 3]
    assert run.replays[0].captured == []
    assert "begin" not in run.replays[0].events
    assert run.replays[0].events.count("finish") == 1  # Finalize any prior PROS recording on entry.
    assert not tuners[0].active and run.sessions[0].paused
    assert run.worker.state.values[-1]["tuning_active"] is False
    assert run.worker.state.values[-1]["paused"] is True


@pytest.mark.parametrize("action,args", [
    ("stop", ()), ("reset", ()), ("configure", (BASE,)),
    ("replay_enter", ()), ("pause", ()), ("tuning_cancel", ()),
])
def test_active_experiment_cancels_before_lifecycle_action_without_extra_tick(monkeypatch, action, args):
    tuners = fake_tuners(monkeypatch)
    replay_events = []
    replay_methods(monkeypatch, replay_events)
    cancelled_at = []
    idle_waits = []
    def control(worker, sessions, timer, closing):
        if tuners[0].active and sessions[0].steps >= 2 and not cancelled_at:
            cancelled_at.append(sessions[0].steps)
            worker._queue.put((action, args))
        elif cancelled_at and not tuners[0].active:
            idle_waits.append(sessions[0].steps)
            if len(idle_waits) == 10:
                closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), experiment()], wait_hook=control)
    cancellations = [event for event in tuners[0].events if event[0] == "cancel"]
    assert len(cancellations) == 1
    assert cancellations[0][3:] == (cancelled_at[0], False)  # Scene still owned/open.
    assert run.sessions[0].steps == cancelled_at[0]
    assert idle_waits == [cancelled_at[0]] * 10
    assert not tuners[0].capturing and run.sessions[0].motor_observer is None
    if action == "configure":
        assert len(run.sessions) == 2 and run.sessions[1].steps == 0
    if action == "reset":
        assert run.sessions[0].resets == 1
    if action == "replay_enter":
        assert replay_events == [("replay_enter", cancelled_at[0], True)]
        assert run.replays[0].active
    assert run.replays[0].captured == []


def test_passive_capture_refreshes_after_each_start_and_keeps_native_step_order(monkeypatch):
    tuners = fake_tuners(monkeypatch)
    restarted = [False]
    def control(worker, sessions, timer, closing):
        if sessions[0].steps >= 4 and not sessions[0].running:
            if not restarted[0]:
                restarted[0] = True
                worker.start("match_auto")
            else:
                closing.set()
    commands = [("configure", (BASE,)), ("tuning_capture", (True, TARGET)), ("start", ("skills_auto",))]
    run = run_worker(monkeypatch, ticks=4, commands=commands, wait_hook=control)
    captured = [event for event in tuners[0].events if event[0] == "capture"]
    assert [(event[3], event[4]) for event in captured] == [(False, 0), (True, 0), (True, 0)]
    assert [event[2] for event in captured] == ["capture-1", "capture-2", "capture-3"]
    samples = run.worker.tuning_sample.values
    assert [row["capture_id"] for row in samples] == ["capture-2"] * 4 + ["capture-3"] * 4
    assert [row["tick"] for row in samples] == [1, 2, 3, 4] * 2
    assert [row["native_calls"] for row in run.replays[0].captured] == [1, 2, 3, 4] * 2
    assert run.sessions[0].resets == 1
    assert run.sessions[0].motor_observer is None  # Worker exit detaches capture.


@pytest.mark.parametrize("action,args", [("reset", ()), ("configure", (BASE,)), ("replay_enter", ())])
def test_passive_capture_detaches_before_scene_or_replay_change(monkeypatch, action, args):
    tuners = fake_tuners(monkeypatch)
    replay_methods(monkeypatch, [])
    commands = [("configure", (BASE,)), ("tuning_capture", (True, TARGET)), (action, args)]
    run = run_worker(monkeypatch, commands=commands,
                     wait_hook=lambda worker, sessions, timer, closing: closing.set())
    cancellations = [event for event in tuners[0].events if event[0] == "cancel"]
    assert len(cancellations) == 1 and cancellations[0][4] is False
    assert "Capture ended" in cancellations[0][1]
    assert not tuners[0].capturing
    assert run.sessions[0].motor_observer is None
    assert all(session.steps == 0 for session in run.sessions)


@pytest.mark.parametrize("invalid_scene", [True, False])
def test_tuning_rejected_without_applied_scene_or_during_replay(monkeypatch, invalid_scene):
    tuners = fake_tuners(monkeypatch)
    replay_methods(monkeypatch, [])
    commands = [] if invalid_scene else [("configure", (BASE,)), ("replay_enter", ())]
    commands.append(experiment())
    run = run_worker(monkeypatch, commands=commands,
                     wait_hook=lambda worker, sessions, timer, closing: closing.set())
    assert tuners[0].events == []
    assert run.worker.tuning_status.values[-1]["state"] == "error"
    assert "Apply a valid scene and return to Live" in run.worker.tuning_status.values[-1]["message"]
    assert all(session.steps == 0 for session in run.sessions)


def test_failed_tuning_request_is_reported_without_entering_active_state(monkeypatch):
    tuners = fake_tuners(monkeypatch)
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), experiment(reject=True)],
                     wait_hook=lambda worker, sessions, timer, closing: closing.set())
    assert run.worker.tuning_status.values[-1] == {"state": "error", "message": "test target is not mapped"}
    assert run.sessions[0].steps == run.sessions[0].stops == 0
    assert tuners[0].events == []


def test_worker_error_cancels_tuning_before_session_cleanup(monkeypatch):
    tuners = fake_tuners(monkeypatch)
    def finish_on_failure(worker, sessions, timer, closing):
        if worker.failed.values:
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), experiment(fail_step=True)],
                     wait_hook=finish_on_failure, expected_failures=1)
    assert run.worker.failed.values == ["injected tuning step failure"]
    cancellations = [event for event in tuners[0].events if event[0] == "cancel"]
    assert len(cancellations) == 1 and cancellations[0][2:] == ("error", 0, False)
    assert run.sessions[0].steps == 0 and not tuners[0].active


def test_worker_close_cancels_active_experiment_without_extra_step(monkeypatch):
    tuners = fake_tuners(monkeypatch)
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), experiment()],
                     wait_hook=lambda worker, sessions, timer, closing: closing.set())
    cancellations = [event for event in tuners[0].events if event[0] == "cancel"]
    assert len(cancellations) == 1 and cancellations[0][3:] == (0, False)
    assert not tuners[0].active and run.sessions[0].steps == 0


@pytest.mark.parametrize("reject", [False, True])
def test_running_pros_replay_finalizes_only_after_successful_experiment_before_tuning_step(monkeypatch, reject):
    tuners = fake_tuners(monkeypatch)
    events = []
    finish = FakeReplay.finish
    def finish_recording(replay, session, reason, *args):
        events.append(("finish", reason, session.steps))
        finish(replay, session, reason, *args)
    monkeypatch.setattr(FakeReplay, "finish", finish_recording)
    queued = []
    def observe_tick(worker, session):
        events.append(("tuning_tick" if tuners[0].active else "pros_tick", session.steps))
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if not queued and session.steps >= 2:
            queued.append(session.steps)
            worker._queue.put(experiment(reject=reject, complete_after=session.steps + 2))
        elif queued:
            if reject and session.steps >= queued[0] + 2:
                assert session.running
                closing.set()
            elif not reject and worker.tuning_status.values[-1]["state"] == "completed":
                closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), ("start", ("skills_auto",))],
                     wait_hook=control, step_hook=observe_tick)
    if reject:
        assert not any(event[0] in {"finish", "tuning_tick"} for event in events)
        assert [row["tick"] for row in run.replays[0].captured] == list(range(1, run.sessions[0].steps + 1))
        assert run.worker.tuning_status.values[-1]["state"] == "error"
    else:
        expected = ("finish", "stopped", queued[0])
        assert events.count(expected) == 1
        first_tuning = next(index for index, event in enumerate(events) if event[0] == "tuning_tick")
        assert events.index(expected) < first_tuning
        assert [row["tick"] for row in run.replays[0].captured] == list(range(1, queued[0] + 1))
