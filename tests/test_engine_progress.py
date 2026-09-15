"""Operation-stage integration with fake sessions/renderers; no native scenes."""
from pathlib import Path

import pytest

from eersim import engine
from test_engine_presentation import FakeReplay, Signal, run_worker


BASE = {"physics_hz": 120, "render_fps": 30, "renderer": "cpu"}


def events(run, action):
    return [event for event in run.worker.operation.values if event["action"] == action]


def check_event_lifetimes(values):
    groups = {}
    for value in values:
        assert set(value) - {"error"} == {"id", "action", "state", "message", "fraction"}
        if "error" in value:
            assert value["state"] == "failed" and isinstance(value["error"], str)
        assert isinstance(value["id"], int) and value["id"] > 0
        assert value["action"] in engine._OperationProgress.labels
        assert isinstance(value["message"], str) and value["message"]
        assert value["fraction"] == (1.0 if value["state"] == "completed" else None)
        groups.setdefault(value["id"], []).append(value)
    assert list(groups) == list(range(1, len(groups) + 1))
    for group in groups.values():
        assert group[0]["state"] == "running"
        assert all(value["state"] == "running" for value in group[:-1])
        assert group[-1]["state"] in {"completed", "failed"}
        assert len({value["action"] for value in group}) == 1


def test_configure_and_start_stages_finish_after_first_frame_without_per_tick_events(monkeypatch):
    signals = []
    original = Signal.emit
    def emit(signal, value):
        if isinstance(value, dict) and "fraction" in value:
            signals.append(("operation", value.copy()))
        elif isinstance(value, dict) and "pixels" in value:
            signals.append(("frame", value.copy()))
        original(signal, value)
    monkeypatch.setattr(Signal, "emit", emit)
    run = run_worker(monkeypatch, ticks=40)
    check_event_lifetimes(run.worker.operation.values)
    assert not any(row["state"] == "failed" for row in run.worker.operation.values)
    assert events(run, "configure")[-1]["state"] == "completed"
    assert "continuing" in events(run, "configure")[-1]["message"]
    start = events(run, "start")
    assert start[-1]["state"] == "completed" and "ready" in start[-1]["message"]
    assert any("recording" in row["message"] for row in start)
    frame = next(index for index, event in enumerate(signals) if event[0] == "frame")
    completed = next(index for index, event in enumerate(signals)
                     if event[0] == "operation" and event[1]["action"] == "start" and event[1]["state"] == "completed")
    assert completed > frame
    count = len(run.worker.operation.values)
    longer = run_worker(monkeypatch, ticks=100)
    assert len(longer.worker.operation.values) == count
    assert run.sessions[0].steps == 40 and longer.sessions[0].steps == 100


def test_build_and_export_report_stage_before_work_then_completion(monkeypatch):
    state, native_calls = {}, []
    def build(workspace, log):
        event = state["worker"].operation.values[-1]
        native_calls.append(("build", workspace, event))
        assert event["state"] == "running" and "Compiling" in event["message"]
        log("compiler detail stays in console")
        return Path("robot.exe")
    def control(worker, sessions, timer, closing):
        if not state:
            state["worker"] = worker
            sessions[0].samples = ["one", "two"]
            def export(path):
                event = worker.operation.values[-1]
                native_calls.append(("export", path, event))
                assert event["state"] == "running" and "Writing" in event["message"]
            sessions[0].export_csv = export
            worker._send("build", "workspace")
            worker._send("export", "telemetry.csv")
        elif any(event["action"] == "export" and event["state"] == "completed" for event in worker.operation.values):
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,))], build=build, wait_hook=control)
    check_event_lifetimes(run.worker.operation.values)
    assert [row[0] for row in native_calls] == ["build", "export"]
    assert events(run, "build")[-1]["state"] == events(run, "export")[-1]["state"] == "completed"
    assert run.worker.built.values == ["robot.exe"]
    assert "compiler detail stays in console" in run.worker.log.values
    assert not any("compiler detail" in row["message"] for row in run.worker.operation.values)
    assert run.sessions[0].steps == 0


@pytest.mark.parametrize("action", ["build", "start", "reset"])
def test_operation_failure_clears_its_progress_and_next_configure_can_finish(monkeypatch, action):
    phase = [0]
    def failure(*args):
        raise RuntimeError("injected " + action + " failure")
    def control(worker, sessions, timer, closing):
        if phase[0] == 0:
            if action == "start": sessions[0].start = failure
            if action == "reset": sessions[0].reset = failure
            if action == "export": sessions[0].export_csv = failure
            worker._send(action, *(("workspace",) if action == "build" else
                                  ("skills_auto",) if action == "start" else
                                  ("telemetry.csv",) if action == "export" else ()))
            phase[0] = 1
        elif phase[0] == 1 and worker.failed.values:
            assert worker.operation.values[-1]["state"] == "failed"
            assert worker.operation.values[-1]["action"] == action
            worker.configure(BASE)
            phase[0] = 2
        elif phase[0] == 2 and len(sessions) == 2 and worker.operation.values[-1]["state"] == "completed":
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,))], build=failure,
                     wait_hook=control, expected_failures=1)
    check_event_lifetimes(run.worker.operation.values)
    failures = [row for row in events(run, action) if row["state"] == "failed"]
    assert len(failures) == 1 and failures[0]["message"] == "injected " + action + " failure"
    assert run.worker.operation.values[-1]["id"] > failures[0]["id"]
    assert not run.worker.reset_completed.values
    traces = [message for message in run.worker.log.values if message.startswith("Traceback (most recent call last):")]
    assert len(traces) == 1
    assert "RuntimeError: injected " + action + " failure" in traces[0]


def test_export_failure_ends_progress_without_stopping_run_or_recording_and_can_retry(monkeypatch):
    phase, exports, failure_tick = [0], [], []
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if phase[0] == 0 and session.steps == 0: return
        if phase[0] == 0:
            session.samples = ["sample"]
            def export(path):
                exports.append(path)
                if path == "locked.csv":
                    failure_tick.append(session.steps)
                    raise PermissionError("Destination is not writable")
            session.export_csv = export
            worker._send("export", "locked.csv")
            phase[0] = 1
        elif phase[0] == 1:
            event = worker.operation.values[-1]
            if event["action"] != "export" or event["state"] != "failed": return
            assert event["error"] == "Destination is not writable"
            assert session.running and not session.paused and session.stops == 0
            assert not worker.failed.values
            worker._send("export", "writable.csv")
            phase[0] = 2
        elif phase[0] == 2 and session.steps == 12:
            closing.set()
    run = run_worker(monkeypatch, ticks=12, wait_hook=control)
    check_event_lifetimes(run.worker.operation.values)
    assert exports == ["locked.csv", "writable.csv"]
    assert [row["state"] for row in events(run, "export") if row["state"] != "running"] == ["failed", "completed"]
    assert run.sessions[0].stops == 0 and run.sessions[0].resets == 0
    assert 0 < failure_tick[0] < run.sessions[0].steps == 12
    assert run.replays[0].events.count("begin") == run.replays[0].events.count("finish") == 1
    assert [sample["tick"] for sample in run.replays[0].captured] == list(range(1, 13))
    assert "Telemetry export failed: Destination is not writable" in run.worker.log.values


def test_engine_failure_tuning_status_is_marked_before_global_error_but_validation_is_not(monkeypatch):
    from test_engine_tuning import experiment, fake_tuners
    fake_tuners(monkeypatch)
    emitted = []
    original = Signal.emit
    def emit(signal, value):
        if isinstance(value, dict) and value.get("state") == "error":
            emitted.append(("tuning", value.copy()))
        elif value == "injected tuning step failure":
            emitted.append(("engine", value))
        original(signal, value)
    monkeypatch.setattr(Signal, "emit", emit)
    run = run_worker(monkeypatch,
                     commands=[("configure", (BASE,)), experiment(fail_step=True)],
                     wait_hook=lambda worker, sessions, timer, closing: closing.set() if worker.failed.values else None,
                     expected_failures=1)
    assert emitted[0][0] == "tuning" and emitted[0][1]["engine_failure"] is True
    assert emitted[1] == ("engine", "injected tuning step failure")
    assert run.sessions[0].steps == 0
    invalid = run_worker(monkeypatch,
                         commands=[("configure", (BASE,)), experiment(reject=True)],
                         wait_hook=lambda worker, sessions, timer, closing: closing.set())
    assert invalid.worker.tuning_status.values[-1] == {"state": "error", "message": "test target is not mapped"}


def test_configure_constructor_failure_still_emits_one_terminal_operation(monkeypatch):
    def fail(*args):
        raise RuntimeError("robot import failed")
    run = run_worker(monkeypatch, commands=[("configure", (BASE,))], session_type=fail,
                     wait_hook=lambda worker, sessions, timer, closing: closing.set(), expected_failures=1)
    check_event_lifetimes(run.worker.operation.values)
    assert events(run, "configure")[-1]["message"] == "robot import failed"
    assert not run.sessions


def test_configure_loading_does_not_publish_none_frames_or_complete_early(monkeypatch):
    phase, attempts = [0], []
    def control(worker, sessions, timer, closing):
        if phase[0] == 0:
            parent = engine.Session
            class LoadingSession(parent):
                def render(self, *args, **kwargs):
                    attempts.append(worker.operation.values[-1].copy())
                    return "live ready" if len(attempts) == 3 else None
            monkeypatch.setattr(engine, "Session", LoadingSession)
            worker.configure(BASE)
            phase[0] = 1
        elif len(attempts) == 3:
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,))], wait_hook=control)
    check_event_lifetimes(run.worker.operation.values)
    assert len(attempts) == 3 and all(row["state"] == "running" for row in attempts)
    assert [row["pixels"] for row in run.worker.frame.values] == [0, "live ready"]
    assert events(run, "configure")[-1]["state"] == "completed"
    assert all(session.steps == 0 for session in run.sessions)


def test_reset_waits_for_first_frame_and_existing_reset_acknowledgement(monkeypatch):
    phase, observations = [0], []
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if phase[0] == 0:
            original_render = session.render
            def render(*args, **kwargs):
                event = worker.operation.values[-1]
                observations.append((event["action"], event["state"], len(worker.reset_completed.values)))
                return original_render(*args, **kwargs)
            session.render = render
            worker._send("reset")
            phase[0] = 1
        elif worker.reset_completed.values:
            assert worker.operation.values[-1]["state"] == "completed"
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,))], wait_hook=control)
    check_event_lifetimes(run.worker.operation.values)
    assert observations == [("reset", "running", 0)]
    assert len(run.worker.reset_completed.values) == 1 and run.sessions[0].steps == 0


def test_reset_renderer_failure_ends_progress_without_acknowledging_reset(monkeypatch):
    phase = [0]
    def control(worker, sessions, timer, closing):
        if phase[0] == 0:
            def fail(*args, **kwargs):
                raise RuntimeError("reset frame unavailable")
            sessions[0].render = fail
            worker._send("reset")
            phase[0] = 1
        elif worker.failed.values:
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,))], wait_hook=control, expected_failures=1)
    check_event_lifetimes(run.worker.operation.values)
    assert events(run, "reset")[-1]["state"] == "failed"
    assert "reset frame unavailable" in events(run, "reset")[-1]["message"]
    assert not run.worker.reset_completed.values


def replay_fixture(monkeypatch, *, no_clip=False, fail_enter=False, fail_render=False):
    attempts = []
    def enter(replay, session, renderer):
        session.paused = True
        replay.generation += 1
        if fail_enter:
            raise RuntimeError("replay import failed")
        if no_clip:
            replay.reason = "No recorded run yet"
            return
        replay.active = True
    def render(replay, *args):
        attempts.append(len(attempts) + 1)
        if fail_render:
            replay.active = False
            replay.generation += 1  # Actual ReplayController.leave changes it.
            replay.reason = "Replay rendering failed: child exited"
            return None
        return "ready pixels" if len(attempts) >= 3 else None
    monkeypatch.setattr(FakeReplay, "enter", enter, raising=False)
    monkeypatch.setattr(FakeReplay, "render", render, raising=False)
    monkeypatch.setattr(FakeReplay, "advance", lambda *args: None, raising=False)
    return attempts


def test_async_replay_loading_remains_indeterminate_until_first_frame(monkeypatch):
    attempts = replay_fixture(monkeypatch)
    observations = []
    def control(worker, sessions, timer, closing):
        row = worker.operation.values[-1]
        if attempts and len(attempts) < 3:
            observations.append((row["state"], row["fraction"], len(worker.replay_frame.values)))
        if row["action"] == "replay_enter" and row["state"] == "completed":
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), ("replay_enter", ())], wait_hook=control)
    check_event_lifetimes(run.worker.operation.values)
    assert attempts == [1, 2, 3] and observations
    assert all(item == ("running", None, 0) for item in observations)
    assert run.worker.replay_frame.values == [{"generation": 1, "pixels": "ready pixels"}]
    assert len(events(run, "replay_enter")) == 4  # Stages only; no polling chatter.
    assert run.sessions[0].steps == 0


@pytest.mark.parametrize("option,message", [("no_clip", "No recorded run"),
                                           ("fail_enter", "replay import failed"),
                                           ("fail_render", "child exited")])
def test_replay_unavailable_or_renderer_failure_ends_progress(monkeypatch, option, message):
    replay_fixture(monkeypatch, **{option: True})
    def control(worker, sessions, timer, closing):
        if worker.operation.values[-1]["state"] == "failed": closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), ("replay_enter", ())], wait_hook=control)
    check_event_lifetimes(run.worker.operation.values)
    assert message in events(run, "replay_enter")[-1]["message"]
    assert message in events(run, "replay_enter")[-1]["error"]
    assert run.sessions[0].steps == 0


@pytest.mark.parametrize("close", [False, True])
def test_replay_loading_cancel_and_worker_close_clear_progress(monkeypatch, close):
    attempts = replay_fixture(monkeypatch)
    queued = []
    def control(worker, sessions, timer, closing):
        if attempts and not queued:
            queued.append(True)
            if close: closing.set()
            else: worker._send("replay_live")
        elif queued and worker.operation.values[-1]["state"] == "failed":
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (BASE,)), ("replay_enter", ())], wait_hook=control)
    check_event_lifetimes(run.worker.operation.values)
    assert len(attempts) == 1 and events(run, "replay_enter")[-1]["state"] == "failed"
    assert "error" not in events(run, "replay_enter")[-1]
    assert run.sessions[0].steps == 0
