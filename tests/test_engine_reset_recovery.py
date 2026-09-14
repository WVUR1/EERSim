"""A partially failed scene reset must not terminate the desktop worker."""
from types import SimpleNamespace

import pytest

from eersim.runner import Session
from test_engine_presentation import run_worker


@pytest.mark.parametrize("stop_fails,state_fails", [(True, False), (False, True), (True, True)])
def test_apply_recovers_after_reset_invalidates_stop_and_state(monkeypatch, stop_fails, state_fails):
    phase = [0]
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if phase[0] == 0 and session.steps >= 5:
            original_reset, original_close = session.reset, session.close
            def invalid_stop():
                raise RuntimeError("Cannot disable partially reset world")
            def invalid_state():
                raise RuntimeError("Cannot read partially reset world")
            def close():
                try:
                    session.stop()
                finally:
                    original_close()
            def reset():
                original_reset()
                if stop_fails:
                    session.stop = invalid_stop
                if state_fails:
                    session.state = invalid_state
                session.close = close
                raise RuntimeError("Scene import failed during reset")
            session.reset = reset
            original_failed = worker.failed.emit
            def failed(message):
                original_failed(message)
                if phase[0] == 1:
                    phase[0] = 2
                    # Stand in for the GUI receiving failure and clicking Apply.
                    worker.configure(dict(session.config))
            worker.failed.emit = failed
            phase[0] = 1
            worker._send("reset")
        elif len(sessions) == 2 and sessions[-1].renders:
            phase[0] = 3
            assert sessions[-1].renders[-1][1] == 0
            assert worker.state.values[-1]["tick"] == 0
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control, expected_failures=1)
    assert phase[0] == 3
    assert run.worker.failed.values == ["Scene import failed during reset"]
    assert run.worker.reset_completed.values == []
    assert not any("Simulator startup failed" in line for line in run.worker.log.values)
    assert len(run.sessions) == 2 and run.sessions[0].closed


def test_session_close_logs_disable_error_after_all_resources_close():
    events = []
    def stop():
        events.append("stop")
        raise RuntimeError("World disable failed")
    obj = SimpleNamespace(stop=stop, runtime=None, log=events.append,
                          world=SimpleNamespace(close=lambda: events.append("close")))
    Session.close(obj)
    assert events == ["stop", "close", "Scene disable during close: World disable failed"]


def test_session_close_keeps_failed_native_runtime_owned():
    events = []
    runtime = object()
    def stop():
        raise RuntimeError("Native runtime could not be reaped")
    obj = SimpleNamespace(stop=stop, runtime=runtime, log=events.append,
                          world=SimpleNamespace(close=lambda: events.append("world.close")))
    with pytest.raises(RuntimeError, match="Native runtime could not be reaped"):
        Session.close(obj)
    assert obj.runtime is runtime and events == ["world.close"]


def test_session_close_keeps_failed_world_owned():
    world = SimpleNamespace(worker=object())
    def close():
        raise RuntimeError("PhysX worker could not be reaped")
    world.close = close
    obj = SimpleNamespace(stop=lambda: None, runtime=None, log=lambda message: None, world=world)
    with pytest.raises(RuntimeError, match="PhysX worker could not be reaped"):
        Session.close(obj)
    assert obj.world is world and obj.world.worker is not None


def test_apply_retains_failed_cleanup_and_retries_before_another_replacement(monkeypatch):
    phase = [0]
    attempts = []
    def control(worker, sessions, timer, closing):
        if phase[0] == 0 and sessions[0].steps >= 5:
            old = sessions[0]
            original_close = old.close
            def close():
                attempts.append(len(attempts) + 1)
                if len(attempts) <= 2:
                    raise RuntimeError("Old native worker is still alive")
                original_close()
            old.close = close
            worker.configure(dict(old.config))
            phase[0] = 1
        elif phase[0] == 1 and len(sessions) == 2 and sessions[-1].renders:
            assert worker._retired_sessions == [sessions[0]]
            assert not sessions[0].closed
            worker.configure(dict(sessions[-1].config))
            phase[0] = 2
        elif phase[0] == 2 and worker.failed.values:
            assert len(sessions) == 2, "Rejected Apply must not create another native world"
            assert worker._retired_sessions == [sessions[0]]
            assert "cleanup is still pending" in worker.failed.values[-1]
            worker.configure(dict(sessions[-1].config))
            phase[0] = 3
        elif phase[0] == 3 and len(sessions) == 3 and sessions[-1].renders:
            assert worker._retired_sessions == []
            assert sessions[0].closed and sessions[1].closed
            phase[0] = 4
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control, expected_failures=1)
    assert phase[0] == 4 and attempts == [1, 2, 3]
    assert run.worker._retired_sessions == []


def test_failed_reset_can_be_retried_without_allowing_steps_on_invalid_world(monkeypatch):
    phase = [0]
    resets = []
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if phase[0] == 0 and session.steps >= 5:
            original_reset, original_state = session.reset, session.state
            def invalid_state():
                raise RuntimeError("Broken reset has no observation")
            def reset():
                original_reset()
                resets.append(True)
                if len(resets) == 1:
                    session.state = invalid_state
                    raise RuntimeError("Transient reset fixture failure")
                session.state = original_state
                session.steps = 0
            session.reset = reset
            worker._send("reset")
            phase[0] = 1
        elif phase[0] == 1 and worker.failed.values:
            steps = session.steps
            worker._send("step")
            phase[0] = ("blocked-step", steps)
        elif isinstance(phase[0], tuple) and len(worker.failed.values) == 2:
            assert session.steps == phase[0][1]
            worker._send("reset")
            phase[0] = 2
        elif phase[0] == 2 and worker.reset_completed.values:
            assert len(sessions) == 1
            assert session.steps == 0 and not session.running
            phase[0] = 3
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control, expected_failures=2)
    assert phase[0] == 3 and len(resets) == 2
    assert len(run.worker.reset_completed.values) == 1
    assert "before using simulation controls" in run.worker.failed.values[1]


def test_persistent_reset_render_failure_stops_retrying_and_apply_recovers(monkeypatch):
    phase = [0]
    waits = []
    render_attempts = []
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if phase[0] == 0 and session.steps >= 5:
            original_reset = session.reset
            def reset():
                original_reset()
                session.steps = 0
                def render(*args, **kwargs):
                    render_attempts.append(True)
                    raise RuntimeError("Persistent reset render fixture failure")
                session.render = render
            session.reset = reset
            worker._send("reset")
            phase[0] = 1
        elif phase[0] == 1 and worker.failed.values:
            waits.append(True)
            assert len(worker.failed.values) == 1
            if len(waits) >= 6:
                assert len(render_attempts) == 1
                worker.configure(dict(session.config))
                phase[0] = 2
        elif phase[0] == 2 and len(sessions) == 2 and sessions[-1].renders:
            phase[0] = 3
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control, expected_failures=1)
    assert phase[0] == 3
    assert len(render_attempts) == 1 and len(run.sessions) == 2
    assert run.worker.reset_completed.values == []
