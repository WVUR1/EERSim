"""Worker shutdown must finish before its owned scene can be discarded."""
from pathlib import Path

import pytest

from eersim.physx_worker import PhysXProcess


class FakeProcess:
    def __init__(self, exits_at="graceful", *, alive=True, stop_errors=()):
        self.exits_at, self.alive = exits_at, alive
        self.stage = "graceful"
        self.events = []
        self.stop_errors = set(stop_errors)

    def is_alive(self):
        return self.alive

    def join(self, timeout):
        assert 0 <= timeout <= 5, "Worker shutdown waits must be bounded"
        self.events.append(("join", self.stage, timeout))
        if self.stage == self.exits_at:
            self.alive = False

    def terminate(self):
        self._stop("terminate")

    def kill(self):
        self._stop("kill")

    def _stop(self, kind):
        self.events.append((kind,))
        self.stage = kind
        if kind in self.stop_errors:
            raise OSError(f"Synthetic {kind} failure")


class FakeConnection:
    def __init__(self, send_error=None):
        self.send_error = send_error
        self.messages = []
        self.closed = False

    def send(self, message):
        assert not self.closed
        self.messages.append(message)
        if self.send_error is not None:
            raise self.send_error("Synthetic pipe failure")

    def close(self):
        self.closed = True


def worker(exits_at="graceful", *, alive=True, failed=False, send_error=None, stop_errors=()):
    result = object.__new__(PhysXProcess)
    result.process = FakeProcess(exits_at, alive=alive, stop_errors=stop_errors)
    result.connection = FakeConnection(send_error)
    result.failed = failed
    result.log_path = Path("owned-scene/runtime/physics.log")
    return result


@pytest.mark.parametrize("exits_at,expected", [
    ("graceful", []), ("terminate", ["terminate"]), ("kill", ["terminate", "kill"]),
])
def test_shutdown_escalates_only_as_needed_and_is_repeatable(exits_at, expected):
    target = worker(exits_at)
    process, connection = target.process, target.connection
    target.close()
    assert not process.is_alive() and connection.closed
    assert target.process is process and target.connection is connection
    assert connection.messages == [("close", None)]
    assert [event[0] for event in process.events if event[0] != "join"] == expected
    events = list(process.events)
    target.close()
    assert process.events == events and connection.messages == [("close", None)]


def test_already_dead_worker_needs_no_control_message():
    target = worker(alive=False)
    target.close()
    assert target.connection.closed and not target.connection.messages
    assert not target.process.events


def test_uninitialized_process_close_is_noop():
    target = object.__new__(PhysXProcess)
    target.close()


def test_failed_request_skips_potentially_blocked_control_channel():
    target = worker("terminate", failed=True)
    target.close()
    assert not target.connection.messages
    assert target.process.events == [("terminate",), ("join", "terminate", 5)]
    assert not target.process.alive and target.connection.closed


@pytest.mark.parametrize("error", [EOFError, BrokenPipeError, OSError, ValueError])
def test_close_pipe_failure_still_stops_worker(error):
    target = worker("terminate", send_error=error)
    target.close()
    assert not target.process.alive and target.connection.closed
    assert target.process.events == [("terminate",), ("join", "terminate", 5)]


def test_terminate_os_error_still_attempts_kill():
    target = worker("kill", stop_errors=("terminate",))
    target.close()
    assert not target.process.alive and target.connection.closed
    assert ("kill",) in target.process.events


@pytest.mark.parametrize("stop_errors", [(), ("terminate", "kill")])
def test_surviving_worker_raises_and_retains_handles_for_retry(stop_errors):
    target = worker("never", stop_errors=stop_errors)
    process, connection = target.process, target.connection
    with pytest.raises(RuntimeError, match="did not exit after terminate and kill") as failure:
        target.close()
    assert str(target.log_path) in str(failure.value)
    assert target.failed and process.alive and not connection.closed
    assert target.process is process and target.connection is connection
    assert ("terminate",) in process.events and ("kill",) in process.events
    process.exits_at, process.stop_errors = "terminate", set()
    target.close()
    assert not process.alive and connection.closed
    assert connection.messages == [("close", None)], "Retry must skip the failed channel"


def scene_world(tmp_path, monkeypatch, target):
    from eersim import physx_world
    scene = tmp_path / "physics" / "scene-owned"
    scene.mkdir(parents=True)
    (scene / "physics.log").write_text("Preserve this diagnostic evidence")
    monkeypatch.setattr(physx_world, "user_data_dir", lambda: tmp_path)
    world = object.__new__(physx_world.PhysXWorld)
    world._process = target
    world._physics_directory = scene
    world._physics = object.__new__(physx_world.PhysXAdapter)
    return world, scene


def test_scene_cleanup_waits_for_confirmed_worker_exit(tmp_path, monkeypatch):
    target = worker("kill")
    world, scene = scene_world(tmp_path, monkeypatch, target)
    world._release_physics()
    assert not target.process.alive
    assert world._process is None and world._physics_directory is None
    assert not scene.exists()


def test_scene_handle_and_directory_survive_unstoppable_worker(tmp_path, monkeypatch):
    target = worker("never")
    world, scene = scene_world(tmp_path, monkeypatch, target)
    with pytest.raises(RuntimeError, match="did not exit"):
        world._release_physics()
    assert world._process is target and world._physics_directory == scene
    assert scene.is_dir() and (scene / "physics.log").is_file()
    assert target.process.alive and not target.connection.closed
    target.process.exits_at = "terminate"
    world._release_physics()
    assert world._process is None and world._physics_directory is None
    assert not target.process.alive and target.connection.closed
    assert scene.is_dir(), "Failed shutdown diagnostics must survive the successful retry"
