"""Pure worker-loop tests: no QObject, application, native scene or thread."""
from copy import deepcopy
import queue
from types import MethodType, SimpleNamespace

import pytest

from eersim import engine
from eersim.runner import FixedStepClock


class Signal:
    def __init__(self): self.values = []
    def emit(self, value): self.values.append(deepcopy(value))


class FakeTime:
    def __init__(self): self.now = 100.
    def perf_counter(self): return self.now
    def advance(self, value): self.now += value


class FakeReplay:
    active = False
    generation = 0
    cursor = None

    def __init__(self, log):
        self.captured = []
        self.events = []
        self.reason = ""
    def metadata(self): return {"active": self.active, "generation": self.generation}
    def capture(self, session, state): self.captured.append(deepcopy(state))
    def finish(self, *args): self.events.append("finish")
    def begin(self, *args): self.events.append("begin")
    def leave(self): self.events.append("leave"); self.active = False
    def discard(self): self.events.append("discard")
    def pause(self): pass
    def close(self): self.events.append("close")


def run_worker(monkeypatch, *, fps=30, tick_cost=.02, render_cost=.003, ticks=40,
               commands=None, build=None, session_type=None, wait_hook=None, expected_failures=0,
               physics_hz=120, step_hook=None):
    timer = FakeTime()
    sessions, replays, clocks, boundaries = [], [], [], []
    base = {"physics_hz": physics_hz, "render_fps": fps, "renderer": "cpu"}
    worker = SimpleNamespace(_defer_configuration=True, _latest_frame=None, _queue=queue.Queue())
    for name in ("frame", "state", "replay_frame", "replay_state", "log", "built", "failed", "reset_completed", "tuning_sample", "tuning_status", "operation"):
        setattr(worker, name, Signal())
    for name in ("_send", "_emit_frame", "pause", "start", "configure", "build"):
        setattr(worker, name, MethodType(getattr(engine.SimulationEngine, name), worker))

    class Closing:
        closed = False
        waits = 0
        def is_set(self): return self.closed
        def set(self): self.closed = True
        def wait(self, duration):
            self.waits += 1
            assert self.waits <= 4000, "Mock worker did not terminate"
            if sessions and sessions[-1].running and not sessions[-1].paused:
                session = sessions[-1]
                boundaries.append((session.steps, clocks[-1].pending,
                                   clocks[-1].added - session.steps * session.dt))
            timer.advance(duration)
            if wait_hook:
                wait_hook(worker, sessions, timer, self)
            elif sessions and sessions[-1].steps >= ticks:
                self.closed = True
    worker._closing = Closing()

    class Clock(FixedStepClock):
        def __init__(self, hz):
            super().__init__(hz); self.added = 0.; clocks.append(self)
        def add(self, elapsed, speed=1):
            self.added += elapsed * speed
            super().add(elapsed, speed)

    class Session:
        def __init__(self, config, log):
            self.config = deepcopy(config)
            self.world = self
            self.running = self.paused = False
            self.manual_joints = {}
            self.controller = {}
            self.renderer_status = {"backend": "cpu", "device": "mock"}
            self.steps = self.resets = self.stops = 0
            self.pristine = True
            self.dt = 1 / config["physics_hz"]
            self.renders = []
            self.conservation = []
            self.closed = False
            self.executable = None
            sessions.append(self)
        def start(self, mode, executable):
            if not self.pristine: self.resets += 1
            self.pristine = False
            self.running, self.paused = True, False
            self.steps = 0
        def step(self):
            self.steps += 1
            if step_hook: step_hook(worker, self)
            timer.advance(tick_cost(self.steps) if callable(tick_cost) else tick_cost)
            if self.steps >= ticks: self.running = False
            return {"tick": self.steps, "native_calls": self.steps, "pose": [self.steps / 100, 0, 0]}
        def state(self):
            if self.running:
                self.conservation.append((clocks[-1].pending, clocks[-1].added - self.steps * self.dt))
            return {"tick": self.steps, "running": self.running, "paused": self.paused}
        def render(self, *args, **kwargs):
            self.renders.append((timer.now, self.steps))
            timer.advance(render_cost)
            return self.steps
        def stop(self):
            self.stops += 1
            self.running = self.paused = False
        def close(self): self.closed = True
        def reset(self): self.resets += 1; self.running = False; self.pristine = True

    def make_replay(log):
        replay = FakeReplay(log); replays.append(replay); return replay
    monkeypatch.setattr(engine, "time", timer)
    monkeypatch.setattr(engine, "Session", session_type or Session)
    monkeypatch.setattr(engine, "ReplayController", make_replay)
    monkeypatch.setattr(engine, "FixedStepClock", Clock)
    monkeypatch.setattr(engine, "default_config", lambda: deepcopy(base))
    monkeypatch.setattr(engine, "build_workspace", build or (lambda *args: pytest.fail("Unexpected compiler call")))
    for action, args in commands or [("configure", (base,)), ("start", ("skills_auto",))]:
        worker._queue.put((action, args))
    engine.SimulationEngine._work(worker)
    assert len(worker.failed.values) == expected_failures, worker.failed.values
    assert all(item.closed for item in sessions)
    return SimpleNamespace(worker=worker, sessions=sessions, replays=replays, timer=timer,
                           clocks=clocks, boundaries=boundaries)


def test_overload_renders_regularly_without_losing_ticks_native_calls_or_replay_samples(monkeypatch):
    run = run_worker(monkeypatch)
    session = run.sessions[0]
    assert len(session.renders) >= 15
    intervals = [b[0] - a[0] for a, b in zip(session.renders, session.renders[1:])]
    assert max(intervals) < .055  # Old overload veto waited more than .25 s.
    assert max(value["physics_backlog_s"] for value in run.worker.state.values) > .1
    assert session.steps == 40
    assert [sample["tick"] for sample in run.replays[0].captured] == list(range(1, 41))
    assert [sample["native_calls"] for sample in run.replays[0].captured] == list(range(1, 41))
    for actual, expected in session.conservation:
        assert actual == pytest.approx(expected, abs=1e-10)


def test_slow_renderer_does_not_starve_physics_or_add_an_extra_frame_period(monkeypatch):
    run = run_worker(monkeypatch, tick_cost=.001, render_cost=.08, ticks=12)
    renders = run.sessions[0].renders
    assert [tick for _, tick in renders] == list(range(13))
    assert all(.081 <= b[0] - a[0] < .09 for a, b in zip(renders, renders[1:]))
    assert len(run.replays[0].captured) == 12


def test_frame_rate_changes_do_not_change_completed_physics_sample_order(monkeypatch):
    slow = run_worker(monkeypatch, fps=4)
    fast = run_worker(monkeypatch, fps=60)
    assert slow.replays[0].captured == fast.replays[0].captured
    assert len(fast.sessions[0].renders) > len(slow.sessions[0].renders)


def test_pause_keeps_completed_work_and_can_be_serviced_under_overload(monkeypatch):
    phase = [0]
    def control(worker, sessions, timer, closing):
        if not sessions: return
        session = sessions[0]
        if phase[0] == 0 and session.steps >= 5:
            phase[0] = session.steps
            worker.pause()
        elif phase[0] and session.paused:
            assert session.steps == phase[0]
            assert worker.state.values[-1]["physics_backlog_s"] == 0
            assert worker.state.values[-1]["real_time_factor"] == 0
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control)
    assert len(run.replays[0].captured) == phase[0]


def test_two_slow_ticks_return_backlog_before_pause_even_between_telemetry_samples(monkeypatch):
    def pause_on_second_step(worker, session):
        if session.steps == 2: worker.pause()

    def stop_when_paused(worker, sessions, timer, closing):
        if sessions and sessions[0].paused: closing.set()

    run = run_worker(monkeypatch, physics_hz=240,
                     tick_cost=lambda step: .15 if step == 1 else .03, render_cost=0,
                     step_hook=pause_on_second_step, wait_hook=stop_when_paused)
    retained = [row for row in run.boundaries if row[0] == 2]
    assert len(retained) == 1
    _, pending, credited_less_completed = retained[0]
    assert pending > .1
    assert pending == pytest.approx(credited_less_completed, abs=1e-10)
    assert [sample["native_calls"] for sample in run.replays[0].captured] == [1, 2]
    assert run.sessions[0].steps == 2
    assert run.worker.state.values[-1]["paused"]
    assert run.worker.state.values[-1]["physics_backlog_s"] == 0


@pytest.mark.parametrize("ticks", [3, 10, 40])
def test_completed_run_publishes_terminal_state_before_final_render_clears_dirty(monkeypatch, ticks):
    idle_passes = [0]
    def finish_after_idle(worker, sessions, timer, closing):
        if sessions and not sessions[0].running and sessions[0].steps == ticks:
            idle_passes[0] += 1
            if idle_passes[0] == 20: closing.set()
    run = run_worker(monkeypatch, ticks=ticks, wait_hook=finish_after_idle)
    final = run.worker.state.values[-1]
    assert final["tick"] == ticks and not final["running"]
    assert run.sessions[0].renders[-1][1] == ticks
    assert [row["native_calls"] for row in run.replays[0].captured] == list(range(1, ticks + 1))
    assert run.replays[0].events.count("finish") == 1


def test_stop_between_periodic_samples_publishes_disabled_state_without_another_tick(monkeypatch):
    stopped_at = []
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if session.running and session.steps >= 3 and not stopped_at:
            stopped_at.append(session.steps)
            worker._queue.put(("stop", ()))
        elif stopped_at and not session.running:
            assert worker.state.values[-1]["tick"] == stopped_at[0]
            assert not worker.state.values[-1]["running"]
            assert worker.state.values[-1]["physics_backlog_s"] == 0
            closing.set()
    run = run_worker(monkeypatch, wait_hook=control)
    assert len(run.replays[0].captured) == stopped_at[0]


def test_manual_single_step_publishes_paused_state_without_starting_robot_code(monkeypatch):
    base = {"physics_hz": 120, "render_fps": 30, "renderer": "cpu"}
    run = run_worker(monkeypatch, ticks=1, commands=[("configure", (base,)), ("step", ())])
    final = run.worker.state.values[-1]
    assert final["tick"] == 1 and final["paused"] and not final["running"]
    assert final["physics_backlog_s"] == 0
    assert "begin" not in run.replays[0].events
    assert len(run.replays[0].captured) == 1


def test_prepared_start_reuses_session_but_repeated_start_resets_advanced_world(monkeypatch):
    base = {"physics_hz": 120, "render_fps": 30, "renderer": "cpu"}
    phase = [0]
    def control(worker, sessions, timer, closing):
        if not sessions: return
        if phase[0] == 0 and sessions[0].steps >= 2:
            worker.configure({**base, "camera": {"yaw": 5}}, reuse_unchanged=True)
            worker.start("match_auto")
            phase[0] = 1
        elif phase[0] and sessions[0].resets == 1:
            closing.set()
    run = run_worker(monkeypatch, commands=[("configure", (base,)), ("configure", (base, True)),
                                           ("start", ("skills_auto",))], wait_hook=control)
    assert len(run.sessions) == 1 and run.sessions[0].resets == 1
    assert run.sessions[0].config == base  # Engine presentation changes do not mutate the world.
    assert run.worker.log.values.count("Scene configuration applied") == 3


def test_explicit_apply_reloads_unchanged_assets_and_physical_changes(monkeypatch):
    base = {"physics_hz": 120, "render_fps": 30, "renderer": "cpu"}
    commands = [("configure", (base,)), ("configure", (base,)),
                ("configure", ({**base, "gravity": 4}, True))]
    run = run_worker(monkeypatch, commands=commands,
                     wait_hook=lambda worker, sessions, timer, closing: closing.set())
    assert len(run.sessions) == 3


def test_failed_presentation_validation_blocks_a_queued_start_until_valid_apply(monkeypatch):
    base = {"physics_hz": 120, "render_fps": 30, "renderer": "cpu"}
    commands = [("configure", (base,)), ("configure", ({**base, "speed": 0}, True)),
                ("start", ("skills_auto",))]
    def control(worker, sessions, timer, closing):
        if len(worker.failed.values) == 2: closing.set()
    run = run_worker(monkeypatch, commands=commands, wait_hook=control, expected_failures=2)
    assert "speed must be positive" in run.worker.failed.values[0]
    assert "Fix and apply" in run.worker.failed.values[1]
    assert run.sessions[0].steps == 0 and "begin" not in run.replays[0].events


def test_presentation_apply_keeps_active_run_backlog_and_recording(monkeypatch):
    phase = [False]
    before = {}
    def control(worker, sessions, timer, closing):
        session = sessions[0]
        if not phase[0] and session.steps >= 5:
            phase[0] = True
            before["config"] = deepcopy(session.config)
            worker.configure({**session.config, "render_fps": 60, "speed": 1.1})
        if session.steps >= 10: closing.set()
    run = run_worker(monkeypatch, wait_hook=control)
    assert len(run.sessions) == 1
    assert run.sessions[0].config == before["config"]
    assert run.sessions[0].stops == 0
    assert run.replays[0].events.count("discard") == 2  # Initial configure and start only.
    assert [x["tick"] for x in run.replays[0].captured] == list(range(1, run.sessions[0].steps + 1))
    assert any(x["physics_backlog_s"] > .05 for x in run.worker.state.values)


@pytest.mark.parametrize("change", [{"physics_hz": 240}, {"renderer": "gpu"},
                                   {"workspace": "new"}, {"duration_overrides": {"match_auto": 3}},
                                   {"physics_cpu_threads": 0}, {"noise": {"dropout": .5}}])
def test_only_explicitly_safe_config_differences_are_reused(change):
    assert not engine._reuse_configuration({}, change, reuse_unchanged=True)


@pytest.mark.parametrize("key,value", [("speed", 0), ("speed", float("nan")),
                                      ("render_fps", -1), ("render_fps", float("inf"))])
def test_fast_presentation_path_validates_pacing(key, value):
    with pytest.raises(ValueError): engine._reuse_configuration({}, {key: value}, reuse_unchanged=True)


def test_latest_frame_keeps_one_pixel_packet_and_rejects_duplicate_and_old_source_tokens():
    mailbox = engine._LatestFrame()
    first = mailbox.publish("live", 0, "initial")
    for index in range(1000): assert mailbox.publish("live", 0, index) is None
    assert mailbox.take(first) == {"generation": 0, "pixels": 999}
    following = mailbox.publish("live", 0, "following")
    assert mailbox.take(first) is None
    replay = mailbox.publish("replay", 1, "recorded")
    assert mailbox.take(following) is None
    assert mailbox.take(replay) == {"generation": 1, "pixels": "recorded"}
    assert mailbox.take(replay) is None
    assert mailbox._packet is None


def test_default_frame_signal_contract_remains_full_pixel_packets():
    facade = SimpleNamespace(_latest_frame=None, frame=Signal(), replay_frame=Signal())
    engine.SimulationEngine._emit_frame(facade, "live", 1, "pixels")
    assert facade.frame.values == [{"generation": 1, "pixels": "pixels"}]
    engine.SimulationEngine._emit_frame(facade, "replay", 2, "recorded")
    assert facade.replay_frame.values == [{"generation": 2, "pixels": "recorded"}]


def test_render_cadence_measures_completed_frames_and_expires_old_history():
    cadence = engine._RenderCadence()
    assert cadence.fps(1) == 0
    for when in (1., 1.04, 1.08): cadence.record(when)
    assert cadence.fps(1.08) == pytest.approx(25)
    assert cadence.fps(2.1) == 0
    cadence.record(3); cadence.record(3.1)
    cadence.reset()
    assert cadence.fps(3.1) == 0
