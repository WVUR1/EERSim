"""A rejected scene must never launch robot code against a previous scene."""
import time
import copy
from pathlib import Path
from types import SimpleNamespace
import pytest

from PySide6.QtCore import QCoreApplication

from eersim import bridge, engine
from eersim.config import ASSETS, default_config


@pytest.mark.parametrize("deferred", [False, True])
def test_failed_configuration_blocks_queued_start(monkeypatch, deferred):
    app = QCoreApplication.instance() or QCoreApplication([])
    created = []

    def runtime_factory(executable):
        created.append(executable)
        raise AssertionError("Robot must not launch after a rejected configuration")

    monkeypatch.setattr(bridge, "NativeRuntime", runtime_factory)
    monkeypatch.setattr(engine, "build_workspace", lambda *args: Path("fake-robot"))
    # Exercise queue/error recovery with real physics without making its deadline
    # depend on importing the current season's detailed CAD assembly.
    scene = default_config()
    scene.update(game_pack=str(ASSETS / "games" / "practice.json"), renderer="cpu", physics_backend="bullet")
    monkeypatch.setattr(engine, "default_config", lambda: copy.deepcopy(scene))
    worker = engine.SimulationEngine(defer_configuration=deferred)
    failures = []
    worker.failed.connect(failures.append)
    try:
        if deferred:
            worker.configure(scene)
        config = copy.deepcopy(scene)
        config["robot_urdf"] = "missing-robot-file.urdf"
        worker.build("unused-workspace")
        worker.configure(config)
        worker.start("skills_driver")
        deadline = time.monotonic() + 5
        while len(failures) < 2 and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(.01)
        assert len(failures) == 2, failures
        assert "scene settings" in failures[-1]
        assert not created
    finally:
        worker.close()


def test_slow_physics_services_pause_and_retains_unprocessed_time(monkeypatch):
    """An expensive contact tick must not lock controls behind sixteen ticks."""
    app = QCoreApplication.instance() or QCoreApplication([])
    states = []
    clocks, pre_pause = [], []

    class ObservedClock(engine.FixedStepClock):
        def __init__(self, hz):
            super().__init__(hz)
            self.added = 0.0
            clocks.append(self)

        def add(self, elapsed, speed=1.0):
            super().add(elapsed, speed)
            self.added += elapsed * speed

    class SlowSession:
        dt = 1 / 240
        renderer_status = {"device": "test renderer"}

        def __init__(self, config, log):
            self.world = self
            self.running = self.paused = False
            self.manual_joints = {}
            self.steps = 0

        @property
        def paused(self):
            return self._paused

        @paused.setter
        def paused(self, value):
            if value:
                # Observe retained work before Pause deliberately clears wall-time
                # backlog. Render/telemetry cadence must not decide this assertion.
                pre_pause.append((self.steps, clocks[-1].pending, clocks[-1].added))
            self._paused = value

        def start(self, mode, executable):
            self.running = True

        def step(self):
            self.steps += 1
            if self.steps == 2:
                worker.pause()
            # First contact pass leaves enough pending time to fill a batch.
            time.sleep(.15 if self.steps == 1 else .03)

        def state(self):
            return {"steps": self.steps, "paused": self.paused}

        def render(self, *args, **kwargs):
            return None

        def stop(self):
            self.running = False

        def close(self):
            self.stop()

    monkeypatch.setattr(engine, "FixedStepClock", ObservedClock)
    monkeypatch.setattr(engine, "Session", SlowSession)
    worker = engine.SimulationEngine(defer_configuration=True)
    worker.state.connect(states.append)
    try:
        worker.configure(default_config())
        worker.start("skills_driver")
        deadline = time.monotonic() + 4
        while not any(state["paused"] for state in states) and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(.005)
        assert any(state["paused"] and state["steps"] == 2 for state in states), states
        assert len(pre_pause) == 1, pre_pause
        steps, pending, credited = pre_pause[0]
        assert steps == 2 and pending > .1, pre_pause
        assert pending == pytest.approx(credited - steps * SlowSession.dt, abs=1e-10)
        assert any(state["paused"] and state["physics_backlog_s"] == 0 for state in states), states
    finally:
        worker.close()


def test_rtf_counts_completed_work_and_resets_between_lifecycle_changes(monkeypatch):
    """Startup/paused time and previous runs must not dilute measured throughput."""
    app = QCoreApplication.instance() or QCoreApplication([])
    states = []

    class Clock:
        now = 100.0

        def advance(self, seconds):
            self.now += seconds

    clock = Clock()

    class TimedSession:
        dt = 1 / 240
        renderer_status = {"device": "test renderer"}

        def __init__(self, config, log):
            clock.advance(3)  # Scene loading is not simulated time.
            self.world = self
            self.running = self.paused = False
            self.manual_joints = {}
            self.steps = 0

        def start(self, mode, executable):
            clock.advance(5)  # Reset/initialization cost precedes tick pacing.
            self.running, self.paused = True, False

        def step(self):
            clock.advance(.25)
            self.steps += 1
            worker.pause()  # Observe the completed tick, then service pause.

        def state(self):
            return {"steps": self.steps, "running": self.running, "paused": self.paused}

        def render(self, *args, **kwargs):
            return 0  # A successful reset must publish a first frame before acknowledgment.

        def reset(self):
            clock.advance(7)
            self.running = self.paused = False
            self.steps = 0

        def stop(self):
            self.running = self.paused = False

        def close(self):
            self.stop()

    def wait_for(predicate):
        deadline = time.monotonic() + 3
        while not predicate() and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(.002)
        assert predicate(), states

    # Patch only the engine's module reference; Qt and test deadlines use real time.
    monkeypatch.setattr(engine, "time", SimpleNamespace(perf_counter=lambda: clock.now))
    monkeypatch.setattr(engine, "Session", TimedSession)
    worker = engine.SimulationEngine(defer_configuration=True)
    worker.state.connect(states.append)
    try:
        worker.configure(default_config())
        worker.start("skills_driver")
        wait_for(lambda: states and states[-1]["running"])
        assert states[-1]["real_time_factor"] == 0

        for completed_steps in (1, 2):
            if completed_steps == 2:
                clock.advance(20)  # Paused wall time cannot enter the next sample.
                worker.resume()
                wait_for(lambda: not states[-1]["paused"])
                assert states[-1]["real_time_factor"] == 0
            clock.advance(.5)
            wait_for(lambda: states[-1]["paused"] and states[-1]["steps"] == completed_steps)
            completed = next(state for state in reversed(states)
                             if state["steps"] == completed_steps and not state["paused"])
            # One tick completed after 0.5 s pacing plus its own 0.25 s work.
            assert completed["real_time_factor"] == pytest.approx((1 / 240) / .75)
            assert completed["physics_backlog_s"] == pytest.approx(.5 - 1 / 240)
            assert states[-1]["real_time_factor"] == 0

        worker.reset()
        wait_for(lambda: states[-1]["steps"] == 0 and not states[-1]["running"])
        assert states[-1]["real_time_factor"] == 0
    finally:
        worker.close()
