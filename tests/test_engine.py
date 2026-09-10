"""A rejected scene must never launch robot code against a previous scene."""
import time
import copy
from pathlib import Path
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
    scene.update(game_pack=str(ASSETS / "games" / "practice.json"), renderer="cpu")
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

    class SlowSession:
        dt = 1 / 240
        renderer_status = {"device": "test renderer"}

        def __init__(self, config, log):
            self.world = self
            self.running = self.paused = False
            self.manual_joints = {}
            self.steps = 0

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
        assert any(state["steps"] == 2 and not state["paused"]
                   and state["physics_backlog_s"] > .1 for state in states), states
    finally:
        worker.close()
