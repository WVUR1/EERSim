"""The benchmark must reject false success while retaining portable evidence."""
import csv
import importlib.util
import json
from pathlib import Path

import pytest


_spec = importlib.util.spec_from_file_location(
    "integration_benchmark", Path(__file__).resolve().parents[1] / "tools" / "benchmark_integration.py")
benchmark = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(benchmark)


class FakeWorld:
    client = 18
    robot = 8
    joints = [{"name": "lift", "position": .04, "velocity": 0}]
    motor_bindings = []

    def __init__(self, config):
        self.config = config
        self.time = 1 / 240  # A prepared PhysX world already contains one real bootstrap tick.
        self.tick = 1
        self._physics = self
        self.field_objects = [{"id": 42, "name": "demo_preload"}]
        self.physics_status = {"backend": config["physics_backend"], "hardware_accelerated": False,
                               "solver_iterations": 100}
        self.closed = False

    def snapshot(self):
        return {"time": self.time, "tick": self.tick, "pose": [0, 0, .1, 0],
                "velocity": [0, 0, 0], "joints": self.joints, "motors": {},
                "scoring": {"placements": []}}

    def step(self, commands):
        self.tick += 1
        self.time = self.tick / 240
        return self.snapshot()

    def getBasePositionAndOrientation(self, body, physicsClientId):
        assert body == 42 and physicsClientId == self.client
        return [0, 0, .1 + self.tick * .001], [0, 0, 0, 1]

    def getBaseVelocity(self, body, physicsClientId):
        assert body == 42 and physicsClientId == self.client
        return [0, 0, .24], [0, 0, 0]

    def getContactPoints(self, body, other, physicsClientId):
        return []

    def _scoring(self):
        return {"placements": []}

    def close(self):
        self.closed = True


class FakeSession:
    def __init__(self, config, log):
        self.world = FakeWorld(config)
        self.config = config
        self.phase = "disabled"
        self.running = False
        self.initialized = False
        self.runtime = None
        self.elapsed = 0

    def start(self, mode, executable):
        self.phase = "initialize"
        self.running = True
        self.runtime = object()

    def step(self):
        if self.phase == "initialize":
            self.phase = "autonomous"
            self.initialized = True
        else:
            self.elapsed += 1
        state = self.world.step({})
        if self.elapsed >= round(self.config["duration_overrides"]["skills_auto"] * 240):
            self.running = False
            self.runtime = None
            self.phase = "disabled"
        return {**state, "phase": self.phase, "running": self.running, "mode_time": self.elapsed / 240}

    def close(self):
        self.world.close()


def arguments(tmp_path, *extra):
    return benchmark.parser().parse_args([
        "--backend", "bullet", "--steps", "6", "--output", str(tmp_path / "results with spaces" / "run.json"),
        *extra])


def workspace(tmp_path):
    folder = tmp_path / "external source"
    (folder / "src").mkdir(parents=True)
    (folder / "include").mkdir()
    (folder / "src/main.cpp").write_text("void autonomous() {}\n")
    (folder / "include/main.h").write_text("void autonomous();\n")
    return folder


def test_passive_fixed_steps_use_real_ids_joint_fields_and_exclude_prepared_tick(tmp_path):
    worlds = []

    def create(config):
        worlds.append(FakeWorld(config))
        return worlds[-1]

    args = arguments(tmp_path, "--no-pros")
    report = benchmark.execute(args, config={}, world_factory=create)
    assert report["status"] == "passed"
    assert report["validation_scope"] == "physics_only"
    assert report["steps"] == {"initialization": 0, "competition": 0, "passive": 6, "total": 6}
    assert report["simulation_seconds"]["total_advanced"] == .025
    assert report["simulation_seconds"]["prepared_world_time"] == pytest.approx(1 / 240)
    assert report["final_field_objects"][0]["id"] == 42
    assert report["preload"]["maximum_height_m"] == pytest.approx(.107)
    assert report["step_realtime_factor"] == pytest.approx(.025 / report["timing_seconds"]["passive_steps"])
    assert "world_construction" in report["timing_seconds"]
    assert "observation" in report["timing_seconds"]
    assert "native_build" not in report["timing_seconds"]
    rows = list(csv.DictReader(args.output.with_suffix(".csv").open()))
    assert len(rows) == 6
    assert json.loads(rows[0]["joints_json"])[0]["name"] == "lift"
    assert json.loads(args.output.read_text())["status"] == "passed"
    assert worlds[0].closed


def test_native_smoke_excludes_initialization_and_preserves_complete_source_hashes(tmp_path):
    folder = workspace(tmp_path)
    args = arguments(tmp_path)
    sessions = []

    def create(config, log):
        sessions.append(FakeSession(config, log))
        return sessions[-1]

    report = benchmark.execute(args, config={"workspace": str(folder)}, session_factory=create,
                               builder=lambda *_: folder / "native.exe")
    assert report["status"] == "passed"
    assert report["validation_scope"] == "native_smoke"
    assert report["steps"] == {"initialization": 1, "competition": 6, "passive": 0, "total": 7}
    assert report["checks"]["source_unchanged"]
    assert set(report["source_before_sha256"]) == {"src/main.cpp", "include/main.h"}
    assert not report["checks"]["full_claw_placement"]
    assert report["step_realtime_factor"] == pytest.approx(.025 / report["timing_seconds"]["competition_steps"])
    assert report["timing_seconds"]["initialization_steps"] > 0
    assert sessions[0].world.closed


def test_source_mutation_changes_success_to_failure_even_if_robot_finishes(tmp_path):
    folder = workspace(tmp_path)

    def mutate(source, log):
        (source / "include/main.h").write_text("changed\n")
        return source / "native.exe"

    report = benchmark.execute(arguments(tmp_path), config={"workspace": str(folder)},
                               session_factory=FakeSession, builder=mutate)
    assert report["status"] == "failed"
    assert not report["checks"]["source_unchanged"]
    assert "changed during this run" in report["source_error"]


def test_source_integrity_includes_final_runtime_shutdown(tmp_path):
    folder = workspace(tmp_path)

    class ShutdownSession(FakeSession):
        def close(self):
            super().close()
            (folder / "src/main.cpp").write_text("modified on shutdown\n")

    report = benchmark.execute(arguments(tmp_path), config={"workspace": str(folder)},
                               session_factory=ShutdownSession, builder=lambda *_: folder / "native.exe")
    assert report["status"] == "failed"
    assert not report["checks"]["source_unchanged"]


@pytest.mark.parametrize("problem", ["nan_joint", "wrong_clock", "wrong_backend"])
def test_bad_simulation_cannot_pass_and_evidence_survives(tmp_path, problem):
    class BadWorld(FakeWorld):
        def step(self, commands):
            state = super().step(commands)
            if problem == "nan_joint":
                state["joints"] = [{"name": "lift", "velocity": float("nan")}]
            if problem == "wrong_clock":
                state["time"] += .02
            return state

    world = BadWorld({"physics_backend": "physx_cpu" if problem == "wrong_backend" else "bullet"})
    args = arguments(tmp_path, "--no-pros")
    report = benchmark.execute(args, config={}, world_factory=lambda _: world)
    assert report["status"] == "failed"
    assert "ValidationError" in report["error"]
    assert world.closed
    assert json.loads(args.output.read_text())["status"] == "failed"
    assert args.output.with_suffix(".csv").is_file()


@pytest.mark.parametrize("problem", ["no_lift", "wrong_goal", "still_grasped", "no_completion", "route_timeout"])
def test_claw_validation_requires_all_physical_and_program_milestones(problem):
    evidence = {"lift_m": .18, "final_robot_contact": False,
                "final_scoring": {"placements": [
                    {"pin": "demo_preload", "goal": "goal_red_5", "alliance": "red"}]}}
    logs = ["Placement complete: inspect the placed-pin count"]
    final = {"running": False, "phase": "disabled"}
    benchmark.validate_claw(evidence, logs, final)
    if problem == "no_lift":
        evidence["lift_m"] = .10
    if problem == "wrong_goal":
        evidence["final_scoring"]["placements"][0]["goal"] = "goal_red_6"
    if problem == "still_grasped":
        evidence["final_robot_contact"] = True
    if problem == "no_completion":
        logs.clear()
    if problem == "route_timeout":
        logs.append("Route timeout: check pose and obstacles")
    with pytest.raises(benchmark.ValidationError):
        benchmark.validate_claw(evidence, logs, final)


def test_explicit_claw_validation_rejects_short_smoke(tmp_path):
    with pytest.raises(ValueError, match="at least 30"):
        benchmark.execute(arguments(tmp_path, "--validation", "claw"), config={})


@pytest.mark.parametrize('selected,gpu', [('physx_cpu', False), ('physx_gpu', True)])
def test_physx_runtime_engine_and_verified_device_must_match_request(selected, gpu):
    status = {'backend': 'physx', 'hardware_accelerated': gpu, 'solver_iterations': 100}
    benchmark.validate_backend(status, selected, 100)
    with pytest.raises(benchmark.ValidationError):
        benchmark.validate_backend(status, 'physx_cpu' if gpu else 'physx_gpu', 100)
    with pytest.raises(benchmark.ValidationError, match='iterations'):
        benchmark.validate_backend(status, selected, 32)
