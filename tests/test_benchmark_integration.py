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
        self.hz = config.get("physics_hz", 240)
        self.time = 1 / self.hz  # A prepared PhysX world already contains one real bootstrap tick.
        self.tick = 1
        self._physics = self
        self.field_objects = [{"id": 42, "name": "demo_preload"}]
        self.physics_status = {"backend": config["physics_backend"], "hardware_accelerated": False,
                               "solver_iterations": config.get("physics_solver_iterations", 100)}
        self.closed = False

    def snapshot(self):
        return {"time": self.time, "tick": self.tick, "pose": [0, 0, .1, 0],
                "velocity": [0, 0, 0], "joints": self.joints, "motors": {},
                "scoring": {"placements": []}}

    def step(self, commands):
        self.tick += 1
        self.time = self.tick / self.hz
        return self.snapshot()

    def getBasePositionAndOrientation(self, body, physicsClientId):
        assert body == 42 and physicsClientId == self.client
        return [0, 0, .1 + self.tick * .001], [0, 0, 0, 1]

    def getBaseVelocity(self, body, physicsClientId):
        assert body == 42 and physicsClientId == self.client
        return [0, 0, .001 * self.hz], [0, 0, 0]

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
        if self.elapsed >= round(self.config["duration_overrides"]["skills_auto"] * self.world.hz):
            self.running = False
            self.runtime = None
            self.phase = "disabled"
        return {**state, "phase": self.phase, "running": self.running, "mode_time": self.elapsed / self.world.hz}

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


@pytest.mark.parametrize("hz", [120, 240])
def test_passive_fixed_steps_use_real_ids_joint_fields_and_exclude_prepared_tick(tmp_path, hz):
    worlds = []

    def create(config):
        worlds.append(FakeWorld(config))
        return worlds[-1]

    args = arguments(tmp_path, "--no-pros", "--physics-hz", str(hz))
    report = benchmark.execute(args, config={}, world_factory=create)
    assert report["status"] == "passed"
    assert report["validation_scope"] == "physics_only"
    assert report["steps"] == {"initialization": 0, "competition": 0, "passive": 6, "total": 6}
    assert report["simulation_seconds"]["total_advanced"] == 6 / hz
    assert report["simulation_seconds"]["prepared_world_time"] == pytest.approx(1 / hz)
    assert report["final_field_objects"][0]["id"] == 42
    assert report["preload"]["maximum_height_m"] == pytest.approx(.107)
    assert report["step_realtime_factor"] == pytest.approx((6 / hz) / report["timing_seconds"]["passive_steps"])
    assert "world_construction" in report["timing_seconds"]
    assert "observation" in report["timing_seconds"]
    assert "native_build" not in report["timing_seconds"]
    rows = list(csv.DictReader(args.output.with_suffix(".csv").open()))
    assert len(rows) == 6
    assert json.loads(rows[0]["joints_json"])[0]["name"] == "lift"
    assert json.loads(args.output.read_text())["status"] == "passed"
    assert worlds[0].closed


@pytest.mark.parametrize("hz", [120, 240])
def test_native_smoke_excludes_initialization_and_preserves_complete_source_hashes(tmp_path, hz):
    folder = workspace(tmp_path)
    args = arguments(tmp_path, "--physics-hz", str(hz))
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
    assert report["step_realtime_factor"] == pytest.approx((6 / hz) / report["timing_seconds"]["competition_steps"])
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


def test_120_hz_full_mode_uses_3600_competition_ticks_and_existing_performance_snapshots(tmp_path):
    from eersim.performance import PerformanceRecorder

    class Worker:
        failed = False

        def __init__(self):
            self.performance = PerformanceRecorder(True)
            self.native = PerformanceRecorder(True)
            self.calls = []
            self.native.increment("worker.bootstrap_steps")

        def call(self, command, payload):
            assert command == "diagnostics" and payload == {"reset": False}
            self.calls.append(command)
            return {"performance": self.native.snapshot()}

    sessions = []

    class CompleteSession(FakeSession):
        def __init__(self, config, log):
            super().__init__(config, log)
            self.log = log
            self.performance = PerformanceRecorder(True)
            self.world.performance = PerformanceRecorder(True)
            self.world._process = Worker()
            self.world._scoring = lambda: {"placements": [
                {"pin": "demo_preload", "goal": "goal_red_5", "alliance": "red"}]}
            sessions.append(self)

        def step(self):
            self.performance.increment("session.ticks")
            self.world.performance.increment("world.steps")
            self.world._process.native.increment("worker.steps")
            result = super().step()
            if not self.running:
                self.log("Placement complete")
            return result

    folder = workspace(tmp_path)
    args = benchmark.parser().parse_args(["--backend", "bullet", "--physics-hz", "120",
        "--iterations", "32", "--seconds", "30", "--validation", "claw",
        "--output", str(tmp_path / "complete.json")])
    report = benchmark.execute(args, config={"workspace": str(folder)}, session_factory=CompleteSession,
                               builder=lambda *_: folder / "native")
    assert report["status"] == "passed", report.get("error")
    assert report["request"]["physics_hz"] == 120 and report["request"]["ticks"] == 3600
    assert report["steps"] == {"initialization": 1, "competition": 3600, "passive": 0, "total": 3601}
    assert report["simulation_seconds"]["competition"] == 30
    assert report["simulation_seconds"]["initialization"] == pytest.approx(1 / 120)
    assert report["simulation_seconds"]["prepared_world_time"] == pytest.approx(1 / 120)
    assert report["final_state"]["time"] == pytest.approx(3602 / 120)
    assert report["final_state"]["mode_time"] == 30 and not report["final_state"]["running"]
    assert report["checks"]["full_claw_placement"] and report["checks"]["source_unchanged"]
    assert report["step_realtime_factor"] == pytest.approx(30 / report["timing_seconds"]["competition_steps"])
    assert report["performance"]["worker"]["counters"] == {"worker.bootstrap_steps": 1, "worker.steps": 3601}
    assert report["performance"]["session"]["counters"]["session.ticks"] == 3601
    assert report["performance"]["world"]["counters"]["world.steps"] == 3601
    assert sessions[0].world._process.calls == ["diagnostics"]
    assert sessions[0].world.closed
    with args.output.with_suffix(".csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 3601 and float(rows[-1]["mode_time"]) == 30


def test_execute_retains_240_hz_default_for_older_programmatic_arguments(tmp_path):
    args = arguments(tmp_path, "--no-pros")
    del args.physics_hz
    report = benchmark.execute(args, config={}, world_factory=FakeWorld)
    assert report["status"] == "passed"
    assert report["request"]["physics_hz"] == 240
    assert report["simulation_seconds"]["total_advanced"] == .025


def test_periodic_native_activity_separates_setup_initialization_and_step_timing(tmp_path, monkeypatch):
    import os
    from eersim.performance import PerformanceRecorder
    # Deterministic synthetic clock: native stepping costs1ms; each requested
    # activity query costs100ms. No waiting or physics implementation is used.
    clock = [0.0]
    monkeypatch.setattr(benchmark.time, 'perf_counter', lambda: clock[0])
    monkeypatch.setenv('EERSIM_PROFILE', 'previous-value')

    class Worker:
        failed = False
        def __init__(self, owner):
            self.owner = owner
            self.performance = PerformanceRecorder(False)
            self.native = PerformanceRecorder(False)
            self.calls = []
        def call(self, command, payload):
            assert command == 'diagnostics'
            self.calls.append((self.owner.elapsed, dict(payload)))
            response = {'performance': self.native.snapshot()}
            if payload.get('activity'):
                clock[0] += .1
                response['scene_activity'] = {'counts': {'regular_sleeping': 2}, 'actors': []}
            return response

    sessions = []
    class InstrumentedSession(FakeSession):
        def __init__(self, config, log):
            assert os.environ['EERSIM_PROFILE'] == '1'
            super().__init__(config, log)
            self.world._process = Worker(self)
            sessions.append(self)
        def step(self):
            clock[0] += .001
            return super().step()

    folder = workspace(tmp_path)
    args = arguments(tmp_path, '--backend', 'physx_cpu', '--physics-hz', '120',
                     '--steps', '300', '--diagnostics-seconds', '1')
    report = benchmark.execute(args, config={'workspace': str(folder)}, session_factory=InstrumentedSession,
                               builder=lambda *_: folder / 'native')
    assert report['status'] == 'passed', report.get('error')
    samples = report['diagnostic_samples']
    assert [(row['label'], row['mode_time']) for row in samples] == [
        ('prepared', 0), ('initialization_complete', 0), ('periodic', 1), ('periodic', 2), ('final', 2.5)]
    assert [row['tick'] for row in samples] == [1, 2, 122, 242, 302]
    assert all(row['diagnostic_wall_seconds'] == pytest.approx(.1) for row in samples)
    assert report['timing_seconds']['diagnostics'] == pytest.approx(.5)
    assert report['timing_seconds']['initialization_steps'] == pytest.approx(.001)
    assert report['timing_seconds']['competition_steps'] == pytest.approx(.3)
    assert report['loop_wall_seconds'] == pytest.approx(.701)
    assert report['loop_diagnostics_wall_seconds'] == pytest.approx(.4)
    assert report['loop_wall_seconds_excluding_diagnostics'] == pytest.approx(.301)
    assert report['step_realtime_factor'] == pytest.approx(2.5 / .3)
    assert report['steps']['competition'] == 300
    assert os.environ['EERSIM_PROFILE'] == 'previous-value'
    assert sessions[0].world.closed


def test_requested_activity_failure_keeps_evidence_closes_and_restores_profile(tmp_path, monkeypatch):
    import os
    from types import SimpleNamespace
    monkeypatch.delenv('EERSIM_PROFILE', raising=False)
    worlds = []
    def broken(command, payload):
        if payload.get('activity'):
            raise RuntimeError('Activity probe is unavailable')
        return {'performance': {'enabled': True}}
    def create(config):
        world = FakeWorld(config)
        world._process = SimpleNamespace(failed=False, call=broken)
        worlds.append(world)
        return world
    args = arguments(tmp_path, '--backend', 'physx_cpu', '--no-pros', '--diagnostics-seconds', '1')
    report = benchmark.execute(args, config={}, world_factory=create)
    assert report['status'] == 'failed' and 'probe is unavailable' in report['error']
    assert report['initial_state']['tick'] == 1
    assert report['diagnostic_samples'] == []
    assert report['steps']['total'] == 0
    assert report['loop_wall_seconds'] == 0
    assert worlds[0].closed
    assert 'EERSIM_PROFILE' not in os.environ
    assert json.loads(args.output.read_text())['status'] == 'failed'


@pytest.mark.parametrize('failure_tick', [120, 180])
def test_periodic_or_final_query_failure_retains_completed_step_counts_and_csv(tmp_path, monkeypatch, failure_tick):
    import os
    from types import SimpleNamespace
    monkeypatch.delenv('EERSIM_PROFILE', raising=False)
    worlds = []
    def create(config):
        world = FakeWorld(config)
        def call(command, payload):
            if payload.get('activity') and world.tick - 1 >= failure_tick:
                raise RuntimeError('Read-only activity query failed')
            return {'performance': {'enabled': True}, 'scene_activity': {'counts': {}, 'actors': []}}
        world._process = SimpleNamespace(failed=False, call=call)
        worlds.append(world)
        return world
    args = arguments(tmp_path, '--backend', 'physx_cpu', '--no-pros', '--physics-hz', '120',
                     '--steps', '180', '--diagnostics-seconds', '1')
    report = benchmark.execute(args, config={}, world_factory=create)
    assert report['status'] == 'failed' and 'query failed' in report['error']
    assert report['steps'] == {'initialization': 0, 'competition': 0, 'passive': failure_tick, 'total': failure_tick}
    assert report['simulation_seconds']['total_advanced'] == failure_tick / 120
    assert report['final_state']['tick'] == failure_tick + 1
    assert report['loop_wall_seconds'] > 0 and report['timing_seconds']['passive_steps'] > 0
    assert report['step_realtime_factor'] > 0
    with args.output.with_suffix('.csv').open() as stream:
        assert len(list(csv.DictReader(stream))) == failure_tick
    assert worlds[0].closed and 'EERSIM_PROFILE' not in os.environ


@pytest.mark.parametrize('value', ['nan', '-1', 'inf'])
def test_invalid_native_diagnostic_interval_rejected_before_start(tmp_path, value):
    with pytest.raises(ValueError, match='diagnostics-seconds'):
        benchmark.execute(arguments(tmp_path, '--diagnostics-seconds', value), config={})


def test_bullet_cannot_silently_ignore_requested_native_activity(tmp_path):
    with pytest.raises(ValueError, match='require PhysX'):
        benchmark.execute(arguments(tmp_path, '--diagnostics-seconds', '1'), config={})
