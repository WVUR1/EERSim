"""Session-only regression checks; no physics engine or native executable."""
import copy
import random
import sys

import pytest

from eersim.runner import Session


class ObservedWorld:
    motor_bindings = []
    physics_status = {"backend": "fake", "hardware_accelerated": False}
    renderer_status = {}

    def __init__(self, config):
        self.joint_property_reads = 0
        self.snapshot_reads = 0
        self.observations = 0
        self.reset(config)

    @property
    def joints(self):
        self.joint_property_reads += 1
        return [{"name": "arm", "position": self.tick / 10, "velocity": self.command}]

    def reset(self, config):
        self.tick = 0
        self.command = 0
        self.dt = 1 / config["physics_hz"]
        self.rng = random.Random(7)
        self.pending = []
        self.delivered = {"position": 0.0, "velocity": 0.0}
        self.commands = []
        self._observe()

    def _observe(self):
        self.observations += 1
        # A deterministic delayed/dropout sensor double makes additional
        # sampling or an accidental return to an older revision observable.
        self.pending.append({"position": self.tick + self.rng.uniform(-.05, .05),
                             "velocity": self.command})
        if len(self.pending) > 2:
            ready = self.pending.pop(0)
            if self.tick % 4:
                self.delivered = ready
        self.observed = {"time": self.tick * self.dt, "tick": self.tick,
                         "pose": [self.tick / 10, 0, 0, 0],
                         "motors": {"1": copy.deepcopy(self.delivered)},
                         "otos": [self.tick, 0, 0], "imu": {"heading": self.tick},
                         "joints": self.joints,
                         "scoring": {"placements": [{"pin": "p", "goal": "g"}]}}

    def snapshot(self):
        self.snapshot_reads += 1
        return copy.deepcopy(self.observed)

    def step(self, commands):
        self.commands.append(copy.deepcopy(commands))
        self.command = commands.get("motors", {}).get("1", {}).get("value", 0)
        self.tick += 1
        self._observe()
        return self.snapshot()

    def disable(self):
        self.command = 0

    def close(self):
        pass


class Runtime:
    def __init__(self, executable):
        self.inputs = []

    def step(self, snapshot, mode, dt, controller):
        self.inputs.append((mode, copy.deepcopy(snapshot)))
        return {"initialized": True, "motors": {"1": {"mode": "voltage",
                "value": int(snapshot["motors"]["1"]["position"] * 100)}}}

    def close(self):
        pass


def make_session(session_class=Session, world_class=ObservedWorld, duration=60):
    session = session_class({"physics_hz": 120, "duration_overrides": {"skills_auto": duration}},
                            world_factory=world_class, runtime_factory=Runtime)
    session.start("skills_auto", "unused-fake-executable")
    return session


def test_same_revision_joint_observation_does_not_resample():
    session = make_session()
    try:
        before = session.world.joint_property_reads
        state = session.step()
        assert state["joints"] == [{"name": "arm", "position": .1, "velocity": 0}]
        assert session.world.joint_property_reads == before + 1  # The new observation only.
        assert session.world.observations == 2
        assert session.world.snapshot_reads == 2  # Native input + completed step.
        reads = session.world.joint_property_reads
        assert session.state()["joints"] == state["joints"]
        assert session.world.joint_property_reads == reads
    finally:
        session.close()


def test_returned_state_mutation_cannot_change_sensors_or_later_revisions():
    session = make_session()
    try:
        first = session.step()
        expected = session.world.snapshot()
        first["joints"][0]["position"] = 999
        first["motors"]["1"]["position"] = 999
        first["scoring"]["placements"].clear()
        assert session.world.snapshot() == expected
        second = session.step()
        assert session.runtime.inputs[-1][1] == expected
        assert second["joints"][0]["position"] == .2
        second_saved = copy.deepcopy(second)
        session.step()
        assert second == second_saved
        session.reset()
        assert session.state()["joints"][0]["position"] == 0
        assert second == second_saved
    finally:
        session.close()


def test_empty_joint_observation_is_valid_and_does_not_use_property():
    session = make_session()
    try:
        before = session.world.joint_property_reads
        assert session.state({"joints": []})["joints"] == []
        assert session.world.joint_property_reads == before
    finally:
        session.close()


def test_injected_world_without_joint_snapshot_keeps_property_fallback():
    class LegacyWorld(ObservedWorld):
        def snapshot(self):
            state = super().snapshot()
            del state["joints"]
            return state

    session = make_session(world_class=LegacyWorld)
    try:
        before = session.world.joint_property_reads
        state = session.step()
        assert state["joints"][0]["position"] == .1
        assert session.world.joint_property_reads == before + 2  # Observe + legacy fallback.
    finally:
        session.close()


def test_world_step_none_fallback_reuses_fresh_snapshot():
    class NoneWorld(ObservedWorld):
        def step(self, commands):
            super().step(commands)
            return None

    session = make_session(world_class=NoneWorld)
    try:
        before = session.world.joint_property_reads
        assert session.step()["joints"][0]["position"] == .1
        assert session.world.joint_property_reads == before + 1
    finally:
        session.close()


def test_full_mock_lifecycle_matches_unmodified_session_exactly():
    # The harness explicitly supplies the unmodified module, preventing this
    # comparison from silently importing the candidate twice.
    baseline_module = sys.modules.get("eersim._snapshot_baseline_runner")
    if baseline_module is None:
        pytest.skip("Run through run_snapshot_tests.py for the baseline comparison")
    baseline = make_session(baseline_module.Session, duration=.2)
    candidate = make_session(duration=.2)
    try:
        baseline_runtime, candidate_runtime = baseline.runtime, candidate.runtime
        baseline_states, candidate_states = [], []
        while baseline.running or candidate.running:
            assert baseline.running == candidate.running
            baseline_states.append(baseline.step())
            candidate_states.append(candidate.step())
        assert candidate_states == baseline_states
        assert candidate_runtime.inputs == baseline_runtime.inputs
        assert candidate.world.commands == baseline.world.commands
        assert candidate.samples == baseline.samples
        assert candidate.world.observations == baseline.world.observations == 26
        assert candidate.world.snapshot_reads == baseline.world.snapshot_reads
        assert candidate.world.rng.getstate() == baseline.world.rng.getstate()
        assert baseline.world.joint_property_reads - candidate.world.joint_property_reads == 24
    finally:
        baseline.close()
        candidate.close()
