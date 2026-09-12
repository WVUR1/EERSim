"""Timing regression tests: slow rendering must never discard physics time."""
import copy

import pytest

from eersim.runner import FixedStepClock, Session


@pytest.mark.parametrize("fps", [7, 15, 30, 60, 144])
def test_render_rate_does_not_change_physics_steps(fps):
    clock = FixedStepClock(240)
    steps = 0
    for _ in range(fps * 5):
        clock.add(1 / fps)
        steps += clock.take(16)
    while clock.pending >= clock.dt - 1e-10:
        steps += clock.take(16)
    assert steps == 1200


def test_large_stall_retains_full_backlog():
    clock = FixedStepClock(240)
    clock.add(3.0)
    assert clock.take(8) == 8
    assert clock.pending == pytest.approx(3.0 - 8 / 240)
    assert sum(clock.take(8) for _ in range(89)) == 712
    assert clock.pending == pytest.approx(0, abs=1e-10)


class FakeWorld:
    joints = []
    motor_bindings = []

    def __init__(self, config):
        self.reset(config)

    def reset(self, config):
        self.dt = 1 / config["physics_hz"]
        self.time = 0
        self.commands = []
        self.disable_calls = 0

    def snapshot(self):
        return {"time": self.time, "pose": [0, 0, 0, 0], "motors": {}}

    def step(self, commands):
        self.commands.append(copy.deepcopy(commands))
        self.time += self.dt
        return self.snapshot()

    def close(self):
        pass

    def disable(self):
        self.disable_calls += 1


class FakeRuntime:
    def __init__(self, executable):
        self.modes = []
        self.closed = False

    def step(self, snapshot, mode, dt, controller):
        self.modes.append(mode)
        return {"initialized": True, "callback_done": True,
                "motors": {"1": {"mode": "voltage", "value": 12000}}}

    def close(self):
        self.closed = True


@pytest.mark.parametrize("mode,callback", [("skills_auto", "autonomous"), ("skills_driver", "opcontrol"),
                                           ("match_auto", "autonomous"), ("driver_auto", "opcontrol")])
def test_mode_lifecycle_and_duration_excludes_initialization(mode, callback):
    config = {"physics_hz": 240, "duration_overrides": {mode: 0.1}}
    session = Session(config, world_factory=FakeWorld, runtime_factory=FakeRuntime)
    session.start(mode, "dummy")
    runtime = session.runtime
    while session.running:
        session.step()
    assert runtime.modes == ["initialize"] + [callback] * 24 + ["disabled"]
    assert session.elapsed_ticks == 24
    assert session.world.time == pytest.approx(25 / 240)
    assert session.world.commands[0]["motors"]["1"]["value"] == 12000
    assert session.world.commands[1]["motors"]
    assert runtime.closed
    assert session.world.disable_calls == 2
    session.close()


def test_game_pack_duration_and_explicit_experiment_override():
    session = Session({"physics_hz": 240}, world_factory=FakeWorld, runtime_factory=FakeRuntime)
    session.world.game = {"mode_durations": {"match_auto": 15}}
    session.start("match_auto", "dummy")
    assert session.duration == 15
    session.config["duration_overrides"] = {"match_auto": 30}
    session.start("match_auto")
    assert session.duration == 30
    session.close()


def test_manual_joint_targets_are_cleared_before_code_starts():
    session = Session({"physics_hz": 240}, world_factory=FakeWorld, runtime_factory=FakeRuntime)
    session.manual_joints = {"arm": .7}
    session.step()
    assert session.world.commands[-1]["joints"] == {"arm": .7}
    session.start("skills_auto", "dummy")
    session.step()
    assert not session.manual_joints
    assert "joints" not in session.world.commands[-1]
    session.close()


def test_native_initialization_can_home_actuators_without_spending_mode_time(tmp_path):
    from eersim.bridge import BuildError, NativeRuntime, _compiler, build_workspace

    try:
        _compiler()
    except BuildError as exc:
        pytest.skip(str(exc))
    (tmp_path / "src").mkdir()
    (tmp_path / "include").mkdir()
    (tmp_path / "include" / "main.h").write_text(
        '#include "api.h"\nextern "C" {void initialize();void competition_initialize();'
        'void autonomous();void opcontrol();void disabled();}\n')
    (tmp_path / "src" / "main.cpp").write_text(r'''
#include "main.h"
pros::Motor arm(1);
pros::adi::DigitalOut latch('A', false);
void initialize() {
  latch.set_value(true);
  arm.move_voltage(6000);
  while (arm.get_position() < 1.0) pros::delay(5);
  arm.move_voltage(0);
}
void competition_initialize() {
  latch.set_value(false);
  arm.move_voltage(-3000);
  pros::delay(10);
  arm.move_voltage(0);
}
void autonomous() { arm.move_voltage(1200); }
void opcontrol() {}
void disabled() {}
''')

    class HomingWorld(FakeWorld):
        """A minimal encoder plant exposes a broken startup feedback loop."""

        def reset(self, config):
            super().reset(config)
            self.position = 0.0

        def snapshot(self):
            return {**super().snapshot(),
                    "motors": {"1": {"position": self.position, "velocity": 0.0}}}

        def step(self, commands):
            voltage = commands.get("motors", {}).get("1", {}).get("value", 0)
            self.position += voltage / 6000 * self.dt
            return super().step(commands)

    executable = build_workspace(tmp_path)
    session = Session({"physics_hz": 240, "duration_overrides": {"skills_auto": .025}},
                      world_factory=HomingWorld, runtime_factory=NativeRuntime)
    try:
        session.start("skills_auto", executable)
        states = []
        for _ in range(100):
            states.append(session.step())
            if not session.running:
                break
        assert not session.running, "Initialization failed to receive actuator feedback"
        assert session.initialized
        assert session.init_ticks > 1
        assert all(state["mode_time"] == 0 for state in states[:session.init_ticks])
        startup = session.world.commands[:session.init_ticks]
        assert startup[0]["motors"]["1"]["value"] == 6000
        assert startup[0]["adi"]["1"] is True
        assert any(command["motors"]["1"]["value"] == -3000
                   and command["adi"]["1"] is False for command in startup)
        competition = session.world.commands[session.init_ticks:]
        assert len(competition) == session.elapsed_ticks == 6
        assert all(command["motors"]["1"]["value"] == 1200 for command in competition)
        assert session.world.time == pytest.approx((session.init_ticks + 6) / 240)
        assert session.world.disable_calls == 2
    finally:
        session.close()
