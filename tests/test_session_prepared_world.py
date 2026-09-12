"""Reuse a pristine preview without weakening explicit reset/start semantics."""
from copy import deepcopy

import pytest

from eersim.runner import Session


class PreparedWorld:
    joints = []
    motor_bindings = []

    def __init__(self, config):
        self.resets = 0
        self.disable_calls = 0
        self.reset(config)

    def reset(self, config):
        self.resets += 1
        self.config = deepcopy(config)
        # Model a backend with a required normal-timestep bootstrap.
        self.time = 1 / 240
        self.position = list(config['start_pose'])
        self.physics_status = {'backend': 'test', 'initialization_steps': 1}

    def snapshot(self):
        return {'time': self.time, 'pose': self.position[:], 'motors': {}}

    def step(self, commands):
        self.time += 1 / 240
        self.position[0] += .01
        return self.snapshot()

    def disable(self):
        self.disable_calls += 1

    def close(self):
        pass


class Runtime:
    def __init__(self, executable):
        self.closed = False

    def step(self, *args):
        return {'initialized': True}

    def close(self):
        self.closed = True


def session(runtime_factory=Runtime):
    return Session({'physics_hz': 240, 'start_pose': [1, 2, .1, 0],
                    'noise': {'position_std_m': .001}},
                   world_factory=PreparedWorld, runtime_factory=runtime_factory)


def test_first_start_reuses_prepared_bootstrap_after_readonly_preview_and_stop():
    run = session()
    try:
        original = run.world.snapshot()
        run.state()
        run.stop()  # A build action disables commands but does not advance the scene.
        run.start('skills_auto', 'dummy')
        assert run.world.resets == 1
        assert run.world.snapshot() == original
        assert run.world.time == pytest.approx(1 / 240)
        assert run.world.disable_calls == 2
    finally:
        run.close()


@pytest.mark.parametrize('manual', [True, False])
def test_any_stepped_scene_resets_before_start(manual):
    run = session()
    try:
        if manual:
            run.manual_joints = {'lift': .2}
        else:
            run.start('skills_auto', 'dummy')
        run.step()
        assert run.world.position[0] > 1
        run.start('skills_auto', 'dummy')
        assert run.world.resets == 2
        assert run.world.position == [1, 2, .1, 0]
        assert run.world.time == pytest.approx(1 / 240)
        assert not run.manual_joints
    finally:
        run.close()


def test_nested_configuration_change_invalidates_prepared_world():
    run = session()
    try:
        run.config['noise']['position_std_m'] = .02
        run.config['start_pose'][0] = -1
        run.start('skills_auto', 'dummy')
        assert run.world.resets == 2
        assert run.world.config['noise']['position_std_m'] == .02
        assert run.world.position[0] == -1
    finally:
        run.close()


def test_explicit_reset_prepares_a_reusable_world_but_repeated_start_consumes_it():
    run = session()
    try:
        run.step()
        run.reset()
        assert run.world.resets == 2
        run.start('skills_auto', 'dummy')
        assert run.world.resets == 2
        previous = run.runtime
        run.start('skills_auto')
        assert previous.closed
        assert run.world.resets == 3
    finally:
        run.close()


def test_failed_runtime_start_cannot_reuse_potentially_changed_prepared_state():
    def fail(executable):
        raise RuntimeError('cannot launch child')

    run = session(fail)
    try:
        with pytest.raises(RuntimeError, match='cannot launch'):
            run.start('skills_auto', 'dummy')
        assert run.world.resets == 1
        run.runtime_factory = Runtime
        run.start('skills_auto')
        assert run.world.resets == 2
    finally:
        run.close()
