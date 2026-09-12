"""Bounded diagnostics must not change simulation state or clock semantics."""
from copy import deepcopy
import importlib.util
import json

import pytest

from eersim.performance import PerformanceRecorder


class Clock:
    now = 0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += int(seconds * 1_000_000_000)


def test_disabled_recorder_never_reads_clock_and_never_swallows_exceptions(monkeypatch):
    monkeypatch.delenv('EERSIM_PROFILE', raising=False)

    def forbidden():
        pytest.fail('Disabled diagnostics read a wall clock')

    recorder = PerformanceRecorder(clock=forbidden)
    with pytest.raises(RuntimeError, match='original failure'):
        with recorder.span('disabled'):
            recorder.increment('calls')
            recorder.gauge('actors', 88)
            raise RuntimeError('original failure')
    assert recorder.snapshot()['spans'] == {}
    assert recorder.snapshot()['counters'] == {}
    assert recorder.snapshot()['gauges'] == {}
    monkeypatch.setenv('EERSIM_PROFILE', '1')
    assert PerformanceRecorder().enabled
    assert not PerformanceRecorder(False).enabled


def test_percentiles_are_bounded_recent_samples_but_aggregates_cover_all_spans():
    clock = Clock()
    recorder = PerformanceRecorder(True, sample_limit=3, clock=clock)
    for duration in (9, 1, 3, 4, 5):
        with recorder.span('native.step'):
            clock.advance(duration)
    summary = recorder.snapshot()
    stage = summary['spans']['native.step']
    assert stage == {'count': 5, 'total': 22., 'mean': 4.4, 'max': 9., 'p50': 4.,
                     'p95': pytest.approx(4.9), 'sample_count': 3}
    assert 'recent' in summary['sample_policy']
    json.dumps(summary, allow_nan=False)


def test_nested_failed_spans_are_inclusive_and_snapshot_reset_starts_new_interval():
    clock = Clock()
    recorder = PerformanceRecorder(True, clock=clock)
    with pytest.raises(ValueError, match='solver error'):
        with recorder.span('world'):
            clock.advance(1)
            with recorder.span('native'):
                clock.advance(2)
                raise ValueError('solver error')
    recorder.increment('steps', 4)
    recorder.gauge('actors', 88)
    previous = recorder.snapshot(reset=True)
    assert previous['spans']['world']['total'] == 3
    assert previous['spans']['native']['total'] == 2
    assert previous['counters']['steps'] == 4
    assert recorder.snapshot()['spans'] == {}
    assert recorder.snapshot()['counters'] == {}
    assert recorder.snapshot()['gauges'] == {'actors': 88}
    with recorder.span('crosses_reset'):
        clock.advance(2)
        recorder.snapshot(reset=True)
        clock.advance(3)
    assert recorder.snapshot()['spans'] == {}
    previous['gauges']['actors'] = -1
    assert recorder.snapshot()['gauges']['actors'] == 88


def test_metric_names_and_sample_storage_stay_bounded():
    recorder = PerformanceRecorder(True, sample_limit=2, max_names=3)
    for _ in range(20):
        for name in ('one', 'two', 'three', 'extra'):
            with recorder.span(name):
                pass
            recorder.increment(name)
            recorder.gauge(name, 1)
    summary = recorder.snapshot()
    assert len(summary['spans']) == len(summary['counters']) == len(summary['gauges']) == 3
    assert all(value['count'] == 20 and value['sample_count'] == 2 for value in summary['spans'].values())
    assert summary['dropped_metrics'] == 60


@pytest.mark.parametrize('options', [{'sample_limit': 0}, {'sample_limit': True}, {'max_names': -1}])
def test_invalid_storage_limits_fail_early(options):
    with pytest.raises(ValueError, match='positive integer'):
        PerformanceRecorder(**options)


def tiny_scene(tmp_path, backend='bullet'):
    robot = tmp_path / 'rotor.urdf'
    robot.write_text('''<robot name="profile_fixture">
      <link name="base_link"><inertial><mass value="1"/>
        <inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <link name="rotor"><inertial><mass value=".2"/>
        <inertia ixx=".02" iyy=".02" izz=".02" ixy="0" ixz="0" iyz="0"/></inertial>
        <collision><geometry><box size=".03 .03 .03"/></geometry></collision></link>
      <joint name="shaft" type="continuous"><parent link="base_link"/><child link="rotor"/>
        <origin xyz="0 0 .2"/><axis xyz="0 0 1"/><limit effort="3" velocity="30"/></joint>
      </robot>''')
    field = tmp_path / 'field.json'
    field.write_text(json.dumps({'field': {'size': [2, 2], 'floor': False, 'walls': False}, 'objects': []}))
    return {'robot_urdf': str(robot), 'robot_config': '', 'game_pack': str(field),
            'physics_backend': backend, 'physics_hz': 240, 'fixed_base': True, 'gravity': 0,
            'start_pose': [0, 0, .5, 0], 'renderer': 'cpu',
            'motors': [{'port': 1, 'joint': 'shaft', 'cartridge': 'red', 'gear_ratio': 1}],
            'noise': {'encoder_std_deg': .2, 'position_std_m': .003}}


def test_bullet_profiling_preserves_exact_state_noise_and_fixed_clock(tmp_path, monkeypatch):
    pytest.importorskip('pybullet')
    from eersim.physics import World
    config = tiny_scene(tmp_path)
    monkeypatch.setenv('EERSIM_PROFILE', '0')
    with World(config) as profiled, World(deepcopy(config)) as plain:
        profiled.performance = PerformanceRecorder(True, sample_limit=3)
        for _ in range(6):
            command = {'motors': {'1': {'mode': 'torque', 'value': .02}}}
            assert profiled.step(command) == plain.step(command)
        summary = profiled.performance.snapshot()
        for stage in ('world.control.motors', 'world.control.tires', 'world.control.pneumatics',
                      'world.control.feedback', 'world.integrate', 'world.rest', 'world.sensors', 'world.snapshot'):
            assert summary['spans'][stage]['count'] == 6
            assert summary['spans'][stage]['sample_count'] == 3
        assert summary['counters']['world.steps'] == 6
        assert profiled.time == plain.time == .025
        assert 'performance' not in profiled.snapshot()
        assert plain.performance.snapshot()['spans'] == {}


def test_session_roundtrip_and_lifecycle_timings_do_not_spend_mode_time(monkeypatch):
    from eersim.runner import Session

    class World:
        joints = motor_bindings = []

        def __init__(self, config):
            self.reset(config)

        def reset(self, config):
            self.time = 0

        def snapshot(self):
            return {'time': self.time, 'pose': [0, 0, 0, 0]}

        def step(self, commands):
            self.time += 1 / 240
            return self.snapshot()

        def disable(self):
            pass

        def close(self):
            pass

    class Runtime:
        def __init__(self, executable):
            pass

        def step(self, *args):
            return {'initialized': True}

        def close(self):
            pass

    monkeypatch.setenv('EERSIM_PROFILE', '1')
    session = Session({'physics_hz': 240, 'duration_overrides': {'skills_auto': .025}},
                      world_factory=World, runtime_factory=Runtime)
    try:
        session.start('skills_auto', 'fixture')
        setup = session.performance.snapshot(reset=True)
        assert setup['spans']['session.setup.world']['count'] == 1
        assert setup['spans']['session.setup.native']['count'] == 1
        while session.running:
            session.step()
        assert session.init_ticks == 1 and session.elapsed_ticks == 6
        stats = session.performance.snapshot()
        assert stats['spans']['session.native.roundtrip']['count'] == 7
        assert stats['spans']['session.world.step']['count'] == 7
        assert stats['spans']['session.native.disable_roundtrip']['count'] == 1
        assert stats['spans']['session.native.close']['count'] == 1
    finally:
        session.close()


@pytest.mark.skipif(any(importlib.util.find_spec(name) is None for name in ('ovphysx', 'ovstage', 'scipy', 'pybullet')),
                    reason='Optional pinned PhysX CPU runtime is unavailable')
def test_private_worker_profiling_can_reset_without_advancing_or_recapturing(tmp_path, monkeypatch):
    from eersim import physx_world
    from eersim.physx_world import PhysXWorld
    monkeypatch.setattr(physx_world, 'user_data_dir', lambda: tmp_path / 'runtime')
    config = tiny_scene(tmp_path, 'physx_cpu')
    monkeypatch.setenv('EERSIM_PROFILE', '1')
    with PhysXWorld(config) as profiled:
        setup = profiled._process.call('diagnostics', {'reset': True})['performance']
        assert setup['enabled']
        assert setup['spans']['worker.native.bootstrap']['count'] == 1
        assert setup['spans']['worker.setup']['count'] == 1
        inventory = setup['gauges']
        assert inventory['scene.dynamic_actors'] > 0
        assert inventory['scene.dofs'] == 1
        monkeypatch.setenv('EERSIM_PROFILE', '0')
        with PhysXWorld(deepcopy(config)) as plain:
            for _ in range(6):
                command = {'motors': {'1': {'mode': 'torque', 'value': .02}}}
                assert profiled.step(command) == plain.step(command)
            before = profiled.snapshot()
            summary = profiled._process.call('diagnostics')['performance']
            for name in ('worker.control.write', 'worker.native.step', 'worker.state.read',
                         'worker.contacts.read_convert'):
                assert summary['spans'][name]['count'] == 6
            assert summary['counters']['worker.steps'] == 6
            assert summary['gauges']['scene.dofs'] == inventory['scene.dofs']
            assert 'worker.contacts.raw_reports_last' in summary['gauges']
            assert summary['gauges']['worker.contacts.materialized_last'] == 0
            assert summary['gauges']['worker.contacts.body_pairs_last'] == 0
            assert profiled._process.call('diagnostics')['performance'] == summary
            assert profiled.snapshot() == before
            profiled._process.call('diagnostics', {'reset': True})
            assert profiled._process.call('diagnostics')['performance']['spans'] == {}
            assert profiled.snapshot() == before
            assert profiled._process.performance.snapshot()['spans']['physx.rpc.step']['count'] == 6
            assert plain._process.call('diagnostics')['performance']['spans'] == {}
            json.dumps(summary, allow_nan=False)
