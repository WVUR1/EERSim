"""Live MuJoCo poses survive render, replay and session reset workflows."""
import json

import numpy as np
import pytest

pytest.importorskip('mujoco')
bullet = pytest.importorskip('pybullet')

from eersim.physics_backend import create_world
from eersim.replay import ReplayRecorder


def scene(tmp_path):
    robot = tmp_path / 'robot.urdf'
    robot.write_text('''<robot name="workflow">
      <link name="base"><inertial><origin xyz=".01 -.02 .03"/>
        <mass value="1"/><inertia ixx=".03" iyy=".03" izz=".03" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><geometry><box size=".2 .2 .04"/></geometry></visual></link>
      <link name="arm"><inertial><mass value=".5"/>
        <inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><origin xyz=".1 0 0"/><geometry><box size=".2 .04 .04"/></geometry></visual></link>
      <joint name="hinge" type="continuous"><parent link="base"/><child link="arm"/>
        <origin xyz="0 0 .1"/><axis xyz="0 0 1"/>
        <limit effort="4" velocity="100"/></joint></robot>''')
    field = tmp_path / 'field.json'
    field.write_text(json.dumps({'field': {'size': [2, 2], 'floor': False, 'walls': False}, 'objects': []}))
    return {'physics_backend': 'mujoco', 'robot_urdf': str(robot), 'robot_config': '',
            'game_pack': str(field), 'fixed_base': True, 'gravity': 0, 'renderer': 'cpu',
            'start_pose': [.2, -.1, .3, 15], 'physics_hz': 240,
            'motors': [{'port': 1, 'joint': 'hinge', 'cartridge': 'green'}]}


def test_replay_and_cpu_render_use_live_mujoco_and_survive_reset(tmp_path, monkeypatch):
    monkeypatch.setenv('EERSIM_RENDERER', 'cpu')
    monkeypatch.setattr(bullet, 'stepSimulation', lambda **_: pytest.fail('Bullet must not integrate MuJoCo worlds'))
    config = scene(tmp_path)
    clip = None
    with create_world(config) as world:
        initial = world.snapshot()
        recorder = ReplayRecorder.start(world, mode='match_auto', duration=1, capture_hz=240,
                                        cache_root=tmp_path / 'replays', observed=initial)
        try:
            for _ in range(48):
                observed = world.step({'motors': {'1': {'mode': 'voltage', 'value': 4000}}})
                recorder.capture(world, phase='autonomous', mode_time=world.time, observed=observed)
            assert abs(observed['joints'][0]['position']) > .05
            before = world.snapshot()
            pixels = world.render(160, 120, target=[.2, -.1, .35], distance=.8)
            assert pixels.shape == (120, 160, 4)
            assert world.snapshot() == before
            assert np.std(pixels[:, :, :3]) > 0
            native_joint = world._physics.getJointState(world.robot, 0)[0]
            assert bullet.getJointState(world.robot, 0, physicsClientId=world.client)[0] == pytest.approx(native_joint)
            clip = recorder.finish(world, phase='disabled', mode_time=world.time, observed=observed)
            start, end = clip.frame_at(0).poses.copy(), clip.frame_at(clip.frame_count - 1).poses.copy()
            assert not np.allclose(start, end)
            # The replay remains self-contained while the live scene is reset.
            world.disable()
            reset = world.reset(config)
            assert reset['physics']['backend'] == 'mujoco'
            assert world.tick == 0 and world.time == 0
            assert reset['joints'][0]['position'] == pytest.approx(0)
            assert clip.frame_at(clip.frame_count - 1).poses == pytest.approx(end)
            assert clip.index_at(clip.duration) == clip.frame_count - 1
            assert clip.duration == pytest.approx(48 / 240)
        finally:
            recorder.close()
            if clip is not None:
                clip.close()


def test_pid_characterization_records_native_motor_feedback_and_disables_effort(tmp_path):
    import math
    from eersim.runner import Session
    from eersim.tuning import export_capture, read_capture
    from eersim.tuning_simulation import SimulationTuner

    config = scene(tmp_path)
    samples, statuses, logs = [], [], []
    session = Session(config, log=logs.append)
    target = {'id': 'hinge', 'name': 'Arm', 'ports': [1], 'axes': {'shaft': [1]}}
    adapter = session.world._physics

    def observe(sample):
        # Check the emitted record while its corresponding native state is live,
        # not against another copy of the tuner's own telemetry conversion.
        joint = adapter.getJointState(session.world.robot, 0)
        assert sample['time'] == pytest.approx(adapter.data.time, abs=1e-12)
        assert sample['motors']['1']['position_deg'] == pytest.approx(math.degrees(joint[0]), abs=1e-10)
        assert sample['motors']['1']['velocity_rpm'] == pytest.approx(joint[1] * 30 / math.pi, abs=1e-10)
        assert sample['motors']['1']['command_mode'] == 'voltage'
        assert sample['motors']['1']['command_value'] == sample['motors']['1']['voltage_mv']
        samples.append(sample)

    tuner = SimulationTuner(observe, statuses.append)
    try:
        tuner.experiment(session, {'kind': 'characterize', 'target': target, 'axis': 'shaft',
                                  'duration_s': 2, 'voltage_mv': 3000, 'max_travel_deg': 3000,
                                  'controller_period_ms': 1000 / 60})
        for _ in range(700):
            if not tuner.active:
                break
            state = tuner.step(session)
            assert state['physics']['backend'] == 'mujoco'
        assert not tuner.active and not tuner.capturing
        assert session.paused and session.motor_observer is None and session.runtime is None
        assert statuses[-1]['state'] == 'completed'
        assert statuses[-1]['termination'] == samples[-1]['termination'] == 'duration'
        assert samples[-1]['time'] == pytest.approx(2.8)
        assert len(samples) > 100
        assert len({sample['capture_id'] for sample in samples}) == 1
        times = np.array([sample['time'] for sample in samples])
        positions = np.array([sample['motors']['1']['position_deg'] for sample in samples])
        voltages = np.array([sample['motors']['1']['voltage_mv'] for sample in samples])
        assert np.all(np.diff(times) > 0)
        assert set(voltages) == {0, 3000}
        assert voltages[0] == voltages[-1] == 0
        assert positions[-1] > 360  # Captured encoders remain continuous across a full turn.
        assert np.all(np.diff(positions) >= -1e-8)
        assert np.max(np.diff(positions)) < 25
        path = tmp_path / 'mujoco-motor-capture.jsonl'
        export_capture(path, samples)
        assert read_capture(path) == samples
        assert np.all(adapter.model.actuator_forcerange == 0)
        session.world.step()
        assert np.all(adapter.data.actuator_force == 0)
        assert any('mujoco' in entry for entry in logs)
    finally:
        tuner.cancel(session)
        session.close()


@pytest.mark.skipif(__import__('os').environ.get('EERSIM_NATIVE_INTEGRATION') != '1',
                    reason='Set EERSIM_NATIVE_INTEGRATION=1 to run the unchanged compiled PROS claw demo')
def test_native_claw_demo_places_pin_at_240hz(tmp_path):
    """Opt-in integration: exact goal/robot/preload, full unmodified PROS routine.

    This isolates grip/insertion correctness; full-field timings belong to the
    separate benchmark. Requires the native C++ build tools documented in README.
    The 120 Hz run has known compliance-induced grip drift and is not substituted
    for this 240 Hz insertion-accuracy check.
    """
    import hashlib
    from pathlib import Path
    import time
    import xml.etree.ElementTree as ET
    from eersim.bridge import build_workspace
    from eersim.demos import claw_demo_config
    from eersim.runner import Session

    config = claw_demo_config()
    original_game = Path(config['game_pack'])
    game = json.loads(original_game.read_text(encoding='utf-8'))
    game['field']['walls'] = False
    game['objects'] = [item for item in game['objects'] if item['name'] == 'goal_red_5']
    assert len(game['objects']) == 1
    # Resolve source-relative URDFs before moving only this diagnostic field file.
    for item in game['objects']:
        item['urdf'] = str((original_game.parent / item['urdf']).resolve())
    field = tmp_path / 'isolated-goal.json'
    field.write_text(json.dumps(game), encoding='utf-8')
    config.update(physics_backend='mujoco', physics_hz=240, physics_cpu_threads=2,
                  renderer='cpu', game_pack=str(field))
    repo = Path(__file__).resolve().parents[1]
    sources = {original_game, Path(config['robot_config']),
               repo / 'eersim/mujoco_world.py', repo / 'eersim/mujoco_scene.py'}
    for urdf in [config['robot_urdf'], config['field_objects'][0]['urdf'], game['objects'][0]['urdf']]:
        urdf = Path(urdf).resolve()
        sources.add(urdf)
        for mesh in ET.parse(urdf).iter('mesh'):
            sources.add((urdf.parent / mesh.attrib['filename']).resolve())
    workspace = Path(config['workspace'])
    for folder in ('src', 'include'):
        sources.update(path for path in (workspace / folder).rglob('*') if path.is_file())
    evidence = {'scenario': 'isolated exact goal; unchanged native claw PROS routine',
                'physics_hz': 240, 'full_field': False,
                'source_sha256': {str(path.relative_to(repo)).replace('\\', '/'):
                                  hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in sorted(sources)}, 'samples': []}
    logs = []
    executable = build_workspace(workspace, logs.append)
    evidence['native_executable_sha256'] = hashlib.sha256(executable.read_bytes()).hexdigest()
    session = Session(config, logs.append)
    world = session.world
    native = world._physics
    pin = next(item['id'] for item in world.field_objects if item['name'] == 'demo_preload')
    started = time.perf_counter()
    try:
        session.start('skills_auto', executable)
        for index in range(240 * 19):
            state = session.step()
            assert np.all(np.isfinite(native.data.qpos))
            assert not np.any(native.data.warning.number)
            if index % 24 == 0:
                position, quaternion = native.getBasePositionAndOrientation(pin)
                evidence['samples'].append({'mode_time': state['mode_time'],
                                            'pin_position': position, 'pin_quaternion': quaternion,
                                            'scoring': world._scoring()})
            if state['mode_time'] >= 17:
                break
        evidence['physics'] = world.physics_status
        evidence['final_scoring'] = world._scoring()
        assert state['mode_time'] >= 17
        assert any('Placement complete' in entry for entry in logs)
        assert not any('timeout' in entry.lower() for entry in logs)
        assert any(item['pin'] == 'demo_preload' and item['goal'] == 'goal_red_5'
                   for item in evidence['final_scoring']['placements'])
        assert not native.getContactPoints(bodyA=pin, bodyB=world.robot)
        assert max(sample['pin_position'][2] for sample in evidence['samples']) > .25
        final_position, quaternion = native.getBasePositionAndOrientation(pin)
        axis = np.array(native.getMatrixFromQuaternion(quaternion)).reshape(3, 3)[:, 2]
        assert axis[2] > .99
        assert .08 < final_position[2] < .10
    finally:
        evidence['wall_seconds'] = time.perf_counter() - started
        evidence['logs'] = logs
        report = tmp_path / 'native-claw-240.json'
        report.write_text(json.dumps(evidence, indent=2), encoding='utf-8')
        print(f'Native claw integration evidence: {report}')
        session.close()
