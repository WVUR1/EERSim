"""Actual pinned PhysX CPU integration; no graphics context or CUDA device.

These tests launch real isolated workers. Environments without the optional
runtime skip them; an installed runtime that fails startup is a test failure.
"""
from copy import deepcopy
import importlib.util
import json
import math

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    any(importlib.util.find_spec(name) is None for name in ('ovphysx', 'ovstage', 'scipy', 'pybullet')),
    reason='The optional pinned ovphysx CPU runtime is not installed')

from eersim import physx_world
from eersim.physx_world import PhysXWorld
from eersim.runner import FixedStepClock


@pytest.fixture(autouse=True)
def owned_runtime_data(tmp_path, monkeypatch):
    monkeypatch.setattr(physx_world, 'user_data_dir', lambda: tmp_path / 'runtime-data')


def inertia_link(name, inertia=.02, mass=.5):
    return f'''<link name="{name}"><inertial><mass value="{mass}"/>
      <inertia ixx="{inertia}" iyy="{inertia}" izz="{inertia}" ixy="0" ixz="0" iyz="0"/></inertial>
      <collision><geometry><box size=".02 .02 .02"/></geometry></collision></link>'''


def scene_config(tmp_path, robot, settings, **options):
    tmp_path.mkdir(parents=True, exist_ok=True)
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text(robot)
    bindings = tmp_path / 'robot.json'
    bindings.write_text(json.dumps(settings))
    field = tmp_path / 'field.json'
    field.write_text(json.dumps({'field': {'size': [2, 2], 'floor': True, 'walls': False}, 'objects': []}))
    return {'physics_backend': 'physx_cpu', 'physics_hz': 240, 'physics_solver_iterations': 100,
            'robot_urdf': str(urdf), 'robot_config': str(bindings), 'game_pack': str(field),
            'start_pose': [0, 0, 1, 37], **options}


def flywheel_config(tmp_path, cartridge='green', rpm=200, *, ratio=1, reversed=False):
    inertia = .02 * (100 / rpm) ** 2
    robot = ('<robot name="flywheel">' + inertia_link('base', .1, 1) + inertia_link('rotor', inertia) + '''
      <joint name="shaft" type="continuous"><parent link="base"/><child link="rotor"/>
      <origin xyz="0 0 .2" rpy=".3 .2 -.4"/><axis xyz="0 -1 0"/>
      <limit effort="10" velocity="200"/></joint></robot>''')
    settings = {'fixed_base': True, 'motors': [{'port': 1, 'joint': 'shaft',
                'cartridge': cartridge, 'gear_ratio': ratio, 'reversed': reversed, 'efficiency': 1.0}]}
    return scene_config(tmp_path, robot, settings, gravity=0), inertia


@pytest.mark.parametrize('reversed', [False, True])
def test_torque_accelerates_known_inertia_on_rotated_axis_with_gearing(tmp_path, reversed):
    config, inertia = flywheel_config(tmp_path, ratio=3, reversed=reversed)
    with PhysXWorld(config) as world:
        initial = world.snapshot()
        torque = .03
        result = world.step({'motors': {'1': {'mode': 'torque', 'value': torque}}})
        sign = -1 if reversed else 1
        expected_joint_velocity = sign * torque * 3 / inertia * world.dt
        joint = result['joints'][0]
        assert joint['velocity'] == pytest.approx(expected_joint_velocity, rel=.015)
        assert result['motors']['1']['velocity'] == pytest.approx(abs(expected_joint_velocity) * 3, rel=.015)
        # A correct scalar encoder alone does not establish world-axis mapping.
        # Check actual link angular velocity against base yaw and URDF origin.
        p = world._physics
        base_q = p.getQuaternionFromEuler([0, 0, math.radians(37)])
        origin_q = p.getQuaternionFromEuler([.3, .2, -.4])
        frame_q = p.multiplyTransforms([0, 0, 0], base_q, [0, 0, 0], origin_q)[1]
        axis_world = np.asarray(p.getMatrixFromQuaternion(frame_q)).reshape(3, 3) @ [0, -1, 0]
        link_velocity = p.getLinkState(world.robot, 0, computeLinkVelocity=1)[7]
        np.testing.assert_allclose(link_velocity, axis_world * expected_joint_velocity, rtol=.015, atol=1e-6)
        assert result['pose'] == pytest.approx(initial['pose'], abs=1e-7)


@pytest.mark.parametrize('cartridge,rpm', [('red', 100), ('green', 200), ('blue', 600)])
def test_physical_cartridge_reaches_its_speed_cap_and_torque_replaces_drive(tmp_path, cartridge, rpm):
    config, _ = flywheel_config(tmp_path, cartridge, rpm)
    with PhysXWorld(config) as world:
        tail = []
        for tick in range(360):
            snapshot = world.step({'motors': {'1': {'mode': 'voltage', 'value': 12000}}})
            if tick >= 300:
                tail.append(snapshot['motors']['1']['velocity'] * 30 / math.pi)
        assert np.mean(tail) == pytest.approx(rpm, rel=.025)
        assert max(tail) < 1.05 * rpm
        assert snapshot['motors']['1']['current_estimate'] <= 2.5
        # An accidentally retained velocity servo defeats this real deceleration.
        stall = world.motor_bindings[0]['stall_torque']
        for _ in range(60):
            snapshot = world.step({'motors': {'1': {'mode': 'torque', 'value': -.1 * stall}}})
        assert snapshot['motors']['1']['velocity'] < .9 * rpm * math.pi / 30


def test_pneumatic_and_manual_position_drives_respect_force_speed_and_limits(tmp_path):
    robot = ('<robot name="slides">' + inertia_link('base', .1, 1) + inertia_link('lift', .02, .5)
             + inertia_link('carriage', .02, 1) + '''
      <joint name="piston" type="prismatic"><parent link="base"/><child link="lift"/>
       <origin xyz=".1 0 .1"/><axis xyz="0 0 1"/><limit lower="0" upper=".1" effort="40" velocity=".25"/></joint>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="carriage"/>
       <origin xyz="-.1 0 .1"/><axis xyz="1 0 0"/><limit lower="0" upper=".08" effort="5" velocity=".2"/></joint>
      </robot>''')
    settings = {'fixed_base': True, 'motors': [], 'pneumatics': [
        {'port': 'A', 'joint': 'piston', 'extended': .09, 'retracted': 0, 'force': 40, 'max_velocity': .25}]}
    config = scene_config(tmp_path, robot, settings)
    with PhysXWorld(config) as world:
        start = {joint['name']: joint for joint in world.joints}
        first = world.step({'adi': {'A': True}, 'joints': {'slide': {'mode': 'position', 'value': .08, 'force': .4}}})
        state = {joint['name']: joint for joint in first['joints']}
        # F=0.4 N on a 1 kg horizontal slider permits only F*dt/m velocity.
        assert state['slide']['velocity'] <= .4 * world.dt * 1.02
        assert state['slide']['position'] - start['slide']['position'] < .00002
        maximum_piston_speed = abs(state['piston']['velocity'])
        for _ in range(240):
            result = world.step({'adi': {'A': True}, 'joints': {'slide': {'mode': 'position', 'value': .5, 'force': 5}}})
            state = {joint['name']: joint for joint in result['joints']}
            maximum_piston_speed = max(maximum_piston_speed, abs(state['piston']['velocity']))
        assert state['piston']['position'] == pytest.approx(.09, abs=.001)
        assert maximum_piston_speed <= .25 * 1.02
        assert state['slide']['position'] == pytest.approx(.08, abs=.001)
        for _ in range(160):
            result = world.step({'adi': {'A': False}})
        assert next(j['position'] for j in result['joints'] if j['name'] == 'piston') == pytest.approx(0, abs=.001)


def test_independent_workers_noise_polling_json_snapshot_and_fixed_clock(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config['noise'] = {'encoder_std_deg': .2, 'position_std_m': .003,
                       'heading_std_deg': .5, 'latency_ms': 12, 'dropout': .2}
    with PhysXWorld(config) as first, PhysXWorld(deepcopy(config)) as second:
        assert first._process.process.pid != second._process.process.pid
        assert first.snapshot() == second.snapshot()
        assert first.time == first.dt  # The SDK bootstrap is an honest fixed tick.
        clock = FixedStepClock(240)
        clock.add(.05)
        ticks = 0
        while (count := clock.take(5)):
            for _ in range(count):
                command = {'motors': {'1': {'mode': 'torque', 'value': .02}}}
                a = first.step(command)
                for _ in range(7):
                    assert first.snapshot() == a
                b = second.step(command)
                assert a == b
                json.dumps(a, allow_nan=False)
                ticks += 1
        assert ticks == 12
        assert first.time == pytest.approx(13 / 240)
        assert first.snapshot()['sensor_age_ms'] >= 12 - 1e-6
        unchanged = second.snapshot()
        first.step({'motors': {'1': {'mode': 'torque', 'value': -.02}}})
        assert second.snapshot() == unchanged
        first_process = first._process.process
        first.close()
        assert not first_process.is_alive()
        # Closing one world cannot tear down the other worker's native runtime.
        assert second.step()['tick'] == unchanged['tick'] + 1
        for name in ('resetBaseVelocity', 'resetJointState', 'changeDynamics', 'getAABB'):
            with pytest.raises(NotImplementedError, match='geometry shadow is read-only'):
                getattr(second._physics, name)


def distance_fixture(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config['start_pose'] = [0, 0, 1, 0]
    config['field_objects'] = [{'name': 'wall', 'shape': 'box', 'size': [.1, 1, 1], 'position': [1, 0, 1]}]
    from pathlib import Path
    path = Path(config['robot_config'])
    settings = json.loads(path.read_text())
    settings['distance_sensors'] = [
        {'port': 2, 'position': [.15, 0, 0], 'direction': [1, 0, 0], 'max_range': 2},
        {'port': 3, 'position': [0, .15, 0], 'direction': [0, 1, 0], 'max_range': 2},
        {'port': 4, 'position': [0, 0, 0], 'direction': [1, 0, 0], 'max_range': 2},
        {'port': 5, 'position': [.15, 0, .2], 'direction': [-1, 0, 0], 'max_range': .2}]
    path.write_text(json.dumps(settings))
    return config


def test_distances_measure_wall_range_and_self_occlusion_without_fabricated_ids(tmp_path):
    from eersim.physics import World
    config = distance_fixture(tmp_path)
    with World(config) as bullet_world:
        reference = bullet_world.snapshot()['distance']
    with PhysXWorld(config) as world:
        state = world.snapshot()
        distances = state['distance']
        assert distances['2'] == pytest.approx(800, abs=.01)
        assert distances['2'] == pytest.approx(reference['2'], abs=.01)
        assert distances['3'] == 9999
        # The rotated own-link collider remains a legitimate visible obstacle.
        assert 130 < distances['5'] < 150
        assert distances['5'] == pytest.approx(reference['5'], abs=.01)
        # A physically embedded sensor returns initial overlap, not a wall
        # reading through its own solid. Bullet skips an initial interior hit.
        assert distances['4'] == 0
        assert reference['4'] == pytest.approx(950, abs=.01)
        starts = [[.15, 0, 1], [0, .15, 1], [.15, 0, 1.2], [.15, 0, 1]]
        ends = [[2.15, 0, 1], [0, 2.15, 1], [-.05, 0, 1.2], [.65, 0, 1]]
        hits = world._physics.rayTestBatch(starts, ends)
        assert hits[0][0] > 0 and hits[2][0] > 0
        assert hits[0][0] != hits[2][0]  # Actual distinct native identities.
        assert hits[1][0] == hits[3][0] == -1  # No hit and beyond maximum range.
        assert hits[0][2] == pytest.approx(.4, abs=1e-6)
        assert hits[0][3] == pytest.approx([.95, 0, 1], abs=1e-6)
        assert hits[0][4] == pytest.approx([-1, 0, 0], abs=1e-6)
        for start, end, hit in zip(starts, ends, hits):
            assert world._physics.rayTest(start, end) == [hit]
        assert world._physics.rayTest([0, 0, 1], [0, 0, 1])[0][0] == -1
        with pytest.raises(ValueError, match='finite'):
            world._physics.rayTest([math.nan, 0, 0], [1, 0, 0])
        assert world.snapshot() == state
        assert world._process.process.is_alive()
        assert 'native opaque' in state['physics']['scene_query_identity']


def test_distance_queries_follow_live_falling_obstacle_not_bullet_geometry_shadow(tmp_path):
    import pybullet as bullet
    config = distance_fixture(tmp_path)
    config['gravity'] = 9.80665
    config['field_objects'] = [{'name': 'falling', 'shape': 'box', 'mass': 1,
                                'size': [.1, .2, .2], 'position': [1, 0, 1.6]}]
    with PhysXWorld(config) as world:
        body = next(item['id'] for item in world.field_objects if item['name'] == 'falling')
        assert world.snapshot()['distance']['2'] == 9999
        for _ in range(82):
            result = world.step()
        assert result['distance']['2'] == pytest.approx(800, abs=.1)
        assert bullet.getBasePositionAndOrientation(body, physicsClientId=world.client)[0][2] == pytest.approx(1.6)
        assert world._physics.getBasePositionAndOrientation(body)[0][2] < 1.1


@pytest.mark.parametrize('robot_name,port', [('xdrive', '2'), ('tank', '6')])
def test_bundled_distance_mount_is_beyond_the_flap_envelope(tmp_path, robot_name, port):
    from eersim.config import ASSETS
    config = distance_fixture(tmp_path)
    config.update(robot_urdf=str(ASSETS / 'robots' / f'{robot_name}.urdf'),
                  robot_config=str(ASSETS / 'robots' / f'{robot_name}.json'), field_objects=[])
    # The flap spans local x=.15..27 and z=.049..061 at rest. Its old sensor
    # origin(.23,0,.06) was inside the solid; the lens is now 10 mm in front.
    with PhysXWorld(config) as world:
        assert world.snapshot()['distance'][port] == 9999
        sensor = world.robot_settings['distance_sensors'][0]
        assert sensor['position'][0] > .27


@pytest.mark.parametrize('frame', ['world', 'local'])
@pytest.mark.parametrize('engine', ['bullet', 'physx'])
def test_external_force_respects_rotated_inertial_frame_and_moment_arm(tmp_path, frame, engine):
    from eersim.physics import World
    import pybullet as bullet
    config, _ = flywheel_config(tmp_path)
    prop = tmp_path / 'offset_body.urdf'
    prop.write_text('''<robot name="offset_prop"><link name="body">
      <inertial><origin xyz=".03 -.02 .01" rpy=".2 -.3 .4"/><mass value="2"/>
      <inertia ixx=".02" iyy=".03" izz=".04" ixy="0" ixz="0" iyz="0"/></inertial>
      <collision><geometry><box size=".12 .08 .06"/></geometry></collision></link></robot>''')
    config['field_objects'] = [{'name': 'offset', 'urdf': str(prop), 'fixed': False,
                               'position': [.4, .3, 1], 'rotation': [.4, .2, -.6]}]
    with (PhysXWorld if engine == 'physx' else World)(config) as world:
        body = next(item['id'] for item in world.field_objects if item['name'] == 'offset')
        position, quaternion = world._physics.getBasePositionAndOrientation(body)
        rotation = np.asarray(bullet.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
        inertia_world = rotation @ np.diag([.02, .03, .04]) @ rotation.T
        if frame == 'world':
            force = np.array([.7, -.2, .4])
            arm = np.array([.04, .03, -.02])
            point = np.asarray(position) + arm
            world._physics.applyExternalForce(body, -1, force, point, bullet.WORLD_FRAME)
        else:
            # Bullet LINK_FRAME means the link's COM/principal frame. Force at
            # local zero has no moment arm, even with an offset URDF inertia.
            force = rotation @ [1., 0., 0.]
            arm = np.zeros(3)
            world._physics.applyExternalForce(body, -1, [1., 0., 0.], [0., 0., 0.], bullet.LINK_FRAME)
        world.step()
        linear, angular = world._physics.getBaseVelocity(body)
        np.testing.assert_allclose(linear, force / 2 * world.dt, rtol=.005, atol=1e-6)
        expected_angular = np.linalg.solve(inertia_world, np.cross(arm, force)) * world.dt
        np.testing.assert_allclose(angular, expected_angular, rtol=.01, atol=1e-6)
