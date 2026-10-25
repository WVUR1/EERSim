"""Native MuJoCo regressions: effort, contact, sensor and shadow authority."""
from copy import deepcopy
import json
import math

import numpy as np
import pytest

pytest.importorskip('mujoco')
pytest.importorskip('pybullet')
import pybullet as bullet

from eersim.config import ASSETS, default_config
from eersim.mujoco_world import MuJoCoWorld


def inertia_link(name, inertia=.005, mass=.5):
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
    return {'physics_backend': 'mujoco', 'physics_hz': 240, 'physics_solver_iterations': 100,
            'robot_urdf': str(urdf), 'robot_config': str(bindings), 'game_pack': str(field),
            'start_pose': [0, 0, 1, 37], **options}


def flywheel_config(tmp_path, cartridge='green', rpm=200, *, ratio=1, reverse=False, inertia=None):
    inertia = .02 * (100 / rpm) ** 2 if inertia is None else inertia
    robot = ('<robot name="flywheel">' + inertia_link('base', .1, 1) + inertia_link('rotor', inertia) + '''
      <joint name="shaft" type="continuous"><parent link="base"/><child link="rotor"/>
      <origin xyz="0 0 .2" rpy=".3 .2 -.4"/><axis xyz="0 -1 0"/>
      <limit effort="10" velocity="200"/></joint></robot>''')
    settings = {'fixed_base': True, 'motors': [{'port': 1, 'joint': 'shaft',
                'cartridge': cartridge, 'gear_ratio': ratio, 'reversed': reverse, 'efficiency': 1.0}]}
    return scene_config(tmp_path, robot, settings, gravity=0), inertia


@pytest.mark.parametrize('reverse', [False, True])
def test_torque_accelerates_known_inertia_rotated_axis_and_gearing(tmp_path, reverse):
    config, inertia = flywheel_config(tmp_path, ratio=3, reverse=reverse)
    with MuJoCoWorld(config) as world:
        initial = world.snapshot()
        torque = .03
        result = world.step({'motors': {'1': {'mode': 'torque', 'value': torque}}})
        expected = (-1 if reverse else 1) * torque * 3 / inertia * world.dt
        assert result['joints'][0]['velocity'] == pytest.approx(expected, rel=.015)
        assert result['motors']['1']['velocity'] == pytest.approx(abs(expected) * 3, rel=.015)
        p = world._physics
        frame_q = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler([0, 0, math.radians(37)]),
                                       [0, 0, 0], p.getQuaternionFromEuler([.3, .2, -.4]))[1]
        axis = np.asarray(p.getMatrixFromQuaternion(frame_q)).reshape(3, 3) @ [0, -1, 0]
        np.testing.assert_allclose(p.getLinkState(world.robot, 0)[7], axis * expected, rtol=.015, atol=1e-6)
        assert result['pose'] == pytest.approx(initial['pose'], abs=1e-7)


@pytest.mark.parametrize('cartridge,rpm', [('red', 100), ('green', 200), ('blue', 600)])
def test_cartridge_speed_current_and_continuous_encoder(tmp_path, cartridge, rpm):
    config, _ = flywheel_config(tmp_path, cartridge, rpm)
    with MuJoCoWorld(config) as world:
        positions, speeds = [], []
        for tick in range(480):
            snapshot = world.step({'motors': {'1': {'mode': 'voltage', 'value': 12000}}})
            positions.append(snapshot['motors']['1']['position'])
            if tick >= 420:
                speeds.append(snapshot['motors']['1']['velocity'] * 30 / math.pi)
        assert np.mean(speeds) == pytest.approx(rpm, rel=.025)
        assert max(speeds) < rpm * 1.05
        assert snapshot['motors']['1']['current_estimate'] <= 2.5
        assert positions[-1] > 4 * math.pi
        assert np.min(np.diff(positions)) >= 0
        assert np.max(np.diff(positions)) < rpm * math.pi / 30 * world.dt * 1.06
        for _ in range(60):
            snapshot = world.step({'motors': {'1': {'mode': 'torque', 'value': -.1 * world.motor_bindings[0]['stall_torque']}}})
        assert snapshot['motors']['1']['velocity'] < .9 * rpm * math.pi / 30


def test_pneumatic_position_actuation_has_finite_force_speed_and_joint_limits(tmp_path):
    robot = ('<robot name="slides">' + inertia_link('base', .1, 1) + inertia_link('lift', .02, .5)
             + inertia_link('carriage', .02, 1) + '''
      <joint name="piston" type="prismatic"><parent link="base"/><child link="lift"/>
       <origin xyz=".1 0 .1"/><axis xyz="0 0 1"/><limit lower="0" upper=".1" effort="40" velocity=".25"/></joint>
      <joint name="slide" type="prismatic"><parent link="base"/><child link="carriage"/>
       <origin xyz="-.1 0 .1"/><axis xyz="1 0 0"/><limit lower="0" upper=".08" effort="5" velocity=".2"/></joint></robot>''')
    settings = {'fixed_base': True, 'motors': [], 'pneumatics': [
        {'port': 'A', 'joint': 'piston', 'extended': .09, 'retracted': 0, 'force': 40, 'max_velocity': .25}]}
    with MuJoCoWorld(scene_config(tmp_path, robot, settings)) as world:
        first = world.step({'adi': {'A': True}, 'joints': {'slide': {'mode': 'position', 'value': .08, 'force': .4}}})
        state = {joint['name']: joint for joint in first['joints']}
        assert state['slide']['velocity'] <= .4 * world.dt * 1.02
        speeds = [state['piston']['velocity']]
        for _ in range(240):
            result = world.step({'adi': {'A': True}, 'joints': {'slide': {'mode': 'position', 'value': .5, 'force': 5}}})
            state = {joint['name']: joint for joint in result['joints']}
            speeds.append(state['piston']['velocity'])
        assert state['piston']['position'] == pytest.approx(.09, abs=.001)
        assert max(speeds) <= .25 * 1.02
        assert state['slide']['position'] == pytest.approx(.08, abs=.001)


def practice_config():
    config = default_config()
    config.update(physics_backend='mujoco', game_pack=str(ASSETS / 'games' / 'practice.json'),
                  start_pose=[-1.2, -1.2, .085, 0])
    return config


def drive(world, forward=0, right=0, clockwise=0):
    demand = {'front_left': forward + right + clockwise,
              'front_right': forward - right - clockwise,
              'back_left': forward - right + clockwise,
              'back_right': forward + right - clockwise}
    return {'motors': {str(m['port']): {'mode': 'voltage', 'value': demand.get(m['joint'], 0) * m['sign']}
                       for m in world.motor_bindings}}


def test_omni_drive_uses_native_ground_contacts_and_shadow_never_steps(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('Bullet must never integrate in MuJoCo mode')
    monkeypatch.setattr(bullet, 'stepSimulation', forbidden)
    with MuJoCoWorld(practice_config()) as world:
        for _ in range(120):
            world.step()
        contacts = world._physics.getContactPoints(bodyA=world.robot)
        ground = [c for c in contacts if c[7][2] > .5 and c[9] > 0]
        assert ground
        mass = sum(world._physics.getDynamicsInfo(world.robot, link)[0]
                   for link in range(-1, world._physics.getNumJoints(world.robot)))
        assert sum(c[9] for c in ground) == pytest.approx(mass * world.config.gravity, rel=.15)
        start = world.snapshot()['pose']
        for _ in range(180):
            result = world.step(drive(world, forward=5000))
        assert result['pose'][0] > start[0] + .1
        assert abs(result['pose'][1] - start[1]) < .1
        assert abs(result['pose'][3]) < .2
        shadow = bullet.getBasePositionAndOrientation(world.robot, physicsClientId=world.client)[0]
        assert shadow[0] == pytest.approx(-1.2)
        world._physics.sync_render_geometry()
        shadow = bullet.getBasePositionAndOrientation(world.robot, physicsClientId=world.client)[0]
        assert shadow == pytest.approx(result['pose'][:3], abs=1e-6)


def test_airborne_wheels_cannot_create_ground_traction():
    config = practice_config()
    config.update(gravity=0, start_pose=[-1.2, -1.2, 1, 0])
    with MuJoCoWorld(config) as world:
        start = world.snapshot()['pose']
        for _ in range(60):
            end = world.step(drive(world, forward=8000))
        assert end['pose'][:2] == pytest.approx(start[:2], abs=.015)


def test_native_rays_follow_live_state_and_map_import_ids(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config.update(gravity=9.80665, field_objects=[{'name': 'falling', 'shape': 'box', 'mass': 1,
                  'size': [.1, .2, .2], 'position': [1, 0, 1.6]}])
    with MuJoCoWorld(config) as world:
        adapter = world._physics
        body = next(obj['id'] for obj in world.field_objects if obj['name'] == 'falling')
        start, end = [.15, 0, 1], [2.15, 0, 1]
        assert adapter.rayTest(start, end)[0][0] == -1
        for _ in range(82):
            world.step()
        hit = adapter.rayTest(start, end)[0]
        assert hit[:2] == (body, -1)
        assert hit[2] == pytest.approx(.4, abs=1e-6)
        assert hit[4] == pytest.approx([-1, 0, 0], abs=1e-6)
        assert bullet.getBasePositionAndOrientation(body, physicsClientId=world.client)[0][2] == pytest.approx(1.6)
        with pytest.raises(ValueError, match='finite'):
            adapter.rayTest([math.nan, 0, 0], end)


def test_noise_clock_independence_disable_reset_and_cpu_render(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config['noise'] = {'encoder_std_deg': .2, 'position_std_m': .003, 'heading_std_deg': .5,
                       'latency_ms': 12, 'dropout': .2}
    with MuJoCoWorld(config) as a, MuJoCoWorld(deepcopy(config)) as b:
        initial = a.snapshot()
        assert initial == b.snapshot()
        for tick in range(24):
            command = {'motors': {'1': {'mode': 'voltage', 'value': 3000}}}
            state = a.step(command)
            for _ in range(4):
                assert state == a.snapshot()
            assert state == b.step(command)
        assert state['time'] == pytest.approx(.1)
        assert a.render(32, 32).shape == (32, 32, 4)
        a.disable()
        assert np.all(a._physics.model.actuator_forcerange == 0)
        old = a._physics
        assert a.reset() == initial
        assert a._physics is not old
        assert b.step()['tick'] == 25
        with pytest.raises(NotImplementedError, match='shadow is read-only'):
            a._physics.resetBaseVelocity(a.robot, [1, 0, 0])


def test_contact_friction_preserves_material_product(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config.update(field_objects=[{'name': 'block', 'shape': 'box', 'mass': 1,
                  'size': [.1, .1, .1], 'position': [.5, 0, .049], 'friction': .5}])
    with MuJoCoWorld(config) as world:
        world.step()
        adapter = world._physics
        body = next(obj['id'] for obj in world.field_objects if obj['name'] == 'block')
        adapter._index_contacts()
        rows = np.flatnonzero(np.any(adapter._contact_keys[:, :2] == body, axis=1))
        assert len(rows)
        assert adapter.data.contact.friction[rows, 0] == pytest.approx(.5 * world.config.friction)


def test_fixed_base_retains_imported_inertial_frame_for_render_and_replay(tmp_path):
    robot = '<robot name="offset"><link name="base"><inertial><origin xyz=".03 -.02 .05" rpy=".1 .2 .3"/><mass value="1"/><inertia ixx=".01" iyy=".02" izz=".03" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><box size=".1 .1 .1"/></geometry></collision></link></robot>'
    config = scene_config(tmp_path, robot, {'fixed_base': True, 'motors': []})
    with MuJoCoWorld(config) as world:
        expected = bullet.getBasePositionAndOrientation(world.robot, physicsClientId=world.client)
        assert world._physics.getDynamicsInfo(world.robot)[0] == 0
        for _ in range(3):
            world.step()
            actual = world._physics.getBasePositionAndOrientation(world.robot)
            assert actual[0] == pytest.approx(expected[0], abs=2e-7)
            assert actual[1] == pytest.approx(expected[1], abs=2e-7)
            assert world.snapshot()['pose'][:3] == pytest.approx(expected[0], abs=2e-7)


@pytest.mark.parametrize('hz', [120, 240])
def test_light_motor_reversal_and_braking_remain_bounded_without_velocity_resets(tmp_path, hz):
    config, _ = flywheel_config(tmp_path, 'blue', 600, inertia=.000001)
    config['physics_hz'] = hz
    with MuJoCoWorld(config) as world:
        for direction in (1, -1):
            for _ in range(hz // 2):
                state = world.step({'motors': {'1': {'mode': 'voltage', 'value': direction * 12000}}})
                assert abs(state['motors']['1']['velocity']) <= 600 * math.pi / 30 * 1.01
                assert abs(state['motors']['1']['torque']) <= world.motor_bindings[0]['stall_torque']
            assert state['motors']['1']['velocity'] == pytest.approx(direction * 600 * math.pi / 30, rel=.025)
        for _ in range(hz // 4):
            state = world.step({'motors': {'1': {'mode': 'voltage', 'value': 0, 'brake': 'brake'}}})
        assert abs(state['motors']['1']['velocity']) < .01


def test_claw_grip_does_not_excite_braked_drive_motors():
    from eersim.demos import claw_demo_config
    config = claw_demo_config()
    config.update(physics_backend='mujoco', physics_hz=120,
                  game_pack=str(ASSETS / 'games' / 'practice.json'))
    commands = {'motors': {str(port): {'mode': 'voltage', 'value': 0,
                           'brake': 'brake' if port < 5 else 'hold'} for port in range(1, 7)}}
    with MuJoCoWorld(config) as world:
        for _ in range(20):
            world.step(commands)
        start = world.snapshot()['pose']
        commands['motors']['6'] = {'mode': 'position', 'value': -144, 'velocity': 100, 'brake': 'hold'}
        peak_speed = 0
        for _ in range(300):
            state = world.step(commands)
            peak_speed = max(peak_speed, *(abs(state['motors'][str(port)]['velocity']) for port in range(1, 5)))
        assert math.dist(start[:2], state['pose'][:2]) < .005
        assert peak_speed * 30 / math.pi < 10
        assert state['motors']['6']['position'] < 0


def test_cached_static_flex_stage_matches_stock_pipeline_with_dynamic_contacts(tmp_path):
    config, _ = flywheel_config(tmp_path)
    mesh = tmp_path / 'plate.obj'
    mesh.write_text('v 0 -.5 0\nv 1 -.5 0\nv 1 .5 0\nv 0 .5 0\nf 1 2 3\nf 1 3 4\n')
    urdf = tmp_path / 'plate.urdf'
    urdf.write_text('<robot name="plate"><link name="plate" concave="yes"><inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry><mesh filename="plate.obj"/></geometry></collision></link></robot>')
    config.update(gravity=9.80665, field_objects=[
        {'name': 'plate', 'urdf': str(urdf), 'fixed': True, 'position': [0, 0, .2]},
        {'name': 'block', 'shape': 'sphere', 'mass': 1, 'radius': .05,
         'position': [.5, 0, .5], 'friction': .5, 'restitution': .5}])
    with MuJoCoWorld(config) as cached, MuJoCoWorld(deepcopy(config)) as stock:
        a, b = cached._physics, stock._physics
        # This optimization is deliberately limited to the non-sleep pipeline.
        import mujoco
        for adapter in (a, b):
            adapter.model.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_SLEEP)
        assert a._can_cache_position_stage()
        b._cached_position_stage = False
        plate = next(obj['id'] for obj in cached.field_objects if obj['name'] == 'plate')
        block = next(obj['id'] for obj in cached.field_objects if obj['name'] == 'block')
        saw_flex_contact = False
        for tick in range(180):
            command = {'motors': {'1': {'mode': 'voltage', 'value': 8000 if tick < 90 else -4000}}}
            if tick == 90:
                for world in (cached, stock):
                    world._physics.applyExternalForce(block, -1, [15, 3, 0], [.5, 0, .25], bullet.WORLD_FRAME)
            cached.step(command)
            stock.step(command)
            np.testing.assert_allclose(a.data.qpos, b.data.qpos, rtol=0, atol=1e-12)
            np.testing.assert_allclose(a.data.qvel, b.data.qvel, rtol=0, atol=1e-12)
            np.testing.assert_allclose(a.data.qacc, b.data.qacc, rtol=0, atol=1e-10)
            assert a.data.ncon == b.data.ncon
            np.testing.assert_allclose(a.data.efc_force, b.data.efc_force, rtol=0, atol=1e-10)
            assert a.getContactPoints() == b.getContactPoints()
            saw_flex_contact |= a.has_contact(plate, block)
        assert saw_flex_contact
        np.testing.assert_array_equal(a.data.flexvert_xpos, b.data.flexvert_xpos)
        assert a.rayTest([.25, 0, .8], [.25, 0, .1]) == b.rayTest([.25, 0, .8], [.25, 0, .1])


def test_native_warning_fails_run_instead_of_accepting_finite_reset_state(tmp_path):
    import mujoco
    config, _ = flywheel_config(tmp_path)
    with MuJoCoWorld(config) as world:
        # Native auto-reset can leave finite qpos/qvel and time=0; the warning
        # remains the evidence that the trajectory is invalid.
        world._physics.data.warning.number[mujoco.mjtWarning.mjWARN_BADQACC] = 1
        with pytest.raises(RuntimeError, match='mjWARN_BADQACC'):
            world.step()


def test_reset_and_close_join_native_workers_even_if_old_data_is_retained(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config['physics_cpu_threads'] = 2
    with MuJoCoWorld(config) as world:
        old = world._physics
        assert bool(old.data.threadpool) is bool(old.scene.cpu_threads)
        world.reset()
        assert old.data.threadpool == 0
        current = world._physics
        assert bool(current.data.threadpool) is bool(current.scene.cpu_threads)
        world.close()
        assert current.data.threadpool == 0


def test_import_uses_effective_robot_joint_damping_and_preserves_field_damping(tmp_path):
    from eersim.demos import claw_demo_config
    field_config, _ = flywheel_config(tmp_path)
    from pathlib import Path
    field_robot = Path(field_config['robot_urdf'])
    field_robot.write_text(field_robot.read_text().replace('<limit effort="10"', '<dynamics damping=".007" friction="0"/><limit effort="10"'))
    config = claw_demo_config()
    config.update(game_pack=str(ASSETS / 'games' / 'practice.json'),
                  field_objects=[{'name': 'fixture', 'urdf': str(field_robot), 'fixed': True, 'position': [0, 0, 1]}])
    with MuJoCoWorld(config) as world:
        adapter = world._physics
        for name in ('front_left', 'front_right', 'back_left', 'back_right', 'lift', 'claw'):
            index = world._joint_info[name][0]
            joint = adapter.joint_ids[world.robot, index]
            expected = .0001 if name not in ('lift', 'claw') else .001
            assert adapter.model.dof_damping[adapter.model.jnt_dofadr[joint]] == pytest.approx(expected)
        body = next(obj['id'] for obj in world.field_objects if obj['name'] == 'fixture')
        joint = adapter.joint_ids[body, 0]
        assert adapter.model.dof_damping[adapter.model.jnt_dofadr[joint]] == pytest.approx(.007)


def test_body_drag_dissipates_energy_and_matches_configured_decay_rate(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config.update(field_objects=[{'name': 'ball', 'shape': 'sphere', 'radius': .1, 'mass': 2,
                                  'position': [.5, 0, 1]}])
    with MuJoCoWorld(config) as world:
        adapter = world._physics
        body = next(obj['id'] for obj in world.field_objects if obj['name'] == 'ball')
        native = adapter.body_ids[body, -1]
        assert dict(adapter._body_drag)[native] == .04
        robot_joints = [native for (source, _), native in adapter.body_ids.items() if source == world.robot]
        assert all(rate == .02 for native, rate in adapter._body_drag if native in robot_joints)
        adapter.applyExternalForce(body, -1, [120, 0, 0], [.5, 0, 1], bullet.WORLD_FRAME)
        adapter.applyExternalTorque(body, -1, [0, 0, 8], bullet.WORLD_FRAME)
        world.step()
        initial_linear, initial_angular = adapter.getBaseVelocity(body)
        energy = []
        for _ in range(120):
            world.step()
            linear, angular = adapter.getBaseVelocity(body)
            energy.append(.5 * adapter.model.body_mass[native] * np.dot(linear, linear)
                          + .5 * adapter.model.body_inertia[native, 0] * np.dot(angular, angular))
        decay = (1 - .04 * world.dt) ** 120
        np.testing.assert_allclose(linear, initial_linear * decay, rtol=1e-10, atol=1e-12)
        np.testing.assert_allclose(angular, initial_angular * decay, rtol=1e-10, atol=1e-12)
        assert np.all(np.diff(energy) < 0)


def test_native_sleep_preserves_passive_drag_and_robot_stays_awake(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config['field_objects'] = [
        {'name': 'rest', 'shape': 'sphere', 'radius': .03, 'mass': 1, 'position': [.5, -.5, 1]},
        {'name': 'moving', 'shape': 'sphere', 'radius': .03, 'mass': 1, 'position': [.5, .5, 1]}]
    with MuJoCoWorld(config) as world:
        p = world._physics
        bodies = {item['name']: item['id'] for item in world.field_objects}
        rest = p.body_ids[bodies['rest'], -1]
        moving = p.body_ids[bodies['moving'], -1]
        moving_dof = p.model.jnt_dofadr[p.scene.free_joints[bodies['moving']]]
        robot_trees = {int(p.model.body_treeid[native]) for (body, _), native in p.body_ids.items()
                       if body == world.robot and p.model.body_treeid[native] >= 0}
        p.applyExternalForce(bodies['moving'], -1, [120, 0, 0], [.5, .5, 1], bullet.WORLD_FRAME)
        world.step()
        saw_sleep = False
        for _ in range(180):
            before = p.getBaseVelocity(bodies['moving'])[0].copy()
            world.step()
            after = p.getBaseVelocity(bodies['moving'])[0]
            np.testing.assert_allclose(after, before * (1 - .04 * world.dt), rtol=1e-10, atol=1e-12)
            assert p.data.qacc[moving_dof] == pytest.approx(-.04 * before[0], abs=1e-10)
            assert np.all(p.data.xfrc_applied == 0)
            assert all(p.data.tree_awake[tree] for tree in robot_trees)
            saw_sleep |= not p.data.tree_awake[p.model.body_treeid[rest]]
        assert saw_sleep
        status = world.snapshot()['physics']['sleeping']
        assert status['enabled'] and status['sleeping_trees'] == 1
        assert status['contact_count'] == p.data.ncon
        assert p.data.tree_awake[p.model.body_treeid[moving]]


def test_native_prop_sleeps_and_external_force_wakes_it(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config.update(gravity=9.80665, field_objects=[
        {'name': 'rest', 'shape': 'sphere', 'radius': .05, 'mass': 1, 'position': [.5, 0, .15]}])
    with MuJoCoWorld(config) as world:
        p = world._physics
        body = next(item['id'] for item in world.field_objects if item['name'] == 'rest')
        native = p.body_ids[body, -1]
        tree = p.model.body_treeid[native]
        for _ in range(1200):
            world.step()
            if not p.data.tree_awake[tree]:
                break
        assert not p.data.tree_awake[tree]
        position = p.getBasePositionAndOrientation(body)[0]
        for _ in range(30):
            world.step()
        assert p.getBasePositionAndOrientation(body)[0] == position
        assert p.has_contact(body, world.field_objects[0]['id'])
        # The native solver's active contact set may omit a sleeping floor pair.
        assert all(contact[9] >= 0 for contact in p.getContactPoints(bodyA=body))
        p.applyExternalForce(body, -1, [0, 0, 120], position, bullet.WORLD_FRAME)
        world.step()
        assert p.data.tree_awake[tree]
        assert p.getBaseVelocity(body)[0][2] > .3
        for _ in range(10):
            world.step()
        assert p.getBasePositionAndOrientation(body)[0][2] > position[2] + .003
        world.reset()
        assert not world._physics._sleep_contacts
        assert world.snapshot()['physics']['sleeping']['sleeping_trees'] == 0


def test_sleeping_scored_pin_wakes_on_robot_contact_and_does_not_reuse_forces(tmp_path):
    robot = '''<robot name="pusher">
      <link name="base"><inertial><mass value="1"/>
        <inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>'''
    robot += inertia_link('finger', .001, .1) + '''
      <joint name="reach" type="prismatic"><parent link="base"/><child link="finger"/>
        <origin xyz=".15 0 .12"/><axis xyz="-1 0 0"/>
        <limit lower="0" upper=".18" effort="20" velocity=".1"/></joint></robot>'''
    config = scene_config(tmp_path, robot, {'fixed_base': True, 'motors': []},
                          gravity=9.80665, start_pose=[0, 0, 0, 0])
    game = json.loads((ASSETS / 'games/override-2026-27.json').read_text())
    goal = next(item for item in game['objects'] if item['name'] == 'goal_red_5')
    goal.update(urdf=str(ASSETS / 'objects/goal_red.urdf'), position=[0, 0, 0])
    config['field_objects'] = [goal, {'name': 'pin', 'kind': 'pin', 'fixed': False,
        'urdf': str(ASSETS / 'objects/pin_red_yellow.urdf'), 'position': [0, 0, .20]}]
    with MuJoCoWorld(config) as world:
        p = world._physics
        bodies = {item['name']: item['id'] for item in world.field_objects}
        pin, goal_id = bodies['pin'], bodies['goal_red_5']
        tree = p.model.body_treeid[p.body_ids[pin, -1]]
        for _ in range(1440):
            world.step()
            if not p.data.tree_awake[tree]:
                break
        assert not p.data.tree_awake[tree]
        for _ in range(30):
            world.step()
        assert world._scoring()['placed_pins'] == 1
        assert p.has_contact(pin, goal_id)
        assert not p.getContactPoints(bodyA=pin, bodyB=goal_id)
        assert p._sleep_contacts
        # Move an actual native joint into the sleeping pin. Contact must wake
        # it and immediately prevent scoring while the robot holds/touches it.
        touched = False
        for _ in range(480):
            world.step({'joints': {'reach': .13}})
            if p.getContactPoints(bodyA=pin, bodyB=world.robot):
                touched = True
                assert p.data.tree_awake[tree]
                assert world._scoring()['placed_pins'] == 0
                assert not p._sleep_contacts.get(tuple(sorted((pin, goal_id))))
                break
        assert touched
        # External force removal must not leave a stale scored pair either.
        for _ in range(300):
            world.step({'joints': {'reach': 0}})
        for _ in range(60):
            position = p.getBasePositionAndOrientation(pin)[0]
            p.applyExternalForce(pin, -1, [3, 0, 6], position, bullet.WORLD_FRAME)
            world.step({'joints': {'reach': 0}})
        assert p.getBasePositionAndOrientation(pin)[0][2] > .3
        assert not p.has_contact(pin, goal_id)
        assert world._scoring()['placed_pins'] == 0
        world.reset()
        assert not world._physics._sleep_contacts


def test_rolling_prop_is_not_put_to_sleep_while_moving(tmp_path):
    config, _ = flywheel_config(tmp_path)
    config.update(gravity=9.80665, field_objects=[
        {'name': 'rolling', 'shape': 'sphere', 'radius': .05, 'mass': .1,
         'position': [.2, 0, .051], 'friction': .3, 'rolling_friction': 0}])
    with MuJoCoWorld(config) as world:
        p = world._physics
        body = next(item['id'] for item in world.field_objects if item['name'] == 'rolling')
        native = p.body_ids[body, -1]
        tree = p.model.body_treeid[native]
        for _ in range(120):
            world.step()
        initial = p.getBasePositionAndOrientation(body)[0]
        p.applyExternalForce(body, -1, [5, 0, 0], initial, bullet.WORLD_FRAME)
        moving_ticks = 0
        for _ in range(240):
            world.step()
            linear, angular = p.getBaseVelocity(body)
            if np.linalg.norm(linear) > .02 or np.linalg.norm(angular) > .4:
                moving_ticks += 1
                assert p.data.tree_awake[tree]
        assert moving_ticks > 100
        assert p.getBasePositionAndOrientation(body)[0][0] > initial[0] + .05
