"""COM reuse and force semantics without a simulation, native worker or GUI."""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from eersim import physx_world


def fixture():
    p = physx_world.bullet
    actors = [
        {'body_id': 7, 'link_index': -1, 'path': '/robot/base',
         'position': [1.2, -2.3, .4], 'orientation': p.getQuaternionFromEuler([.2, -.1, .4]),
         'local_com': [.1, -.03, .04], 'principal_axes': p.getQuaternionFromEuler([.7, -.15, .25])},
        {'body_id': 7, 'link_index': 2, 'path': '/robot/wheel',
         'position': [-.6, .9, .73], 'orientation': p.getQuaternionFromEuler([-.35, .2, .65]),
         'local_com': [-.12, .2, -.05], 'principal_axes': p.getQuaternionFromEuler([-.4, .5, -.8])},
        {'body_id': 9, 'link_index': -1, 'path': '/floor',
         'position': [0, 0, -.02], 'orientation': [0, 0, 0, 1],
         'local_com': [0, 0, 0], 'principal_axes': [0, 0, 0, 1]},
    ]
    state = {'poses': np.asarray([a['position'] + list(a['orientation']) for a in actors[:2]], np.float32),
             'velocities': np.zeros((2, 6), np.float32), 'q': np.zeros((1, 0), np.float32),
             'qd': np.zeros((1, 0), np.float32), 'contacts': []}
    process = SimpleNamespace(initial={'actor_paths': [a['path'] for a in actors[:2]],
                                       'dof_names': [], 'state': state})
    manifest = {'actors': actors, 'joints': []}
    adapter = physx_world.PhysXAdapter(SimpleNamespace(client=0, robot=7, dt=1 / 120), process, manifest)
    return adapter, process, manifest


def uncached_com(adapter, key):
    actor = adapter.actors[key]
    if key in adapter.keys:
        row = adapter._state['poses'][adapter.keys[key]]
        point, quaternion = row[:3], row[3:]
    else:
        point, quaternion = np.asarray(actor['position']), np.asarray(actor['orientation'])
    return physx_world.bullet.multiplyTransforms(point, quaternion, actor['local_com'], actor['principal_axes'])


def original_force(adapter, output, key, force, point, flags):
    """The pre-cache operation order, including float32 wrench accumulation."""
    force, point = np.asarray(force), np.asarray(point)
    if flags == physx_world.bullet.LINK_FRAME:
        position, quaternion = uncached_com(adapter, key)
        force = physx_world.rotation(quaternion) @ force
        point = np.asarray(position) + physx_world.rotation(quaternion) @ point
    centre = np.asarray(uncached_com(adapter, key)[0])
    wrench = output[adapter.keys[key]]
    wrench[:3] += force
    wrench[3:6] += np.cross(point - centre, force)
    wrench[6:] = centre


def original_torque(adapter, output, key, torque, flags):
    torque = np.asarray(torque)
    if flags == physx_world.bullet.LINK_FRAME:
        torque = physx_world.rotation(uncached_com(adapter, key)[1]) @ torque
    wrench = output[adapter.keys[key]]
    wrench[3:6] += torque
    wrench[6:] = uncached_com(adapter, key)[0]


@pytest.mark.parametrize('dtype', [np.float32, np.float64, np.int64])
def test_mixed_contact_forces_and_torques_match_original_bytes_after_every_update(dtype):
    adapter, _, _ = fixture()
    expected = np.zeros_like(adapter.controls['wrenches'])
    rng = np.random.default_rng(1701)
    for index in range(180):
        key = (7, -1 if index % 3 else 2)
        flags = physx_world.bullet.LINK_FRAME if index % 2 else physx_world.bullet.WORLD_FRAME
        force = (rng.normal(size=3) * 17).astype(dtype)
        point = (rng.normal(size=3) * 2).astype(dtype)
        original_force(adapter, expected, key, force, point, flags)
        adapter.applyExternalForce(*key, force, point, flags)
        assert adapter.controls['wrenches'].tobytes() == expected.tobytes()
        if index % 4 == 0:
            torque = (rng.normal(size=3) * 3).astype(dtype)
            original_torque(adapter, expected, key, torque, flags)
            adapter.applyExternalTorque(*key, torque, flags)
            assert adapter.controls['wrenches'].tobytes() == expected.tobytes()


def test_link_frame_uses_rotated_com_and_principal_axes_not_link_origin():
    adapter, _, _ = fixture()
    adapter._state['poses'][0] = [2, 3, 4, *physx_world.bullet.getQuaternionFromEuler([0, 0, np.pi / 2])]
    actor = adapter.actors[(7, -1)]
    actor['local_com'] = [1, 0, 0]
    actor['principal_axes'] = physx_world.bullet.getQuaternionFromEuler([np.pi / 2, 0, 0])
    adapter.applyExternalForce(7, -1, [0, 1., 0], [1., 0, 0], physx_world.bullet.LINK_FRAME)
    adapter.applyExternalTorque(7, -1, [2., 0, 0], physx_world.bullet.LINK_FRAME)
    np.testing.assert_allclose(adapter.controls['wrenches'][0], [0, 0, 1, 1, 2, 0, 2, 4, 4], atol=8e-7)


def test_equal_opposite_world_forces_keep_contact_reaction_and_accumulation_order():
    adapter, _, _ = fixture()
    expected = np.zeros_like(adapter.controls['wrenches'])
    for scale in (1e6, 1e-7, -1e6, .31):
        force, point = np.array([scale, -.23 * scale, .75 * scale]), [.63, -.12, .9]
        for key, reaction in (((7, -1), force), ((7, 2), -force)):
            original_force(adapter, expected, key, reaction, point, physx_world.bullet.WORLD_FRAME)
            adapter.applyExternalForce(*key, reaction, point, physx_world.bullet.WORLD_FRAME)
    assert adapter.controls['wrenches'].tobytes() == expected.tobytes()
    np.testing.assert_array_equal(adapter.controls['wrenches'][:, :3].sum(axis=0), [0, 0, 0])


def test_com_and_rotation_computed_once_per_actor_in_completed_state(monkeypatch):
    adapter, _, _ = fixture()
    counts = {'transforms': 0, 'rotations': 0}
    multiply, rotation = physx_world.bullet.multiplyTransforms, physx_world.rotation
    def counted_transform(*args):
        counts['transforms'] += 1
        return multiply(*args)
    def counted_rotation(quaternion):
        counts['rotations'] += 1
        return rotation(quaternion)
    monkeypatch.setattr(physx_world.bullet, 'multiplyTransforms', counted_transform)
    monkeypatch.setattr(physx_world, 'rotation', counted_rotation)
    for _ in range(15):
        adapter.getBasePositionAndOrientation(7)
        adapter.applyExternalForce(7, -1, [1, 2, 3], [.1, .2, .3], physx_world.bullet.LINK_FRAME)
        adapter.applyExternalTorque(7, -1, [1, 0, 0], physx_world.bullet.LINK_FRAME)
        adapter.getLinkState(7, 2)
        adapter.applyExternalForce(7, 2, [1, 2, 3], [.1, .2, .3], physx_world.bullet.WORLD_FRAME)
    assert counts == {'transforms': 2, 'rotations': 1}


@pytest.mark.parametrize('same_state_object', [False, True])
def test_successful_step_invalidates_all_com_values_even_if_worker_reuses_state(same_state_object):
    adapter, process, _ = fixture()
    adapter.applyExternalForce(7, -1, [1, 2, 3], [.1, .2, .3], physx_world.bullet.LINK_FRAME)
    before = adapter.getBasePositionAndOrientation(7)
    submitted = []
    def step(operation, controls):
        assert operation == 'step'
        submitted.append(controls['wrenches'].copy())
        state = adapter._state if same_state_object else deepcopy(adapter._state)
        state['poses'][0, :3] += [.2, -.7, .13]
        state['poses'][0, 3:] = physx_world.bullet.getQuaternionFromEuler([-.5, .6, -.2])
        return state
    process.call = step
    adapter.stepSimulation()
    assert submitted and np.any(submitted[0])
    assert not np.any(adapter.controls['wrenches'])
    assert adapter.getBasePositionAndOrientation(7) != before
    expected = np.zeros_like(adapter.controls['wrenches'])
    original_force(adapter, expected, (7, -1), [1, 2, 3], [.1, .2, .3], physx_world.bullet.LINK_FRAME)
    adapter.applyExternalForce(7, -1, [1, 2, 3], [.1, .2, .3], physx_world.bullet.LINK_FRAME)
    assert adapter.controls['wrenches'].tobytes() == expected.tobytes()


def test_direct_snapshot_refresh_invalidates_pose_and_rotation_cache():
    adapter, _, _ = fixture()
    adapter.applyExternalTorque(7, 2, [1, 2, 3], physx_world.bullet.LINK_FRAME)
    adapter._state = deepcopy(adapter._state)
    adapter._state['poses'][1] = [.3, -.1, 1., *physx_world.bullet.getQuaternionFromEuler([.8, -.4, .6])]
    expected = adapter.controls['wrenches'].copy()
    original_torque(adapter, expected, (7, 2), [.2, .5, -.9], physx_world.bullet.LINK_FRAME)
    adapter.applyExternalTorque(7, 2, [.2, .5, -.9], physx_world.bullet.LINK_FRAME)
    assert adapter.controls['wrenches'].tobytes() == expected.tobytes()


def test_reset_load_adapter_cannot_reuse_previous_body_ids_or_inertial_metadata():
    old, process, manifest = fixture()
    previous = old.getBasePositionAndOrientation(7)
    changed = deepcopy(manifest)
    changed['actors'][0]['local_com'] = [.9, .8, .7]
    new = physx_world.PhysXAdapter(old.world, process, changed)
    assert new.getBasePositionAndOrientation(7) == uncached_com(new, (7, -1))
    assert new.getBasePositionAndOrientation(7) != previous


def test_fixed_and_unknown_actor_forces_remain_noops():
    adapter, _, _ = fixture()
    for body in (9, 123456):
        adapter.applyExternalForce(body, -1, [1, 2, 3], [1, 2, 3], physx_world.bullet.LINK_FRAME)
        adapter.applyExternalTorque(body, -1, [1, 2, 3], physx_world.bullet.LINK_FRAME)
    assert not adapter._com_frames
    assert not np.any(adapter.controls['wrenches'])
