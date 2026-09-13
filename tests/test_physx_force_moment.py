"""Exact three-component wrench arithmetic, with no simulation or worker."""
import numpy as np
import pytest
from types import SimpleNamespace

from eersim import physx_world


@pytest.mark.parametrize('left_type,right_type', [
    (np.float32, np.float32), (np.float32, np.float64),
    (np.float64, np.float32), (np.float64, np.float64),
])
def test_moment_matches_numpy_bytes_for_contact_scale_and_extreme_inputs(left_type, right_type):
    random = np.random.default_rng(82307)
    samples = [(random.normal(size=3) * size, random.normal(size=3) * force)
               for size in (1e-100, 1e-6, .03, 1., 1e100)
               for force in (1e-100, 1e-6, 1., 100., 1e100)
               for _ in range(16)]
    samples.extend((np.asarray(a), np.asarray(b)) for a, b in [
        ([0., -0., 0.], [-0., 0., -0.]),
        ([-0., 0., -0.], [0., -0., 0.]),
        ([1., 1., 1.], [1., 1., 1.]),
        ([1e308, 1e308, 1e308], [1e308, -1e308, 1e308]),
        ([np.inf, 0., -np.inf], [0., np.inf, 1.]),
    ])
    with np.errstate(all='ignore'):
        for offset, force in samples:
            offset, force = offset.astype(left_type), force.astype(right_type)
            expected = np.cross(offset, force)
            actual = physx_world._vector_cross(offset, force)
            assert actual.dtype == expected.dtype
            assert actual.tobytes() == expected.tobytes()


def test_common_tire_input_avoids_generic_cross_and_returns_independent_result(monkeypatch):
    offset = np.array([.03, -.06, .09])
    force = np.array([1.2, -.3, 5.7])
    expected = np.cross(offset, force)
    before = offset.copy(), force.copy()

    def unexpected(*args, **kwargs):
        raise AssertionError('Generic cross called for three float64 components')

    monkeypatch.setattr(physx_world.np, 'cross', unexpected)
    actual = physx_world._vector_cross(offset, force)
    assert actual.tobytes() == expected.tobytes()
    actual[:] = 0
    np.testing.assert_array_equal(offset, before[0])
    np.testing.assert_array_equal(force, before[1])


@pytest.mark.parametrize('offset,force', [
            (np.ones(3, np.int64), np.ones(3, np.float64)),
    (np.ones(3, np.float64), np.ones(3, np.int64)),
    (np.ones((1, 3)), np.ones(3)),
    (np.ones(3), np.ones((2, 3))),
    (np.ones(3, object), np.ones(3, object)),
])
def test_other_dtypes_and_shapes_keep_numpy_handling(monkeypatch, offset, force):
    calls = []
    generic = np.cross
    expected = generic(offset, force)

    def recorded(a, b):
        calls.append((a, b))
        return generic(a, b)

    monkeypatch.setattr(physx_world.np, 'cross', recorded)
    actual = physx_world._vector_cross(offset, force)
    assert len(calls) == 1 and calls[0][0] is offset and calls[0][1] is force
    assert actual.dtype == expected.dtype
    np.testing.assert_array_equal(actual, expected)


def test_invalid_vector_shape_retains_numpy_exception():
    with pytest.raises(ValueError):
        physx_world._vector_cross(np.ones(4), np.ones(3))


def passive_adapter(kind):
    p = physx_world.bullet
    actors = [
        {'body_id': 7, 'link_index': index, 'path': f'/field/link_{index}',
         'local_com': [.13, -.04, .02],
         'principal_axes': p.getQuaternionFromEuler([.2, -.3, .1])}
        for index in (-1, 2)
    ]
    joint = {'body_id': 7, 'joint_index': 1, 'name': 'passive_joint',
             'type': kind, 'parent_link_index': -1, 'child_link_index': 2,
             'origin_xyz': [.23, -.07, .11], 'origin_rpy': [.3, -.5, .1],
             'axis': [1., 0., 0.]}
    state = {'poses': np.zeros((2, 7), np.float32),
             'velocities': np.zeros((2, 6), np.float32),
             'q': np.zeros((1, 0), np.float32), 'qd': np.zeros((1, 0), np.float32),
             'contacts': []}
    state['poses'][:, 6] = 1.
    process = SimpleNamespace(initial={'actor_paths': [a['path'] for a in actors],
                                       'dof_names': [], 'state': state})
    return physx_world.PhysXAdapter(SimpleNamespace(client=0, robot=0, dt=1 / 120),
                                   process, {'actors': actors, 'joints': [joint]})


@pytest.mark.parametrize('kind', ['continuous', 'revolute', 'prismatic'])
def test_passive_joint_motion_matches_generic_cross_with_translated_rotated_frames(monkeypatch, kind):
    adapter = passive_adapter(kind)
    random = np.random.default_rng(5971)
    specialized = physx_world._vector_cross
    for _ in range(80):
        for row in range(2):
            adapter._state['poses'][row] = [
                *random.normal(size=3),
                *physx_world.bullet.getQuaternionFromEuler(random.normal(size=3)),
            ]
        adapter._state['velocities'][:] = random.normal(size=(2, 6)) * 3
        adapter._invalidate_com_cache()
        monkeypatch.setattr(physx_world, '_vector_cross', np.cross)
        expected = adapter.getJointState(7, 1)
        monkeypatch.setattr(physx_world, '_vector_cross', specialized)
        actual = adapter.getJointState(7, 1)
        assert np.asarray(actual[:2]).tobytes() == np.asarray(expected[:2]).tobytes()
        assert actual[2:] == expected[2:]


def test_prismatic_motion_cancels_common_rigid_rotation():
    adapter = passive_adapter('prismatic')
    for actor in adapter.actors.values():
        actor['local_com'] = [0., 0., 0.]
        actor['principal_axes'] = [0., 0., 0., 1.]
    joint = adapter.joint_records[(7, 1)]
    joint.update(origin_xyz=[0., 0., 0.], origin_rpy=[0., 0., 0.], axis=[1., 0., 0.])
    # Rotating both bodies about the parent with the matching child linear
    # velocity introduces no translation along their prismatic degree of freedom.
    adapter._state['poses'][:, :3] = [[1., 2., 3.], [1.5, 2., 3.]]
    adapter._state['velocities'][:] = [[0., 0., 0., 0., 0., 2.],
                                     [0., 1., 0., 0., 0., 2.]]
    assert adapter.getJointState(7, 1)[:2] == (.5, 0.)
    adapter._state['velocities'][1, 0] = .125
    assert adapter.getJointState(7, 1)[:2] == (.5, .125)


@pytest.mark.parametrize('kind', ['continuous', 'revolute'])
@pytest.mark.parametrize('static_link', [None, -1, 2])
def test_hinge_velocity_uses_angular_motion_without_reading_unused_com(monkeypatch, kind, static_link):
    adapter = passive_adapter(kind)
    if static_link is not None:
        row = adapter.keys.pop((7, static_link))
        actor = adapter.actors[(7, static_link)]
        actor.update(position=adapter._state['poses'][row, :3].tolist(),
                     orientation=adapter._state['poses'][row, 3:].tolist())
    adapter._state['velocities'][:] = [[.7, -.3, .8, -.4, .5, -.6],
                                     [.8, -.2, .9, -.3, .6, -.5]]
    p0, q0 = adapter._actor_pose(7, -1)
    joint = adapter.joint_records[(7, 1)]
    _, frame_quat = physx_world.bullet.multiplyTransforms(
        p0, q0, joint['origin_xyz'], physx_world.bullet.getQuaternionFromEuler(joint['origin_rpy']))
    axis = physx_world.rotation(frame_quat) @ np.asarray(joint['axis'])
    angular0 = np.zeros(6)[3:] if static_link == -1 else adapter._state['velocities'][0, 3:]
    angular1 = np.zeros(6)[3:] if static_link == 2 else adapter._state['velocities'][1, 3:]
    expected = float(np.dot(axis, angular1-angular0))

    def unused(*args, **kwargs):
        raise AssertionError('Hinge speed does not need anchor COM or linear velocity')

    monkeypatch.setattr(adapter, '_com_pose', unused)
    monkeypatch.setattr(physx_world, '_vector_cross', unused)
    assert adapter.getJointState(7, 1)[1] == expected
    # Arbitrary changes to linear velocities cannot change a hinge's speed.
    adapter._state['velocities'][:, :3] += [[1e5, -1e5, 1e5], [-1e5, 1e5, -1e5]]
    assert adapter.getJointState(7, 1)[1] == expected
