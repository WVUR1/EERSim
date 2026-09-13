"""Sparse/mixed contact regressions; no SDK, simulation, or graphics is created."""
import math
import struct

import numpy as np
import pytest

from eersim.physx_contacts import ContactReader
from test_physx_contacts import Binding, Physics, actor


def sparse_fixture():
    quiet = [actor(f'/World/quiet_{i}', i + 200) for i in range(80)]
    actors = quiet + [actor('/World/right', 90, link=7), actor('/World/left', 90, link=2),
                      actor('/World/pin', 4), actor('/World/floor', 0, mass=0),
                      actor('/World/goal', 2, mass=0)]
    sensors = [item['path'] for item in quiet]
    sensors.insert(6, '/World/right')
    sensors.insert(51, '/World/pin')
    sensors.append('/World/left')
    point = (1., 2., 3.)
    reports = {
        '/World/right': [(10, (3, 2, 1), (0., -0., 1.), -.001, 4.)],
        '/World/pin': [(11, (4, 5, 6), (0, 0, 1), -.002, 5.)] * 50
                       + [(12, point, (1., -0., 0.), -.003, -0.)] * 2
                       + [(11, (7, 8, 9), (0, 0, 1), -.002, 5.)] * 50,
        '/World/left': [(13, point, (1., -0., 0.), -.003, 0.)] * 2,
    }
    binding = Binding(sensors, reports, {10: '/World/floor/collider', 11: '/World/goal/collider',
                                        12: '/World/left', 13: '/World/pin'}, capacity=128)
    reader = ContactReader(Physics(binding), {'actors': actors}, capacity=128)
    return reader, binding


def test_sparse_mixed_selection_preserves_pair_order_duplicates_and_signed_zero():
    reader, binding = sparse_fixture()
    result = reader.capture_runtime(90)
    contacts = result['contacts']
    assert result['contact_body_pairs'] == [(0, 90), (2, 4), (4, 90)]
    assert [point[1:5] for point in contacts] == [(0, 90, -1, 7), (4, 90, -1, 2), (4, 90, -1, 2)]
    assert contacts[1] == contacts[2]  # Distinct coincident manifold points survive.
    assert contacts[0][5] == (3., 2., 1.)
    assert contacts[1][5] == (1., 2., 3.)
    assert tuple(math.copysign(1., item) for item in contacts[0][7]) == (-1., 1., -1.)
    assert tuple(math.copysign(1., item) for item in contacts[1][7]) == (-1., 1., -1.)
    assert contacts[1][9] == 0 and math.copysign(1., contacts[1][9]) == 1
    full = reader.capture()
    assert [point for point in full if 90 in point[1:3]] == contacts
    assert len(binding.resolutions) == 1
    reader.close()


@pytest.mark.parametrize('force', [0., -0., 1e-30, -1e-30])
@pytest.mark.parametrize('sensor_body,other_body', [(1, 2), (2, 1)])
def test_scalar_normal_transform_preserves_ieee_zero_signs_and_direction(force, sensor_body, other_body):
    normal = (0., -0., 1.)
    binding = Binding(['/World/sensor'], {'/World/sensor': [(11, normal, normal, -.002, force)]},
                      {11: '/World/static'})
    reader = ContactReader(Physics(binding), {'actors': [actor('/World/sensor', sensor_body),
                          actor('/World/static', other_body, mass=0)]}, capacity=32)
    observed = reader.capture()[0][7]
    expected = np.array(normal, dtype=np.float32).astype(float)
    if np.signbit(force):
        expected = -expected
    if sensor_body > other_body:
        expected = -expected
    assert b''.join(struct.pack('!d', item) for item in observed) == expected.astype('>f8').tobytes()
    reader.close()


def test_selection_changes_when_caller_chooses_another_robot_body():
    reader, binding = sparse_fixture()
    first = reader.capture_runtime(90)
    second = reader.capture_runtime(4)
    assert len(first['contacts']) == 3
    assert len(second['contacts']) == 102
    assert first['contact_body_pairs'] == second['contact_body_pairs']
    assert all(4 in point[1:3] for point in second['contacts'])
    assert reader.capture_runtime(90) == first
    reader.close()


def test_empty_bucket_transition_does_not_reuse_selected_points_or_hide_missing_mirror():
    reader, binding = sparse_fixture()
    assert reader.capture_runtime(90)['contacts']
    binding.reports = {}
    assert reader.capture_runtime(90) == {'contacts': [], 'contact_body_pairs': []}
    binding.reports = {'/World/pin': [(12, (1, 2, 3), (1, 0, 0), -.003, -1.)]}
    with pytest.raises(RuntimeError, match='Mirrored contact reports disagree'):
        reader.capture_runtime(90)
    reader.close()


@pytest.mark.parametrize('defect', ['nonfinite', 'zero_normal', 'self', 'unmapped'])
def test_unselected_nonrobot_bucket_keeps_all_validation(defect):
    reader, binding = sparse_fixture()
    binding.reports = {'/World/pin': [(11, (0, 0, 0), (0, 0, 1), 0, 1.)]}
    if defect == 'nonfinite':
        binding.reports['/World/pin'][0] = (11, (float('nan'), 0, 0), (0, 0, 1), 0, 1.)
    elif defect == 'zero_normal':
        binding.reports['/World/pin'][0] = (11, (0, 0, 0), (0, 0, 0), 0, 1.)
    else:
        binding.paths[11] = '/World/pin' if defect == 'self' else '/World/absent'
    with pytest.raises(RuntimeError, match='nonfinite|not unit|colliding with itself|absent'):
        reader.capture_runtime(90)
    reader.close()


def test_retained_presence_still_requires_exact_pose_continuity_with_empty_buckets():
    reader, binding = sparse_fixture()
    poses = {path: np.array([0, 0, 0, 0, 0, 0, 1.]) for path in binding.sensor_paths}
    reader.capture_runtime(90, poses)
    binding.reports = {}
    assert reader.capture_runtime(90, poses) == {'contacts': [], 'contact_body_pairs': [(2, 4)]}
    assert reader.retained_pair_count == 1
    poses['/World/pin'][1] = np.nextafter(0., 1.)
    assert reader.capture_runtime(90, poses) == {'contacts': [], 'contact_body_pairs': []}
    assert reader.retained_pair_count == 0
    reader.close()
