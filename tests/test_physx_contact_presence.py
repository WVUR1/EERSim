"""Preserve sleeping contact presence without retaining forces or stale geometry."""
import numpy as np
import pytest

from eersim.physx_contacts import ContactReader
from test_physx_contacts import Binding, Physics, actor


def fixture():
    actors = [actor('/World/robot', 1), actor('/World/pin', 2),
              actor('/World/pin_link', 2, link=0), actor('/World/goal', 3, mass=0),
              actor('/World/goal_link', 3, link=0), actor('/World/other', 4)]
    # Include a dynamic goal link so all-links checking applies to both bodies.
    paths = [item['path'] for item in actors if item['mass_kg'] > 0]
    binding = Binding(paths, paths={10: '/World/goal/collider', 11: '/World/robot', 12: '/World/pin'})
    binding.reports['/World/pin'] = [(10, (0, 0, 0), (0, 0, 1), -.001, 2)]
    manifest = {'actors': actors}
    reader = ContactReader(Physics(binding), manifest, capacity=32)
    poses = {path: np.array([0, 0, 0, 0, 0, 0, 1.], dtype=np.float64) for path in paths}
    return reader, binding, poses, manifest


def test_unchanged_nonrobot_pair_survives_missing_reports_without_fabricating_forces():
    reader, binding, poses, _ = fixture()
    first = reader.capture_runtime(1, poses)
    assert first == {'contacts': [], 'contact_body_pairs': [(2, 3)]}
    assert reader.retained_pair_count == 0
    binding.reports.clear()
    # Different body movement must not invalidate a stationary pin-goal pair.
    poses['/World/other'][0] += .5
    for _ in range(3):
        assert reader.capture_runtime(1, poses) == first
        assert reader.retained_pair_count == 1
        assert reader.capture() == []  # no stale full manifold or force


@pytest.mark.parametrize('path,component', [('/World/pin', 0), ('/World/pin', 6),
    ('/World/pin_link', 1), ('/World/pin_link', 4), ('/World/goal_link', 2), ('/World/goal_link', 3)])
def test_any_link_translation_or_orientation_change_drops_old_presence(path, component):
    reader, binding, poses, _ = fixture()
    reader.capture_runtime(1, poses)
    binding.reports.clear()
    # Mutation of the same input arrays also verifies that history was copied.
    poses[path][component] += 1e-12
    assert reader.capture_runtime(1, poses)['contact_body_pairs'] == []
    assert reader.retained_pair_count == 0
    poses[path][component] -= 1e-12
    assert reader.capture_runtime(1, poses)['contact_body_pairs'] == []


def test_fresh_report_replaces_changed_pose_history_and_is_not_counted_as_retained():
    reader, binding, poses, _ = fixture()
    reader.capture_runtime(1, poses)
    poses['/World/pin'][0] += .2
    assert reader.capture_runtime(1, poses)['contact_body_pairs'] == [(2, 3)]
    assert reader.retained_pair_count == 0
    binding.reports.clear()
    assert reader.capture_runtime(1, poses)['contact_body_pairs'] == [(2, 3)]
    assert reader.retained_pair_count == 1


def test_robot_presence_and_forces_are_never_cached():
    reader, binding, poses, _ = fixture()
    binding.reports['/World/robot'] = [(12, (0, 0, 0), (1, 0, 0), -.001, 3)]
    binding.reports['/World/pin'].append((11, (0, 0, 0), (1, 0, 0), -.001, -3))
    active = reader.capture_runtime(1, poses)
    assert active['contact_body_pairs'] == [(1, 2), (2, 3)]
    assert len(active['contacts']) == 1 and active['contacts'][0][9] == 3
    binding.reports.clear()
    sleeping = reader.capture_runtime(1, poses)
    assert sleeping == {'contacts': [], 'contact_body_pairs': [(2, 3)]}


def test_omitting_poses_or_recreating_reader_clears_contact_history():
    reader, binding, poses, manifest = fixture()
    reader.capture_runtime(1, poses)
    binding.reports.clear()
    assert reader.capture_runtime(1)['contact_body_pairs'] == []
    assert reader.capture_runtime(1, poses)['contact_body_pairs'] == []
    reader.close()
    replacement = ContactReader(Physics(binding), manifest, capacity=32)
    assert replacement.capture_runtime(1, poses)['contact_body_pairs'] == []


@pytest.mark.parametrize('invalid', ['missing_actor', 'unknown_actor', 'wrong_shape', 'nonfinite'])
def test_invalid_complete_pose_observation_breaks_cache_continuity(invalid):
    reader, binding, poses, _ = fixture()
    reader.capture_runtime(1, poses)
    binding.reports.clear()
    broken = {key: value.copy() for key, value in poses.items()}
    if invalid == 'missing_actor':
        del broken['/World/other']
    elif invalid == 'unknown_actor':
        broken['/World/unknown'] = np.zeros(7)
    elif invalid == 'wrong_shape':
        broken['/World/pin'] = np.zeros(6)
    else:
        broken['/World/pin'][0] = np.nan
    with pytest.raises(ValueError):
        reader.capture_runtime(1, broken)
    assert reader.capture_runtime(1, poses)['contact_body_pairs'] == []
