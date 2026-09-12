"""Contact query compatibility and bounded candidate traversal; no simulation."""
from types import SimpleNamespace

import numpy as np
import pytest

from eersim.physx_world import PhysXAdapter


def point(a, b, left=-1, right=-1, value=1.):
    return (0, a, b, left, right, (1., 2., 3.), (4., 5., 6.),
            (0., 0., 1.), -.001, value, 0., (0., 0., 0.), 0., (0., 0., 0.))


def reverse(contact):
    return (contact[0], contact[2], contact[1], contact[4], contact[3],
            contact[6], contact[5], tuple(-x for x in contact[7]), *contact[8:])


def adapter(contacts):
    value = PhysXAdapter.__new__(PhysXAdapter)
    value._state = {'contacts': contacts}
    value._invalidate_contact_index()
    return value


@pytest.fixture
def scene():
    floor = point(1, 9, right=0)
    piece = point(4, 9, right=0, value=2.)
    same_robot = point(9, 9, left=1, right=2, value=3.)
    another_wheel = point(1, 9, right=3, value=4.)
    # Distinct compound leaves may produce exactly coincident manifold points.
    return [floor, piece, piece, same_robot, another_wheel]


def test_body_link_query_reverses_witnesses_and_preserves_manifold_multiplicity(scene):
    world = adapter(scene)
    assert world.getContactPoints(bodyA=9, linkIndexA=0) == [reverse(scene[0]), reverse(scene[1]), reverse(scene[2])]
    assert world.getContactPoints(bodyB=9, linkIndexB=0) == scene[:3]
    assert world.getContactPoints(bodyA=4, bodyB=9) == scene[1:3]
    assert world.getContactPoints(bodyA=9, bodyB=4) == [reverse(scene[1]), reverse(scene[2])]


def test_same_body_contacts_choose_orientation_by_link_without_duplicates(scene):
    world = adapter(scene)
    assert world.getContactPoints(9, 9) == [scene[3]]
    assert world.getContactPoints(9, 9, 2, 1) == [reverse(scene[3])]
    assert world.getContactPoints(9, 9, 1, 2) == [scene[3]]
    assert world.getContactPoints(9, 9, 1, 1) == []


def test_wildcards_and_empty_queries_preserve_original_order(scene):
    world = adapter(scene)
    result = world.getContactPoints()
    assert result == scene
    result.clear()
    assert world.getContactPoints() == scene
    assert world.getContactPoints(linkIndexA=0) == [reverse(item) for item in scene[:3]]
    assert world.getContactPoints(linkIndexB=0) == scene[:3]
    assert world.getContactPoints(bodyA=-3, bodyB=1) == [reverse(scene[0]), reverse(scene[4])]
    assert world.getContactPoints(bodyA=9, linkIndexA=-3) == []
    assert world.getContactPoints(bodyA=12345) == []
    assert world.getContactPoints(bodyA=1, bodyB=4) == []


def test_new_contact_snapshot_cannot_reuse_old_reversed_force_or_indices():
    first, second = point(1, 9, right=0), point(3, 9, right=0, value=17.)
    world = adapter([first])
    assert world.getContactPoints(9) == [reverse(first)]
    world._state = {'contacts': [second]}
    assert world.getContactPoints(9) == [reverse(second)]
    assert world.getContactPoints(1) == []


def test_step_invalidates_indexes_when_worker_reuses_contact_list():
    contacts = [point(1, 9, right=0)]
    world = adapter(contacts)
    world.joint_records, world.dofs = {}, {}
    world.controls = {'wrenches': np.zeros((1, 9))}
    assert world.getContactPoints(9) == [reverse(contacts[0])]

    def step(command, controls):
        assert command == 'step'
        contacts[:] = [point(3, 9, right=0, value=23.)]
        return {'contacts': contacts}

    world.process = SimpleNamespace(call=step)
    world.stepSimulation()
    assert world.getContactPoints(1) == []
    assert world.getContactPoints(9) == [reverse(contacts[0])]


def test_repeated_wheel_query_visits_only_matching_candidates():
    class ObservedContacts(list):
        traversed = 0

        def __iter__(self):
            for contact in super().__iter__():
                self.traversed += 1
                yield contact

        def __getitem__(self, index):
            self.traversed += 1
            return super().__getitem__(index)

    contacts = ObservedContacts([point(1, body) for body in range(100, 20100)])
    target = point(1, 9, right=3)
    contacts.append(target)
    world = adapter(contacts)
    assert world.getContactPoints(9, linkIndexA=3) == [reverse(target)]
    contacts.traversed = 0
    for _ in range(4):
        assert world.getContactPoints(9, linkIndexA=3) == [reverse(target)]
    assert contacts.traversed == 4
