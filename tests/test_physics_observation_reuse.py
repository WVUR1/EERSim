"""Observation/control reuse must preserve contacts and live world edits."""
from collections import Counter
import math
from types import SimpleNamespace

import pytest

pytest.importorskip("pybullet")

from eersim.performance import PerformanceRecorder
from eersim.physics import World


class ObservationPhysics:
    WORLD_FRAME = 2

    def __init__(self):
        self.poses = {}
        self.pose_reads = Counter()
        self.masses = {2: 0.0}
        self.mass_reads = Counter()
        self.contacts = []
        self.forces = []
        self.rays = []

    def getBasePositionAndOrientation(self, body, **kwargs):
        self.pose_reads[body] += 1
        return self.poses[body], (0.0, 0.0, 0.0, 1.0)

    def getMatrixFromQuaternion(self, quaternion):
        return (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

    def getBaseVelocity(self, body, **kwargs):
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    def getLinkState(self, body, link, **kwargs):
        return ((0.0, 0.0, 0.0), None, None, None, None, None,
                (1.0, 0.25, 0.0), (0.0, 0.0, 0.0))

    def getContactPoints(self, **kwargs):
        return self.contacts

    def getDynamicsInfo(self, body, link, **kwargs):
        self.mass_reads[body, link] += 1
        return (self.masses[body],)

    def applyExternalForce(self, body, link, force, point, frame, **kwargs):
        self.forces.append((body, link, tuple(force), tuple(point), frame))

    def rayTest(self, start, end, **kwargs):
        self.rays.append((start, end))
        return [(2, -1, 0.25)]


def observation_world():
    world = World.__new__(World)
    world._physics = ObservationPhysics()
    world.performance = PerformanceRecorder(enabled=False)
    world.robot = 1
    world.client = 0
    world.config = SimpleNamespace(friction=0.6)
    world.dt = 1 / 120
    world.robot_settings = {}
    return world


def goal(body):
    return {"id": body, "name": f"goal_{body}", "scoring": {
        "type": "goal", "height": 0.1, "socket_depth": 0.055,
        "socket_radius": 0.027, "alliance": "red",
    }}


def pin(body):
    return {"id": body, "name": f"pin_{body}", "scoring": {
        "type": "pin", "length": 0.165, "radius": 0.020,
    }}


def test_scoring_reuses_goal_observation_but_refreshes_movement_and_live_metadata():
    world = observation_world()
    first, second = goal(10), goal(11)
    world.field_objects = [first, second, pin(3), pin(4), pin(5)]
    physics = world._physics
    physics.poses = {10: (0.0, 0.0, 0.0), 11: (0.2, 0.0, 0.0),
                     3: (0.0, 0.0, 0.1575), 4: (0.008, 0.0, 0.1575),
                     5: (0.2, 0.0, 0.1575)}
    touching = {(3, 10), (4, 10), (5, 11)}
    world._has_contact = lambda a, b: (a, b) in touching

    assert world._scoring()["placements"] == [
        {"pin": "pin_3", "goal": "goal_10", "alliance": "red"},
        {"pin": "pin_4", "goal": "goal_10", "alliance": "red"},
        {"pin": "pin_5", "goal": "goal_11", "alliance": "red"},
    ]
    assert physics.pose_reads[10] == physics.pose_reads[11] == 1

    # A goal can move and scoring parameters can change between observations.
    physics.poses[10] = (0.008, 0.0, 0.0)
    first["scoring"]["socket_radius"] = 0.0175
    first["scoring"]["alliance"] = "blue"
    second["scoring"]["height"] = 0.3
    assert world._scoring()["placements"] == [
        {"pin": "pin_4", "goal": "goal_10", "alliance": "blue"},
    ]
    assert physics.pose_reads[10] == physics.pose_reads[11] == 2

    # Reload-like replacement and ID reuse must not leave stale goal metadata.
    replacement = goal(10)
    replacement["scoring"]["type"] = "loader"
    world.field_objects = [replacement, pin(4)]
    assert world._scoring()["placements"] == []


def test_scoring_still_requires_release_and_socket_contact():
    world = observation_world()
    world.field_objects = [goal(10), pin(3), pin(4), pin(5)]
    world._physics.poses = {10: (0.0, 0.0, 0.0),
                           3: (0.0, 0.0, 0.1575), 4: (0.0, 0.0, 0.1575),
                           5: (0.0, 0.0, 0.3)}
    touching = {(3, 10), (3, 1), (5, 10)}
    world._has_contact = lambda a, b: (a, b) in touching
    assert world._scoring()["placed_pins"] == 0
    assert world._physics.pose_reads[10] == 1


def test_goal_bounds_remain_lazy_when_no_pin_reaches_them():
    world = observation_world()
    malformed_goal = goal(10)
    malformed_goal["scoring"]["socket_depth"] = "invalid"
    world.field_objects = [malformed_goal, pin(3)]
    world._physics.poses = {10: (0.0, 0.0, 0.0), 3: (1.0, 0.0, 0.1575)}
    world._has_contact = lambda a, b: False
    assert world._scoring()["placed_pins"] == 0
    world._physics.poses[3] = (0.0, 0.0, 0.1575)
    with pytest.raises(ValueError):
        world._scoring()


def tire_contact(point):
    return (0, 1, 2, 0, -1, point, point, (0.0, 0.0, 1.0), 0.0, 5.0,
            0.0, (0.0, 0.0, 0.0), 0.0, (0.0, 0.0, 0.0))


def test_tire_settings_and_contact_mass_refresh_between_ticks():
    world = observation_world()
    world.robot_settings = {"drive": {"type": "omni"}}
    wheel = {"index": 0, "direction": [1.0, 0.0, 0.0], "inertia": 1.0,
             "radius": 0.1, "roller_friction": 0.02}
    world._wheels = [wheel]
    physics = world._physics
    physics.poses = {1: (0.0, 0.0, 0.0), 2: (0.0, 0.0, 0.0)}
    physics.contacts = [tire_contact((0.2, 0.0, 0.0)), tire_contact((0.3, 0.0, 0.0))]
    world._apply_omni_tires()
    expected = (-0.6 * 5 * math.tanh(1.0 / 0.15),
                -0.02 * 5 * math.tanh(0.25 / 0.1), 0.0)
    assert [entry[2] for entry in physics.forces] == [expected, expected]
    assert physics.mass_reads[2, -1] == 1

    physics.forces.clear()
    physics.masses[2] = 2.0
    world.config.friction = 0.3
    wheel["roller_friction"] = 0.04
    world._apply_omni_tires()
    expected = (-0.3 * 5 * math.tanh(1.0 / 0.15),
                -0.04 * 5 * math.tanh(0.25 / 0.1), 0.0)
    assert [entry[0] for entry in physics.forces] == [1, 2, 1, 2]
    assert [entry[2] for entry in physics.forces] == [
        expected, tuple(-value for value in expected),
        expected, tuple(-value for value in expected),
    ]
    assert physics.mass_reads[2, -1] == 2


def test_airborne_wheel_keeps_lazy_setting_reads():
    world = observation_world()
    world.robot_settings = {"drive": {"type": "omni"}}
    world._wheels = [{"index": 0, "direction": [1.0, 0.0, 0.0], "friction": "invalid"}]
    world._physics.poses[1] = (0.0, 0.0, 0.0)
    world._apply_omni_tires()
    assert world._physics.forces == []


def test_distance_sensor_edits_and_empty_configuration_are_live():
    world = observation_world()
    assert world._distances() == {}
    assert world._physics.pose_reads == {}
    world._physics.poses[1] = (1.0, 2.0, 3.0)
    sensor = {"port": 9, "position": [0.1, 0.0, 0.0], "direction": [2.0, 0.0, 0.0],
              "max_range": 2.0}
    world.robot_settings["distance_sensors"] = [sensor]
    assert world._distances() == {"9": 500.0}
    assert world._physics.rays[-1] == ([1.1, 2.0, 3.0], [3.1, 2.0, 3.0])
    sensor["direction"][:] = [0.0, 3.0, 0.0]
    sensor["max_range"] = 1.0
    assert world._distances() == {"9": 250.0}
    assert world._physics.rays[-1] == ([1.1, 2.0, 3.0], [1.1, 3.0, 3.0])
    sensor["direction"][:] = [0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="direction cannot be zero"):
        world._distances()
