"""Behavior tests: contacts, signed shafts, fixed ticks and sensor timing."""
from copy import deepcopy
import json
import math

import numpy as np
import pytest

pytest.importorskip("pybullet")

from eersim.config import ASSETS, default_config
from eersim.physics import World


def drive_commands(world, forward=0, right=0, clockwise=0):
    demand = {
        "front_left": forward + right + clockwise,
        "front_right": forward - right - clockwise,
        "back_left": forward - right + clockwise,
        "back_right": forward + right - clockwise,
    }
    return {"motors": {
        str(motor["port"]): {"mode": "voltage", "value": demand.get(motor["joint"], 0) * motor["sign"]}
        for motor in world.motor_bindings
    }}


def clean_config():
    config = default_config()
    config["game_pack"] = str(ASSETS / "games" / "practice.json")
    config["start_pose"] = [-1.2, -1.2, .085, 0]
    return config


def settle(world, steps=120):
    for _ in range(steps):
        world.step()


def test_signed_group_drives_forward_through_wheel_contact():
    with World(clean_config()) as world:
        settle(world)
        start = world.snapshot()["pose"]
        for _ in range(180):
            end = world.step(drive_commands(world, forward=5000))
        assert end["pose"][0] > start[0] + .10
        assert abs(end["pose"][1] - start[1]) < .10
        assert abs(end["pose"][3]) < .2
        assert end["motors"]["15"]["position"] > 0
        assert end["motors"]["16"]["position"] < 0
        assert end["motors"]["15"]["position"] == pytest.approx(-end["motors"]["16"]["position"])


def test_xdrive_strafes_and_turns_with_contacts():
    with World(clean_config()) as world:
        settle(world)
        start = world.snapshot()["pose"]
        for _ in range(120):
            end = world.step(drive_commands(world, right=3500))
        assert end["pose"][1] < start[1] - .05
        world.reset()
        settle(world)
        for _ in range(120):
            end = world.step(drive_commands(world, clockwise=2500))
        assert end["pose"][3] < -.1


def test_airborne_drive_cannot_translate_chassis():
    config = clean_config()
    config["gravity"] = 0
    config["start_pose"][2] = 1
    with World(config) as world:
        start = world.snapshot()["pose"]
        for _ in range(60):
            end = world.step(drive_commands(world, forward=8000))
        # Wheels may exchange angular momentum with the chassis, but no invented
        # ground traction may translate this free-floating robot horizontally.
        assert abs(end["pose"][0] - start[0]) < .015
        assert abs(end["pose"][1] - start[1]) < .015


def test_wall_collision_blocks_powered_drive():
    config = clean_config()
    config["start_pose"] = [1.40, -.9, .085, 0]
    with World(config) as world:
        settle(world)
        for _ in range(480):
            end = world.step(drive_commands(world, forward=10000))
        assert end["pose"][0] < 1.68
        assert end["pose"][2] < .25


def test_sensor_noise_deterministic_and_independent_of_read_or_render_rate():
    config = clean_config()
    config["noise"].update(position_std_m=.003, heading_std_deg=.3, encoder_std_deg=.2,
                           distance_std_mm=3, latency_ms=25, dropout=.2)
    with World(config) as a, World(deepcopy(config)) as b:
        for i in range(60):
            sa = a.step(drive_commands(a, forward=2500))
            for _ in range(i % 4):
                assert a.snapshot() == sa
            if i % 20 == 0:
                assert a.render(32, 32).shape == (32, 32, 4)
            sb = b.step(drive_commands(b, forward=2500))
            assert sa == sb
        assert sa["time"] == .25
        assert sa["sensor_age_ms"] >= 25 - 1e-8


def test_dropout_holds_sensors_while_ground_truth_advances():
    config = clean_config()
    config["noise"]["dropout"] = 1
    with World(config) as world:
        initial = world.snapshot()
        settle(world, 50)
        final = world.snapshot()
        assert final["otos"] == initial["otos"]
        assert final["motors"] == initial["motors"]
        assert final["time"] > 0
        assert final["pose"][2] < initial["pose"][2]


def test_pneumatic_prismatic_joint_and_disable():
    with World(clean_config()) as world:
        settle(world)
        for _ in range(240):
            world.step({"adi": {"B": True}})
        lift = next(joint for joint in world.joints if joint["name"] == "lift")
        assert .08 < lift["position"] < .125
        time = world.time
        world.step(drive_commands(world, forward=7000))
        world.disable()
        assert world.time == time + world.dt
        final = world.step()
        assert all(motor["torque"] == 0 for motor in final["motors"].values())


def test_import_tank_and_current_game_pack():
    config = clean_config()
    config.update(robot_urdf=str(ASSETS / "robots" / "tank.urdf"),
                  robot_config=str(ASSETS / "robots" / "tank.json"),
                  game_pack=str(ASSETS / "games" / "override-2026-27.json"),
                  start_pose=[-1.5, -.9, .085, 0])
    with World(config) as world:
        settle(world)
        initial = world.snapshot()["pose"][0]
        for _ in range(120):
            snapshot = world.step(drive_commands(world, forward=5000))
        assert np.isfinite(snapshot["pose"]).all()
        assert snapshot["pose"][0] > initial + .05
        assert "not official" in world.game["geometry_accuracy"]


def test_bad_binding_is_actionable_and_does_not_leak_connection():
    config = clean_config()
    config["motors"] = [{"port": 1, "joint": "no_such_joint"}]
    with pytest.raises(ValueError, match="missing URDF joint 'no_such_joint'"):
        World(config)


def test_unbound_active_motor_fails_and_gearing_mismatch_preserves_hardware():
    with World(clean_config()) as world:
        with pytest.raises(ValueError, match="Motor port 4.*no URDF joint binding"):
            world.step({"motors": {"4": {"mode": "voltage", "value": 5000}}})
        command = {"motors": {"15": {"mode": "voltage", "value": 5000, "gearset": 100}}}
        for _ in range(3):
            snapshot = world.step(command)
        motor = next(binding for binding in world.motor_bindings if binding["port"] == 15)
        assert motor["cartridge"] == "blue"
        assert motor["rpm"] == 600
        assert motor["stall_torque"] == .35
        assert len(snapshot["diagnostics"]) == 1
        assert "physical blue cartridge" in snapshot["diagnostics"][0]


def test_imported_field_can_supply_its_own_floor(tmp_path):
    game = tmp_path / "scene.json"
    game.write_text(json.dumps({"field": {"floor": False, "walls": False}, "objects": []}))
    config = clean_config()
    config["game_pack"] = str(game)
    with World(config) as world:
        assert not world.field_objects
        settle(world, 40)
        assert world.snapshot()["pose"][2] < 0


def test_tile_material_and_friction_are_shared_between_seasons():
    import pybullet as bullet
    actual = []
    for pack in ("practice.json", "override-2026-27.json"):
        config = clean_config()
        config.update(game_pack=str(ASSETS / "games" / pack), friction=1.15)
        with World(config) as world:
            floor = next(item for item in world.field_objects if item["name"] == "floor")
            actual.append(bullet.getDynamicsInfo(floor["id"], -1, physicsClientId=world.client)[1])
            assert world.game["field"]["material"] == "vex_foam_tile"
    assert actual == [1.15, 1.15]


def test_external_reduction_applies_physical_joint_torque_and_shaft_encoder_scale():
    states = []
    for ratio in (1, 3):
        config = clean_config()
        config.update(fixed_base=True, gravity=0,
                      motors=[{"port": 15, "joint": "front_left", "cartridge": "blue", "gear_ratio": ratio}])
        with World(config) as world:
            snapshot = world.step({"motors": {"15": {"mode": "torque", "value": .04}}})
            joint = next(item for item in world.joints if item["name"] == "front_left")
            states.append((joint["velocity"], snapshot["motors"]["15"]["velocity"]))
    # Same joint inertia, 3:1 output reduction gives 3x generalized torque and
    # acceleration. The motor shaft spins 3x faster than the output joint.
    assert states[1][0] == pytest.approx(states[0][0] * 3, rel=.01)
    assert states[1][1] == pytest.approx(states[1][0] * 3)


def test_hold_brake_recaptures_position_after_subsequent_drive():
    with World({**clean_config(), "fixed_base": True, "gravity": 0}) as world:
        motor = next(item for item in world.motor_bindings if item["port"] == 15)
        hold = {"mode": "velocity", "value": 0, "brake": "hold"}
        world._motor_torque(motor, 1.0, 0, hold)
        world._motor_torque(motor, 2.0, 1, {"mode": "velocity", "value": 100})
        world._motor_torque(motor, 3.0, 0, hold)
        assert world._hold_targets[15] == 3.0
        assert world._motor_torque(motor, 3.0, 1, {"mode": "velocity", "value": 0, "brake": "coast"}) == 0
        assert world._motor_torque(motor, 3.0, 1, {"mode": "velocity", "value": 0, "brake": "brake"}) < 0


def test_override_inventory_layout_and_white_perimeter():
    config = default_config()
    assert config["game_pack"].endswith("override-2026-27.json")
    game = json.loads((ASSETS / "games" / "override-2026-27.json").read_text(encoding="utf-8"))
    kinds = [item.get("kind") for item in game["objects"]]
    assert {kind: kinds.count(kind) for kind in ("pin", "cup", "goal", "loader", "toggle")} == {
        "pin": 37, "cup": 36, "goal": 9, "loader": 4, "toggle": 4}
    assert game["inventory"]["pins"]["total"] == 63
    assert game["inventory"]["cups"]["total"] == 56
    assert min(game["field"]["wall_color"][:3]) > .9
    red_goal = next(item for item in game["objects"] if item["name"] == "goal_red_5")
    assert red_goal["position"] == pytest.approx([-1.196117068, -.598058534, 0])
    assert kinds.count("white_panel") == 12
    assert game["field"]["walls"] is False  # Individual authored CAD panels.
    assert game["field"]["inner_panel_span"] == pytest.approx(3.570357986)
    manifest = json.loads((ASSETS / "cad_manifest.json").read_text(encoding="utf-8"))
    placements = {row["source_instance_index"]: row for row in manifest["field_placements"]["instances"]}
    for item in game["objects"]:
        if "source_instance_index" in item:
            row = placements[item["source_instance_index"]]
            assert item["position"] == pytest.approx(row["position"], abs=1e-10)
            assert item["rotation"] == pytest.approx(row["rpy"], abs=1e-10)


def test_cad_pin_has_tapered_ends_and_wide_flange():
    def vertices(name):
        return np.array([[float(v) for v in line.split()[1:4]]
                         for line in (ASSETS / "objects" / name).read_text().splitlines() if line.startswith("v ")])
    flange = vertices("pin_flange.obj")
    assert np.ptp(flange, axis=0) == pytest.approx([.080293, .076793, .016], abs=1e-8)
    taper = vertices("pin_taper_1.obj")
    assert max(taper[:, 2]) == pytest.approx(.0665)
    radii = np.linalg.norm(taper[:, :2], axis=1)
    assert min(taper[:, 2]) == pytest.approx(.010)
    assert radii[taper[:, 2] == .010].max() == pytest.approx(.029708910092)
    assert radii[taper[:, 2] == .0665].max() == pytest.approx(.017797517)
    assert min(taper[taper[:, 2] == .040, 1]) == pytest.approx(-.020029)
    tip = vertices("pin_tip_1.obj")
    assert max(tip[:, 2]) == pytest.approx(.0825)
    assert min(tip[tip[:, 2] == .080, 1]) == pytest.approx(-.017543638 + .00225)
    # The CAD resting pose must not start with the old 2.3 mm circular-tip
    # penetration into the floor. Both ends share the measured D-flat.
    rotation = np.array([[1, 0, 0], [0, math.cos(1.30947132), -math.sin(1.30947132)],
                         [0, math.sin(1.30947132), math.cos(1.30947132)]])
    points = np.concatenate([vertices(name) for name in
                            ("pin_root_-1.obj", "pin_root_1.obj", "pin_taper_-1.obj", "pin_taper_1.obj",
                             "pin_tip_-1.obj", "pin_tip_1.obj", "pin_flange.obj")])
    assert min((points @ rotation.T)[:, 2] + .035631617) > -.0002


def test_loaded_pin_convex_hull_preserves_main_taper_radius():
    import pybullet as bullet
    with World(clean_config()) as world:
        pin = world._add_object({"name": "taper_ray_probe", "kind": "pin", "fixed": True,
                                "urdf": str(ASSETS / "objects" / "pin_yellow_yellow.urdf"),
                                "position": [.5, .9, .2]}, ASSETS)
        probe = bullet.createMultiBody(0, bullet.createCollisionShape(
            bullet.GEOM_SPHERE, radius=.00001, physicsClientId=world.client), physicsClientId=world.client)
        # Closest-point contact queries test the rebuilt Bullet hull. Its convex
        # ray caster has a ~1 mm convergence tolerance and is unsuitable here.
        # Joining the 8.1 mm root fillet to the same hull creates a false bulge.
        for offset, expected_sign in [(0, -1), (.0002, 1)]:
            bullet.resetBasePositionAndOrientation(probe, [.5+.0275977+offset, .9, .22],
                                                    [0, 0, 0, 1], physicsClientId=world.client)
            distance = min(c[8] for c in bullet.getClosestPoints(pin, probe, .01, physicsClientId=world.client))
            assert 0 < expected_sign*distance < .0002


def test_fixed_base_preview_lift_moves_under_finite_motor_voltage():
    config = clean_config()
    config.update(robot_urdf=str(ASSETS / "robots" / "xdrive_claw.urdf"),
                  robot_config=str(ASSETS / "robots" / "xdrive_claw.json"), fixed_base=True)
    with World(config) as world:
        base = world.snapshot()["pose"]
        for _ in range(240):
            state = world.step({"motors": {"5": {"mode": "voltage", "value": 8000}}})
        height = next(row["position"] for row in state["joints"] if row["name"] == "lift")
        assert .015 < height < .067  # 100 rpm and a 40 mm screw lead.
        assert state["pose"] == pytest.approx(base)


def test_dynamic_cup_collision_preserves_both_open_ends():
    import pybullet as bullet
    with World(clean_config()) as world:
        cup = world._add_object({"name": "probe_cup", "urdf": str(ASSETS / "objects" / "cup_gray_up.urdf"),
                                  "fixed": False, "position": [-.8, .8, .001]}, ASSETS)
        through_centre = bullet.rayTest([-.8, .8, .4], [-.8, .8, -.1], physicsClientId=world.client)[0]
        through_rim = bullet.rayTest([-.765, .8, .4], [-.765, .8, -.1], physicsClientId=world.client)[0]
        assert through_centre[0] != cup
        assert through_rim[0] == cup


def test_authored_cup_pin_stack_settles_then_wakes_on_collision():
    import pybullet as bullet
    with World(clean_config()) as world:
        cup = world._add_object({"name": "resting_cup", "kind": "cup", "fixed": False,
                                "urdf": str(ASSETS / "objects" / "cup_gray_up.urdf"),
                                "position": [.5, .9, .00003265]}, ASSETS)
        pin = world._add_object({"name": "resting_pin", "kind": "pin", "fixed": False,
                                "urdf": str(ASSETS / "objects" / "pin_yellow_yellow.urdf"),
                                "position": [.5, .9, .16728265]}, ASSETS)
        for _ in range(1920):
            world.step()
        position = bullet.getBasePositionAndOrientation(pin, physicsClientId=world.client)[0]
        # CAD assembly coordinates are initial conditions. Faceted contacts may
        # settle several millimetres; require bounded settling, then true rest.
        assert position == pytest.approx([.5, .9, .16728265], abs=.01)
        assert bullet.getBaseVelocity(pin, physicsClientId=world.client) == ((0, 0, 0), (0, 0, 0))
        for _ in range(240):
            world.step()
        assert bullet.getBasePositionAndOrientation(pin, physicsClientId=world.client)[0] == pytest.approx(position, abs=1e-8)
        # A moving test fixture supplies the impact. The resting objects are
        # never manually woken, attached or repositioned by the simulator.
        ball = world._add_object({"name": "impactor", "shape": "sphere", "mass": .2,
                                 "radius": .025, "position": [.68, .9, .04]}, ASSETS)
        bullet.resetBaseVelocity(ball, [-.8, 0, 0], physicsClientId=world.client)
        peak_speed = 0.
        for _ in range(180):
            world.step()
            velocity = bullet.getBaseVelocity(cup, physicsClientId=world.client)[0]
            peak_speed = max(peak_speed, math.hypot(velocity[0], velocity[1]))
        moved = bullet.getBasePositionAndOrientation(cup, physicsClientId=world.client)[0]
        assert peak_speed > .01
        assert abs(moved[0] - .5) > .001


def test_bounded_rest_goal_pin_sleeps_and_contact_wakes_it():
    import pybullet as bullet
    with World(clean_config()) as world:
        world._add_object({"name": "rest_goal", "kind": "goal", "fixed": True,
                           "urdf": str(ASSETS / "objects" / "goal_red.urdf"),
                           "position": [.5, .9, 0]}, ASSETS)
        pin = world._add_object({"name": "rest_pin", "kind": "pin", "fixed": False,
                                "urdf": str(ASSETS / "objects" / "pin_yellow_yellow.urdf"),
                                "position": [.5, .9, .0845]}, ASSETS)
        for _ in range(1920):
            world.step()
        position = bullet.getBasePositionAndOrientation(pin, physicsClientId=world.client)[0]
        assert position[:2] == pytest.approx([.5, .9], abs=.001)
        assert .082 < position[2] < .091
        assert bullet.getBaseVelocity(pin, physicsClientId=world.client) == ((0, 0, 0), (0, 0, 0))
        ball = world._add_object({"name": "goal_impactor", "shape": "sphere", "mass": .2,
                                 "radius": .025, "position": [.65, .9, .14]}, ASSETS)
        bullet.resetBaseVelocity(ball, [-1.5, 0, .7], physicsClientId=world.client)
        peak_speed = 0.
        for _ in range(180):
            world.step()
            velocity = bullet.getBaseVelocity(pin, physicsClientId=world.client)[0]
            peak_speed = max(peak_speed, math.hypot(velocity[0], velocity[1]))
        assert peak_speed > .01


def test_bounded_rest_preserves_continuous_slow_rolling():
    import pybullet as bullet
    with World(clean_config()) as world:
        ball = world._add_object({"name": "slow_roller", "shape": "sphere", "mass": .2,
                                 "radius": .025, "rolling_friction": 0,
                                 "position": [-.9, 1.3, .025]}, ASSETS)
        bullet.changeDynamics(ball, -1, linearDamping=0, angularDamping=0, physicsClientId=world.client)
        floor = next(item["id"] for item in world.field_objects if item["name"] == "floor")
        bullet.changeDynamics(floor, -1, rollingFriction=0, physicsClientId=world.client)
        bullet.resetBaseVelocity(ball, [.02, 0, 0], [0, .8, 0], physicsClientId=world.client)
        for _ in range(720):
            world.step()
        position = bullet.getBasePositionAndOrientation(ball, physicsClientId=world.client)[0]
        velocity = bullet.getBaseVelocity(ball, physicsClientId=world.client)[0]
        assert .04 < position[0] + .9 < .08
        assert velocity[0] > .012  # Two seconds of real motion must not be erased.


def test_rest_exclusion_follows_stationary_prop_chain_to_robot(tmp_path):
    import pybullet as bullet
    robot = tmp_path / "fixed_block.urdf"
    robot.write_text('''<robot name="fixture"><link name="base"><inertial><mass value="1"/>
      <inertia ixx=".01" iyy=".01" izz=".01" ixy="0" ixz="0" iyz="0"/></inertial>
      <collision><geometry><box size=".1 .1 .1"/></geometry></collision></link></robot>''')
    config = clean_config()
    config.update(robot_urdf=str(robot), robot_config="", motors=[], fixed_base=True,
                  start_pose=[0, 0, .05, 0])
    with World(config) as world:
        neighbour = world._add_object({"name": "robot_neighbour", "mass": .075,
                                       "size": [.02, .02, .02], "position": [-.05999, 0, .01]}, ASSETS)
        outer = world._add_object({"name": "outer_prop", "mass": .075,
                                   "size": [.02, .02, .02], "position": [-.07998, 0, .01]}, ASSETS)
        for _ in range(720):
            # Static friction balances a small external load while maintaining
            # a real contact chain into the fixed robot fixture.
            bullet.applyExternalForce(outer, -1, [.05, 0, 0], [0, 0, 0], bullet.LINK_FRAME,
                                      physicsClientId=world.client)
            bullet.stepSimulation(physicsClientId=world.client)
            world.tick += 1
            world.time = world.tick * world.dt
            # Supply a controlled, one-micrometre/second solver residual on A
            # and an exact-zero sample on B. Their physical contacts and poses
            # still come from Bullet, while this reproduces the exclusion gap.
            bullet.resetBaseVelocity(outer, [1e-6, 0, 0], [0, 0, 0], physicsClientId=world.client)
            bullet.resetBaseVelocity(neighbour, [0, 0, 0], [0, 0, 0], physicsClientId=world.client)
            world._deactivate_bounded_rest()
            assert bullet.getBaseVelocity(outer, physicsClientId=world.client)[0][0] == 1e-6
        assert bullet.getContactPoints(outer, neighbour, physicsClientId=world.client)
        assert bullet.getContactPoints(neighbour, world.robot, physicsClientId=world.client)
        # An exact-zero sample can occur during a pause or turning point. It
        # must not hide the neighbour's contact with the robot from the group.
        bullet.resetBaseVelocity(neighbour, [0, 0, 0], [0, 0, 0], physicsClientId=world.client)
        before = bullet.getBaseVelocity(outer, physicsClientId=world.client)
        assert any(v != 0 for vector in before for v in vector)
        world._deactivate_bounded_rest()
        assert bullet.getBaseVelocity(outer, physicsClientId=world.client) == before


@pytest.mark.parametrize("height", [.035631617, .25], ids=["rolling", "falling"])
def test_cad_pin_contacts_do_not_add_mechanical_energy(height):
    import pybullet as bullet
    with World(clean_config()) as world:
        pin = world._add_object({"name": "energy_pin", "kind": "pin", "fixed": False,
                                "urdf": str(ASSETS / "objects" / "pin_yellow_yellow.urdf"),
                                "position": [.5, .9, height], "rotation": [1.30947132, 0, 0]}, ASSETS)
        bullet.resetBaseVelocity(pin, [.1, 0, 0], [0, 2.5, -.6686], physicsClientId=world.client)
        dynamics = bullet.getDynamicsInfo(pin, -1, physicsClientId=world.client)
        mass, inertia = dynamics[0], dynamics[2]

        def energy():
            position, quaternion = bullet.getBasePositionAndOrientation(pin, physicsClientId=world.client)
            linear, angular = bullet.getBaseVelocity(pin, physicsClientId=world.client)
            local_angular = np.array(bullet.getMatrixFromQuaternion(quaternion)).reshape(3, 3).T @ angular
            return (mass * world.config.gravity * position[2] + .5*mass*sum(v*v for v in linear)
                    + .5*sum(i*v*v for i, v in zip(inertia, local_angular)))

        initial, peak = energy(), energy()
        for _ in range(480):
            world.step()
            peak = max(peak, energy())
        assert peak <= initial * 1.01  # Allow integration error, not friction-driven acceleration.
        assert energy() < initial


def test_claw_grips_lifts_and_releases_a_pin_into_physical_socket():
    import pybullet as bullet
    config = clean_config()
    config.update(robot_urdf=str(ASSETS / "robots" / "xdrive_claw.urdf"),
                  robot_config=str(ASSETS / "robots" / "xdrive_claw.json"),
                  start_pose=[-1.2, -1.2, .075, 0], fixed_base=True,
                  field_objects=[{"name": "grasp_pin", "kind": "pin", "fixed": False,
                                  "urdf": str(ASSETS / "objects" / "pin_red_yellow.urdf"),
                                  "position": [-.86, -1.2, .083]}])
    with World(config) as world:
        pin = next(item["id"] for item in world.field_objects if item["name"] == "grasp_pin")
        commands = {"motors": {"5": {"mode": "position", "value": 0},
                                "6": {"mode": "position", "value": -144}}}
        for _ in range(600):
            world.step(commands)
        start_height = bullet.getBasePositionAndOrientation(pin, physicsClientId=world.client)[0][2]
        commands["motors"]["5"]["value"] = 1800
        for _ in range(960):
            world.step(commands)
        lifted = bullet.getBasePositionAndOrientation(pin, physicsClientId=world.client)[0][2]
        assert lifted > start_height + .18
        assert bullet.getContactPoints(pin, world.robot, physicsClientId=world.client)
        # Introduce the receptacle below the suspended pin to isolate placement
        # from driving. The pin is never teleported, welded or attached.
        goal = world._add_object({"name": "receptacle", "urdf": str(ASSETS / "objects" / "goal_red.urdf"),
                                  "fixed": True, "position": [-.86, -1.2, 0],
                                  "scoring": {"type": "goal", "height": .08253265, "socket_radius": .0384, "socket_depth": .08253265}}, ASSETS)
        assert world.snapshot()["scoring"]["placed_pins"] == 0  # Suspended in the robot.
        commands["motors"]["5"]["value"] = 900
        for _ in range(480):
            world.step(commands)
        commands["motors"]["6"]["value"] = 720
        for _ in range(600):
            world.step(commands)
        commands["motors"]["5"]["value"] = 1800
        for _ in range(480):
            state = world.step(commands)
        assert state["scoring"]["placed_pins"] == 1
        assert state["scoring"]["placements"][0]["pin"] == "grasp_pin"
        # Re-grip the seated piece without lifting it. This prototype counter
        # deliberately requires release, even while goal contact is retained.
        commands["motors"]["5"]["value"] = 0
        for _ in range(960):
            world.step(commands)
        commands["motors"]["6"]["value"] = -144
        for _ in range(600):
            state = world.step(commands)
        assert bullet.getContactPoints(pin, goal, physicsClientId=world.client)
        assert bullet.getContactPoints(pin, world.robot, physicsClientId=world.client)
        assert state["scoring"]["placed_pins"] == 0


@pytest.mark.parametrize("cartridge,rpm", [("red", 100), ("green", 200), ("blue", 600)])
def test_no_load_cartridge_speed_in_a_physical_flywheel(tmp_path, cartridge, rpm):
    # Known inertia isolates the shaft/governor from wheel traction and contact
    # variation. Mechanical inertia scales as a cartridge's squared reduction.
    inertia = .02 * (100 / rpm) ** 2
    urdf = tmp_path / "flywheel.urdf"
    urdf.write_text(f'''<robot name="flywheel"><link name="base"><inertial><mass value="1"/>
      <inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <link name="rotor"><inertial><mass value=".5"/>
      <inertia ixx="{inertia}" iyy="{inertia}" izz="{inertia}" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <joint name="shaft" type="continuous"><parent link="base"/><child link="rotor"/>
      <axis xyz="0 0 1"/><limit effort="10" velocity="200"/></joint></robot>''')
    binding = tmp_path / "flywheel.json"
    binding.write_text(json.dumps({"fixed_base": True, "motors": [{"port": 1, "joint": "shaft", "cartridge": cartridge}]}))
    config = clean_config()
    config.update(robot_urdf=str(urdf), robot_config=str(binding), start_pose=[0, 0, 1, 0])
    with World(config) as world:
        velocities = []
        for tick in range(720):
            snapshot = world.step({"motors": {"1": {"mode": "voltage", "value": 12000}}})
            if tick >= 600:
                velocities.append(snapshot["motors"]["1"]["velocity"] * 30 / math.pi)
        assert np.mean(velocities) == pytest.approx(rpm, rel=.025)
        assert max(velocities) < rpm * 1.05
        assert max(m["current_estimate"] for m in snapshot["motors"].values()) <= 2.5
        # A direct shaft torque must replace the prior velocity constraint;
        # retaining that servo would keep the rotor at its previous setpoint.
        stall = world.motor_bindings[0]["stall_torque"]
        for _ in range(60):
            snapshot = world.step({"motors": {"1": {"mode": "torque", "value": -.1*stall}}})
        assert snapshot["motors"]["1"]["velocity"] < .9*rpm*math.pi/30


def test_forward_kinematics_pose_reads_do_not_change_contact_physics():
    """The OpenGL renderer's pose queries must not alter the next tire impulse.

    Run the actual native link queries without importing VTK or opening a
    graphics context. Cached sensor reads alone did not expose this defect.
    """
    import pybullet as bullet
    config = clean_config()
    config["renderer"] = "cpu"
    config["noise"].update(position_std_m=.003, heading_std_deg=.3, encoder_std_deg=.2,
                           distance_std_mm=3, latency_ms=25, dropout=.2)
    with World(config) as observed, World(deepcopy(config)) as unread:
        bodies = [observed.robot] + [item["id"] for item in observed.field_objects]
        links = sorted({(body, shape[1]) for body in bodies
                        for shape in bullet.getVisualShapeData(body, physicsClientId=observed.client)
                        if shape[1] >= 0})
        assert links, "Fixture must contain articulated visuals"
        for tick in range(60):
            state = observed.step(drive_commands(observed, forward=2500))
            if tick % 20 == 0:
                for body, link in links:
                    bullet.getLinkState(body, link, computeForwardKinematics=1,
                                        physicsClientId=observed.client)
                assert observed.snapshot() == state
            reference = unread.step(drive_commands(unread, forward=2500))
            assert state == reference
