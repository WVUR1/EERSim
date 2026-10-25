"""Native checks for MuJoCo import, inertia and collision topology."""
import json
import math
from pathlib import Path

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("pybullet")

from eersim.physics import World
from eersim.mujoco_scene import compile_scene


def fixture_config(tmp_path, fixed=False):
    inertia = '<inertial><origin xyz=".012 -.008 .015" rpy=".1 -.2 .3"/><mass value="1"/><inertia ixx=".02" iyy=".03" izz=".04" ixy="0" ixz="0" iyz="0"/></inertial>'
    def link(name):
        return f'<link name="{name}">{inertia}<collision><origin xyz=".01 .02 .03" rpy=".1 .2 .3"/><geometry><box size=".1 .08 .06"/></geometry></collision></link>'
    robot = tmp_path / "robot.urdf"
    robot.write_text('<robot name="fixture">' + link("base") + link("arm") + link("slider") + link("fixed") + '''
      <joint name="slide" type="prismatic"><parent link="arm"/><child link="slider"/>
       <origin xyz=".1 0 .05" rpy=".2 .1 -.1"/><axis xyz="0 1 0"/><limit lower="0" upper=".1" effort="25" velocity=".5"/></joint>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
       <origin xyz="0 0 .2" rpy="0 .2 .1"/><axis xyz="0 0 -1"/><limit lower="-1" upper="1" effort="2" velocity="10"/></joint>
      <joint name="mount" type="fixed"><parent link="slider"/><child link="fixed"/><origin xyz=".2 0 .1"/></joint>
      </robot>''')
    settings = tmp_path / "robot.json"
    settings.write_text(json.dumps({"initial_joints": {"slide": .04, "hinge": .3}, "motors": []}))
    field = tmp_path / "field.json"
    field.write_text(json.dumps({"field": {"size": [2, 2], "floor": False, "walls": False},
                                "objects": [{"name": "floor", "shape": "box", "size": [2, 2, .1], "position": [0, 0, -.05]}]}))
    return {"robot_urdf": str(robot), "robot_config": str(settings), "game_pack": str(field),
            "start_pose": [.2, -.3, .4, 27], "fixed_base": fixed, "physics_backend": "bullet"}


@pytest.mark.parametrize("fixed", [False, True])
def test_import_preserves_link_and_com_poses_mass_inertia_and_joint_state(tmp_path, fixed):
    with World(fixture_config(tmp_path, fixed)) as world:
        before = world.snapshot()
        scene = compile_scene(world)
        assert world.snapshot() == before
        assert scene.data.time == 0
        assert (world.robot in scene.free_joints) is not fixed
        assert len(scene.joint_ids) == 2
        assert len(scene.actuator_ids) == 2
        assert len(scene.body_ids) == 5
        assert scene.model.opt.timestep == pytest.approx(world.dt)
        for (body, link), mjbody in scene.body_ids.items():
            dynamics = scene.dynamics[body, link]
            if link < 0:
                com = world._physics.getBasePositionAndOrientation(body, physicsClientId=world.client)
                link_pose = world._physics.multiplyTransforms(*com, *world._physics.invertTransform(dynamics[3], dynamics[4]))
            else:
                state = world._physics.getLinkState(body, link, physicsClientId=world.client)
                com, link_pose = state[:2], state[4:6]
            assert scene.data.xpos[mjbody] == pytest.approx(link_pose[0], abs=2e-7)
            assert scene.data.xipos[mjbody] == pytest.approx(com[0], abs=2e-7)
            if dynamics[0] > 0:
                assert scene.model.body_mass[mjbody] == pytest.approx(dynamics[0])
                assert scene.model.body_inertia[mjbody] == pytest.approx(dynamics[2])
                assert scene.model.body_ipos[mjbody] == pytest.approx(dynamics[3])
        for key, joint in scene.joint_ids.items():
            expected = world._physics.getJointState(*key, physicsClientId=world.client)
            assert scene.data.qpos[scene.model.jnt_qposadr[joint]] == pytest.approx(expected[0])
        assert scene.model.nexclude == 6


def test_static_concave_mesh_retains_each_triangle_and_opening(tmp_path):
    config = fixture_config(tmp_path)
    # Two disjoint vertical plates with an open central gap. A convex hull
    # would incorrectly close the gap; a rigid flex retains the four faces.
    vertices = np.array([[-.2, -.1, 0], [-.2, .1, 0], [-.2, .1, .2], [-.2, -.1, .2],
                         [.2, -.1, 0], [.2, .1, 0], [.2, .1, .2], [.2, -.1, .2]])
    faces = [(0, 1, 2), (0, 2, 3), (4, 5, 6), (4, 6, 7)]
    mesh = tmp_path / "concave.obj"
    mesh.write_text("\n".join("v " + " ".join(map(str, vertex)) for vertex in vertices) + "\n" +
                    "\n".join("f " + " ".join(str(index+1) for index in face) for face in faces))
    urdf = tmp_path / "fixture.urdf"
    urdf.write_text('<robot name="static"><link name="fixture" concave="yes"><inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0"/></inertial><collision><origin xyz=".1 0 .2"/><geometry><mesh filename="concave.obj"/></geometry></collision></link></robot>')
    field = Path(config["game_pack"])
    game = json.loads(field.read_text())
    game["objects"].append({"urdf": str(urdf), "fixed": True})
    field.write_text(json.dumps(game))
    with World(config) as world:
        scene = compile_scene(world)
        assert scene.model.nflex == 1
        assert scene.model.flex_rigid[0]
        assert scene.model.flex_dim[0] == 2
        assert scene.model.flex_radius[0] == pytest.approx(1e-6)
        assert scene.model.flex_elemnum[0] == 4
        assert scene.model.flex_vertnum[0] == 8
        assert scene.model.flex_vert == pytest.approx(vertices + [.1, 0, .2])
        assert tuple(scene.model.flex_elem.reshape(-1, 3)[0]) == faces[0]
        assert scene.flex_keys[0] == (1, -1)


@pytest.mark.parametrize("feature", ["mimic", "safety_controller", "calibration"])
def test_rejects_unsupported_joint_features_instead_of_discarding_them(tmp_path, feature):
    config = fixture_config(tmp_path)
    with World(config) as world:
        path = Path(config["robot_urdf"])
        path.write_text(path.read_text().replace('</joint>', f'<{feature} joint="hinge"/></joint>', 1))
        with pytest.raises(ValueError, match=feature):
            compile_scene(world)


def test_obj_welding_preserves_every_triangle_coordinate(tmp_path):
    from eersim.mujoco_scene import _mesh_triangles
    # Two faces use duplicated coordinates at a visual seam.
    vertices = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0],
                         [0, 0, 0], [1, 1, 0], [0, 1, 0]])
    path = tmp_path / "seam.obj"
    path.write_text("\n".join("v " + " ".join(map(str, p)) for p in vertices)
                    + "\nf 1 2 3\nf 4 5 6\n")
    points, faces = _mesh_triangles(path)
    assert len(points) == 4
    assert len(faces) == 2
    np.testing.assert_array_equal(points[faces], vertices.reshape(2, 3, 3))


def test_native_workers_preserve_collision_trajectory(tmp_path):
    from eersim.mujoco_scene import physics_worker_count
    config = fixture_config(tmp_path)
    config['physics_cpu_threads'] = 0
    with World(config) as world:
        serial = compile_scene(world)
        world.config.physics_cpu_threads = 2
        parallel = compile_scene(world)
        assert serial.data.threadpool == 0
        assert parallel.cpu_threads == physics_worker_count(2)
        assert bool(parallel.data.threadpool) == bool(parallel.cpu_threads)
        contact_seen = False
        for _ in range(160):
            mujoco.mj_step(serial.model, serial.data)
            mujoco.mj_step(parallel.model, parallel.data)
            contact_seen |= serial.data.ncon > 0
            np.testing.assert_allclose(serial.data.qpos, parallel.data.qpos, atol=1e-12, rtol=0)
            np.testing.assert_allclose(serial.data.qvel, parallel.data.qvel, atol=1e-12, rtol=0)
        assert contact_seen


@pytest.mark.parametrize("fixed", [False, True])
def test_native_sleep_preserves_moving_props_and_motor_responsiveness(tmp_path, fixed):
    config = fixture_config(tmp_path, fixed)
    config.update(start_pose=[.7, .7, .4, 0], gravity=0, physics_cpu_threads=0)
    with World(config) as world:
        still = world._add_object({"shape": "sphere", "radius": .04, "mass": .1,
                                   "position": [-.7, -.7, .3]}, tmp_path)
        moving = world._add_object({"shape": "sphere", "radius": .04, "mass": .1,
                                    "position": [-.7, .5, .3]}, tmp_path)
        world._physics.resetBaseVelocity(moving, [.02, 0, 0], physicsClientId=world.client)
        scene = compile_scene(world)
        model, data = scene.model, scene.data
        assert model.opt.enableflags & mujoco.mjtEnableBit.mjENBL_SLEEP
        assert model.opt.sleep_tolerance == pytest.approx(.001)
        robot_trees = {int(model.body_treeid[native]) for (body, _), native in scene.body_ids.items()
                       if body == world.robot and model.body_treeid[native] >= 0}
        assert robot_trees
        assert all(model.tree_sleep_policy[tree] == mujoco.mjtSleepPolicy.mjSLEEP_NEVER
                   for tree in robot_trees)
        still_tree = model.body_treeid[scene.body_ids[still, -1]]
        moving_tree = model.body_treeid[scene.body_ids[moving, -1]]
        assert model.tree_sleep_policy[still_tree] == mujoco.mjtSleepPolicy.mjSLEEP_ALLOWED
        for _ in range(100):
            mujoco.mj_step(model, data)
        assert data.tree_asleep[still_tree] >= 0
        assert data.tree_asleep[moving_tree] < 0
        moving_joint = scene.free_joints[moving]
        assert data.qpos[model.jnt_qposadr[moving_joint]] == pytest.approx(-.7 + 2 * world.dt)
        assert all(data.tree_asleep[tree] < 0 for tree in robot_trees)
        key, actuator = next(iter(scene.actuator_ids.items()))
        dof = model.jnt_dofadr[scene.joint_ids[key]]
        before = data.qvel[dof]
        data.ctrl[actuator] = .1
        mujoco.mj_step(model, data)
        assert abs(data.qvel[dof] - before) > 1e-6


def test_passive_prop_sleeps_on_world_welded_flex_and_wakes_on_impact(tmp_path):
    config = fixture_config(tmp_path, fixed=True)
    config.update(start_pose=[.7, .7, .4, 0], physics_cpu_threads=0, restitution=0)
    plate = tmp_path / 'sleep_plate.obj'
    plate.write_text('v -1 -1 0\nv 0 -1 0\nv 0 0 0\nv -1 0 0\nf 1 2 3\nf 1 3 4\n')
    fixture = tmp_path / 'sleep_plate.urdf'
    fixture.write_text('<robot name="plate"><link name="plate" concave="yes">'
                       '<inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" '
                       'ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry>'
                       '<mesh filename="sleep_plate.obj"/></geometry></collision></link></robot>')
    with World(config) as world:
        world._add_object({"urdf": str(fixture), "fixed": True, "position": [0, 0, .2]}, tmp_path)
        target = world._add_object({"shape": "sphere", "radius": .05, "mass": .2,
                                    "position": [-.5, -.5, .5]}, tmp_path)
        ball = world._add_object({"shape": "sphere", "radius": .04, "mass": .2,
                                  "position": [-.8, -.5, .3]}, tmp_path)
        scene = compile_scene(world)
        model, data = scene.model, scene.data
        assert model.nflex == 1 and model.flex_rigid[0]
        assert all(model.body_treeid[body] < 0 for body in model.flex_vertbodyid)
        target_native, ball_native = scene.body_ids[target, -1], scene.body_ids[ball, -1]
        target_tree, ball_tree = model.body_treeid[[target_native, ball_native]]
        target_joint = scene.free_joints[target]
        target_pos = model.jnt_qposadr[target_joint]
        flex_contact_seen = False
        for _ in range(600):
            mujoco.mj_step(model, data)
            flex_contact_seen |= bool(np.any(data.contact.flex[:data.ncon] >= 0))
        assert flex_contact_seen
        assert data.qpos[target_pos + 2] == pytest.approx(.25, abs=.002)
        assert data.tree_asleep[target_tree] >= 0
        assert data.tree_asleep[ball_tree] >= 0
        original_x = float(data.qpos[target_pos])
        data.xfrc_applied[ball_native, 0] = 10.
        for _ in range(10):
            mujoco.mj_step(model, data)
        data.xfrc_applied[ball_native, 0] = 0.
        assert data.tree_asleep[ball_tree] < 0
        collision_woke_target = False
        for _ in range(120):
            mujoco.mj_step(model, data)
            for contact in data.contact[:data.ncon]:
                if np.all(contact.geom >= 0) and set(model.geom_bodyid[contact.geom]) == {target_native, ball_native}:
                    collision_woke_target |= data.tree_asleep[target_tree] < 0
        assert collision_woke_target
        assert data.qpos[target_pos] > original_x + .001


def test_fixed_mounts_keep_separate_robot_branches_awake_but_allow_passive_branches_to_sleep(tmp_path):
    config = fixture_config(tmp_path, fixed=True)
    config.update(start_pose=[.4, 0, .3, 0], gravity=0, physics_cpu_threads=0)
    links = ''.join('<link name="' + name + '"><inertial><mass value=".1"/>'
                    '<inertia ixx=".001" iyy=".001" izz=".001" ixy="0" ixz="0" iyz="0"/>'
                    '</inertial><collision><geometry><sphere radius=".01"/></geometry></collision></link>'
                    for name in ('base', 'mount', 'left', 'right'))
    robot = Path(config['robot_urdf'])
    robot.write_text('<robot name="branches">' + links + '''
      <joint name="mount" type="fixed"><parent link="base"/><child link="mount"/>
       <origin xyz="0 0 .1"/></joint>
      <joint name="left" type="continuous"><parent link="mount"/><child link="left"/>
       <origin xyz="0 .1 0"/><axis xyz="0 0 1"/></joint>
      <joint name="right" type="continuous"><parent link="mount"/><child link="right"/>
       <origin xyz="0 -.1 0"/><axis xyz="0 0 1"/></joint></robot>''')
    Path(config['robot_config']).write_text('{"motors": []}')
    with World(config) as world:
        passive = world._add_object({'urdf': str(robot), 'fixed': True,
                                      'position': [-.4, 0, .3]}, tmp_path)
        scene = compile_scene(world)
        model, data = scene.model, scene.data
        def trees(body):
            return {int(model.body_treeid[native]) for (source, _), native in scene.body_ids.items()
                    if source == body and model.body_treeid[native] >= 0}
        robot_trees, passive_trees = trees(world.robot), trees(passive)
        assert len(robot_trees) == len(passive_trees) == 2
        for _ in range(100):
            mujoco.mj_step(model, data)
        assert all(data.tree_asleep[tree] < 0 for tree in robot_trees)
        assert all(data.tree_asleep[tree] >= 0 for tree in passive_trees)
        for key, actuator in scene.actuator_ids.items():
            data.ctrl[actuator] = .1
        mujoco.mj_step(model, data)
        assert all(abs(data.qvel[model.jnt_dofadr[scene.joint_ids[key]]]) > 1e-6
                   for key in scene.actuator_ids)


@pytest.mark.parametrize("hz", [120, 240])
def test_configured_restitution_changes_rebound_without_adding_energy(tmp_path, hz):
    from eersim.mujoco_world import MuJoCoWorld
    robot = tmp_path / 'sphere.urdf'
    robot.write_text('<robot name="drop"><link name="sphere"><inertial><mass value=".1"/>'
                     '<inertia ixx=".0001" iyy=".0001" izz=".0001" ixy="0" ixz="0" iyz="0"/>'
                     '</inertial><collision><geometry><sphere radius=".05"/></geometry>'
                     '</collision></link></robot>')
    settings = tmp_path / 'robot.json'
    settings.write_text('{"motors": []}')
    field = tmp_path / 'field.json'
    field.write_text(json.dumps({'field': {'size': [2, 2], 'floor': False, 'walls': False},
                                 'objects': [{'shape': 'box', 'size': [2, 2, .1],
                                              'position': [0, 0, -.05]}]}))
    rebounds = []
    for restitution in (0., .2, .5, .8):
        # The imported materials combine by product, so each body gets sqrt(e).
        config = dict(robot_urdf=str(robot), robot_config=str(settings), game_pack=str(field),
                      start_pose=[0, 0, .5, 0], physics_backend='mujoco', physics_hz=hz,
                      physics_cpu_threads=0, restitution=math.sqrt(restitution))
        with MuJoCoWorld(config) as world:
            down, up = 0., 0.
            contact_seen = False
            for _ in range(hz):
                world.step()
                velocity = world._physics.getBaseVelocity(world.robot)[0][2]
                down, up = min(down, velocity), max(up, velocity)
                contact_seen |= world._physics.data.ncon > 0
            assert contact_seen and down < -2.
            rebound = up / -down
            # This is a compliance mapping, not exact impulse restitution.
            # It must remain dissipative and distinguish low/high settings.
            assert 0 <= rebound <= 1.05
            rebounds.append(rebound)
    assert all(a < b for a, b in zip(rebounds, rebounds[1:]))
    assert rebounds[-1] - rebounds[0] > .4


def test_moving_box_contacts_large_static_concave_triangle(tmp_path):
    from eersim.mujoco_world import MuJoCoWorld
    robot = tmp_path / 'box.urdf'
    robot.write_text('<robot name="drop"><link name="box"><inertial><mass value="1"/>'
                     '<inertia ixx=".0016666667" iyy=".0016666667" izz=".0016666667" '
                     'ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry>'
                     '<box size=".1 .1 .1"/></geometry></collision></link></robot>')
    settings = tmp_path / 'robot.json'
    settings.write_text('{"motors": []}')
    plate = tmp_path / 'plate.obj'
    plate.write_text('v -.5 -.5 0\nv .5 -.5 0\nv .5 .5 0\nv -.5 .5 0\nf 1 2 3\nf 1 3 4\n')
    fixture = tmp_path / 'plate.urdf'
    fixture.write_text('<robot name="plate"><link name="plate" concave="yes">'
                       '<inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" '
                       'ixy="0" ixz="0" iyz="0"/></inertial><collision><geometry>'
                       '<mesh filename="plate.obj"/></geometry></collision></link></robot>')
    field = tmp_path / 'field.json'
    field.write_text(json.dumps({'field': {'size': [2, 2], 'floor': False, 'walls': False},
                                 'objects': [{'urdf': str(fixture), 'fixed': True,
                                              'position': [0, 0, .2]}]}))
    config = dict(robot_urdf=str(robot), robot_config=str(settings), game_pack=str(field),
                  start_pose=[.1, -.1, .7, 0], physics_backend='mujoco', physics_hz=240,
                  physics_cpu_threads=0, restitution=0)
    with MuJoCoWorld(config) as world:
        adapter = world._physics
        model, data = adapter.model, adapter.data
        geom = next(g for g, key in adapter.scene.geom_keys.items() if key == (world.robot, -1))
        assert model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        vertices = model.mesh_vert[start:start+count]
        # The imported mesh is exactly the requested cuboid, including width.
        assert count == 8
        np.testing.assert_allclose(np.ptp(vertices, axis=0), [.1, .1, .1], atol=1e-8)
        assert model.body_mass[adapter.body_ids[world.robot, -1]] == 1.
        flex_contact = False
        for _ in range(240):
            world.step()
            flex_contact |= bool(np.any(data.contact.flex[:data.ncon] >= 0))
        assert flex_contact
        position = adapter.getBasePositionAndOrientation(world.robot)[0]
        assert position[2] == pytest.approx(.25, abs=.005)


@pytest.mark.parametrize("hz", [120, 240])
def test_bundled_goal_retains_dropped_pin_with_actual_contact(tmp_path, hz):
    """A socket must stay open and retain the official pin collision leaves."""
    from eersim.config import ASSETS
    from eersim.mujoco_world import MuJoCoWorld
    config = fixture_config(tmp_path, fixed=True)
    config.update(start_pose=[.7, .7, .4, 0], physics_hz=hz,
                  physics_backend="mujoco", physics_cpu_threads=0)
    game = json.loads((ASSETS / "games" / "override-2026-27.json").read_text())
    goal = next(item for item in game["objects"] if item["name"] == "goal_red_5")
    goal.update(urdf=str(ASSETS / "objects" / "goal_red.urdf"), position=[0, 0, 0])
    pin = {"name": "falling_pin", "urdf": str(ASSETS / "objects" / "pin_red_yellow.urdf"),
           "fixed": False, "kind": "pin", "position": [0, 0, .20]}
    field = {"field": {"size": [2, 2], "floor": True, "walls": False},
             "objects": [goal, pin]}
    Path(config["game_pack"]).write_text(json.dumps(field))
    with MuJoCoWorld(config) as world:
        bodies = {item["name"]: item["id"] for item in world.field_objects}
        contacts = 0
        for _ in range(hz):
            world.step()
            contacts += world._has_contact(bodies["falling_pin"], bodies["goal_red_5"])
        xyz, quaternion = world._physics.getBasePositionAndOrientation(bodies["falling_pin"])
        axis = np.asarray(world._physics.getMatrixFromQuaternion(quaternion)).reshape(3, 3)[:, 2]
        assert contacts > hz / 2
        assert np.linalg.norm(xyz[:2]) < .002
        assert .08 < xyz[2] < .09
        assert abs(axis[2]) > .99
        assert world._scoring()["placements"] == [
            {"pin": "falling_pin", "goal": "goal_red_5", "alliance": "red"}]
