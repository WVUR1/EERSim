"""Real loader/export checks without creating a GPU context or taking a step."""
import json
import math

import numpy as np
import pytest

pytest.importorskip('pybullet')
pytest.importorskip('scipy')
pytest.importorskip('pxr.Usd')
from pxr import Usd, UsdGeom, UsdPhysics

from eersim.physics import World
from eersim.physx_export import export_world


def fixture_config(tmp_path, fixed=False):
    inertia = '<inertial><origin xyz=".012 -.008 .015" rpy=".1 -.2 .3"/><mass value="1"/><inertia ixx=".02" iyy=".03" izz=".04" ixy="0" ixz="0" iyz="0"/></inertial>'
    link = lambda name: f'<link name="{name}">{inertia}<collision><origin xyz=".01 .02 .03" rpy=".1 .2 .3"/><geometry><box size=".1 .08 .06"/></geometry></collision></link>'
    # XML joint order differs from the loader's tree order deliberately.
    robot = tmp_path / 'robot.urdf'
    robot.write_text('<robot name="export">' + link('base') + link('arm') + link('slider') + '''
      <joint name="slide" type="prismatic"><parent link="arm"/><child link="slider"/>
       <origin xyz=".1 0 .05" rpy=".2 .1 -.1"/><axis xyz="0 1 0"/><limit lower="0" upper=".1" effort="25" velocity=".5"/></joint>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/>
       <origin xyz="0 0 .2" rpy="0 .2 .1"/><axis xyz="0 0 -1"/><limit lower="-1" upper="1" effort="2" velocity="10"/></joint>
      </robot>''')
    settings = tmp_path / 'robot.json'
    settings.write_text(json.dumps({'initial_joints': {'slide': .04, 'hinge': .3}, 'motors': []}))
    field = tmp_path / 'field.json'
    field.write_text(json.dumps({'field': {'size': [2, 2], 'floor': False, 'walls': False},
                                'objects': [{'name': 'floor', 'shape': 'box', 'size': [2, 2, .1], 'position': [0, 0, -.05]}]}))
    return {'robot_urdf': str(robot), 'robot_config': str(settings), 'game_pack': str(field),
            'start_pose': [.2, -.3, .4, 27], 'fixed_base': fixed}


def test_articulation_export_maps_loaded_joint_indices_and_nonzero_state(tmp_path):
    with World(fixture_config(tmp_path)) as world:
        before = world.snapshot()
        manifest = export_world(world, tmp_path / 'export', backend='gpu', iterations=100)
        assert world.snapshot() == before  # Export must not bootstrap or advance physics.
        assert len(world._export_items) == 1
        assert manifest['counts']['collision_shapes'] == 4
        assert manifest['solver']['position_iterations'] == 100
        stage = Usd.Stage.Open(str(tmp_path / 'export' / 'field.usda'))
        root = stage.GetPrimAtPath(manifest['robot']['articulation_root'])
        assert root.HasAPI(UsdPhysics.ArticulationRootAPI)
        assert root.GetAttribute('physxArticulation:enabledSelfCollisions').Get() is False
        joints = {row['name']: row for row in manifest['joints']}
        assert joints['slide']['joint_index'] == 1
        assert joints['hinge']['joint_index'] == 0
        for name, expected, axis in [('slide', .04, 'linear'), ('hinge', math.degrees(.3), 'angular')]:
            prim = stage.GetPrimAtPath(joints[name]['path'])
            assert prim.GetAttribute(f'state:{axis}:physics:position').Get() == pytest.approx(expected)
            assert prim.GetAttribute(f'drive:{axis}:physics:type').Get() == 'force'
            assert prim.GetAttribute(f'drive:{axis}:physics:maxForce').Get() == 0
        for row in manifest['actors']:
            if row['mass_kg'] > 0:
                prim = stage.GetPrimAtPath(row['path'])
                # PhysX codeless APIs remain authored metadata when validating
                # with stock usd-core, which does not register NVIDIA schemas.
                assert 'PhysxContactReportAPI' in prim.GetMetadata('apiSchemas').GetAddedOrExplicitItems()
                assert prim.GetAttribute('physxContactReport:threshold').Get() == 0
        for row in manifest['actors']:
            if row['robot']:
                body, index = row['body_id'], row['link_index']
                if index < 0:
                    d = world._physics.getDynamicsInfo(body, index, physicsClientId=world.client)
                    pose = world._physics.multiplyTransforms(
                        *world._physics.getBasePositionAndOrientation(body, physicsClientId=world.client),
                        *world._physics.invertTransform(d[3], d[4]))
                else:
                    pose = world._physics.getLinkState(body, index, physicsClientId=world.client)[4:6]
                transform = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(row['path']))
                assert tuple(transform.ExtractTranslation()) == pytest.approx(pose[0], abs=1e-8)


def test_fixed_base_articulation_restores_authored_inertia_and_world_anchor(tmp_path):
    with World(fixture_config(tmp_path, fixed=True)) as world:
        manifest = export_world(world, tmp_path / 'export', backend='cpu', iterations=32)
        stage = Usd.Stage.Open(str(tmp_path / 'export' / 'field.usda'))
        root = stage.GetPrimAtPath(manifest['robot']['articulation_root'])
        assert root.IsA(UsdPhysics.FixedJoint)
        assert root.HasAPI(UsdPhysics.ArticulationRootAPI)
        assert root.GetRelationship('physics:body0').GetTargets() == [manifest['robot']['base_path']]
        row = next(x for x in manifest['actors'] if x['path'] == manifest['robot']['base_path'])
        assert row['mass_kg'] == 1
        assert row['position'] == pytest.approx([.2, -.3, .4], abs=1e-6)
        rotation = np.asarray(world._physics.getMatrixFromQuaternion(row['principal_axes'])).reshape(3, 3)
        tensor = rotation @ np.diag(row['inertia_kg_m2']) @ rotation.T
        source_rotation = np.asarray(world._physics.getMatrixFromQuaternion(
            world._physics.getQuaternionFromEuler([.1, -.2, .3]))).reshape(3, 3)
        assert tensor == pytest.approx(source_rotation @ np.diag([.02, .03, .04]) @ source_rotation.T)
        assert stage.GetPrimAtPath('/World/physicsScene').GetAttribute('physxScene:enableGPUDynamics').Get() is False


def test_world_adapter_is_per_instance_and_retains_cpu_behavior(tmp_path):
    class RecordingAdapter:
        def __init__(self, wrapped):
            self.wrapped, self.calls = wrapped, []

        def __getattr__(self, name):
            target = getattr(self.wrapped, name)
            if not callable(target):
                return target
            def call(*args, **kwargs):
                self.calls.append(name)
                return target(*args, **kwargs)
            return call

    config = fixture_config(tmp_path)
    with World(config) as first, World(config) as second:
        adapter = RecordingAdapter(first._physics)
        first._physics = adapter
        a, b = first.step(), second.step()
        assert a == b
        assert 'stepSimulation' in adapter.calls
        assert 'getBasePositionAndOrientation' in adapter.calls
        assert 'getJointState' in adapter.calls
        assert second._physics is adapter.wrapped
        assert a['physics']['backend'] == 'bullet'
        first.physics_status = {'backend': 'test'}
        assert first.snapshot()['physics'] == {'backend': 'test'}
        first.snapshot()['physics']['backend'] = 'mutated'
        assert first.physics_status == {'backend': 'test'}


def test_custom_rigid_primitive_uses_captured_inertial_frame(tmp_path):
    config = fixture_config(tmp_path)
    config['field_objects'] = [{'name': 'ball', 'shape': 'sphere', 'mass': .2,
                               'radius': .05, 'position': [.3, .4, .5]}]
    with World(config) as world:
        body = world._export_items[-1][0]
        wrapped = world._physics
        class UninitializedFrameAPI:
            def __getattr__(self, name):
                return getattr(wrapped, name)

            def getDynamicsInfo(self, query_body, index, **kwargs):
                result = list(wrapped.getDynamicsInfo(query_body, index, **kwargs))
                if query_body == body and index == -1:
                    result[3], result[4] = (math.nan,) * 3, (math.nan,) * 4
                return tuple(result)
        world._physics = UninitializedFrameAPI()
        manifest = export_world(world, tmp_path / 'export')
        actor = next(row for row in manifest['actors'] if row['body_id'] == body)
        assert actor['local_com'] == pytest.approx(world.base_inertial_frames[body][0])
        assert actor['principal_axes'] == pytest.approx(world.base_inertial_frames[body][1])
        assert actor['position'] == pytest.approx([.3, .4, .5], abs=1e-7)


@pytest.mark.parametrize('feature', ['mimic', 'safety_controller', 'calibration'])
def test_unsupported_joint_behavior_fails_before_writing_a_scene(tmp_path, feature):
    from pathlib import Path
    config = fixture_config(tmp_path)
    path = Path(config['robot_urdf'])
    text = path.read_text()
    text = text.replace('<axis xyz="0 1 0"/>', f'<axis xyz="0 1 0"/><{feature} joint="hinge" multiplier="-1"/>', 1)
    path.write_text(text)
    with World(config) as world:
        before = world.snapshot()
        with pytest.raises(ValueError, match=f'URDF {feature} behavior.*slide'):
            export_world(world, tmp_path / 'rejected')
        assert world.snapshot() == before
        assert not (tmp_path / 'rejected' / 'field.usda').exists()
