"""Pose capture, clocks, assets and bounds without physics or rendering."""
from pathlib import Path
from types import SimpleNamespace
import math

import numpy as np
import pytest

from eersim.replay import (ReplayCursor, ReplayRecorder, _Assets, _link_from_com,
                           _quat_product, _rotate)


class FakePhysics:
    VISUAL_SHAPE_DATA_TEXTURE_UNIQUE_IDS = 1
    GEOM_SPHERE, GEOM_BOX, GEOM_CYLINDER, GEOM_MESH, GEOM_CAPSULE = 2, 3, 4, 5, 7

    def __init__(self):
        self.bodies = [10, 20, 30]
        self.calls = []
        self.inertials = {body: ((0., 0., 0.), (0., 0., 0., 1.)) for body in self.bodies}
        self.poses = {(body, link): np.array([body / 10, link, .5, 0., 0., 0., 1.])
                      for body in self.bodies for link in ([-1, 0] if body != 30 else [-1])}
        self.shapes = {body: [(body, link, 3, (.1, .2, .3), b'', (.01, .02, .03),
                              (0., 0., 0., 1.), (.1, .2, .3, 1.), -1)
                             for b, link in self.poses if b == body] for body in self.bodies}

    def getNumBodies(self, **kwargs):
        return len(self.bodies)

    def getBodyUniqueId(self, index, **kwargs):
        return self.bodies[index]

    def getNumJoints(self, body, **kwargs):
        return int(body != 30)

    def getDynamicsInfo(self, body, link, **kwargs):
        return (1. if body == 10 or (body == 20 and link == 0) else 0., 0., 0., *self.inertials[body])

    def getVisualShapeData(self, body, **kwargs):
        return self.shapes[body]

    def getJointInfo(self, body, link, **kwargs):
        return (None,) * 12 + (b'arm',)

    def getBasePositionAndOrientation(self, body, **kwargs):
        self.calls.append(('base', body))
        value = self.poses[body, -1]
        return value[:3], value[3:]

    def getLinkState(self, body, link, **kwargs):
        self.calls.append(('link', body, link))
        assert kwargs.get('computeForwardKinematics') == 1
        value = self.poses[body, link]
        return (None,) * 4 + (value[:3], value[3:])

    def __getattr__(self, name):
        if name.startswith('_') or name in ('keys', 'actors'):
            raise AttributeError(name)
        raise AssertionError(f'Unexpected physics operation: {name}')


def world(hz=120, tick=0):
    api = FakePhysics()
    return SimpleNamespace(_physics=api, client=83, robot=10, dt=1 / hz, tick=tick,
                           config=SimpleNamespace(robot_urdf=''), game={'name': 'Test field'},
                           base_inertial_frames=api.inertials, _dynamic_field_bodies={20})


def advance(scene, tick):
    scene.tick = tick
    for key, pose in scene._physics.poses.items():
        if key[0] != 30:
            pose[0] = tick / 10 + key[0]


@pytest.mark.parametrize('hz', [120, 240, 125])
def test_capture_uses_source_ticks_preserves_initial_final_and_all_moving_links(tmp_path, hz):
    scene = world(hz, tick=7)
    recorder = ReplayRecorder.start(scene, mode='match_auto', duration=.4, cache_root=tmp_path,
                                    observed={'pose': [1., 2., 3., 4.]})
    initial = recorder.source.scene.initial_poses.copy()
    source_rows = recorder.source.rows
    assert recorder.source.moving == (0, 1, 2, 3)  # Includes fixed-base field child/root.
    expected_ticks = [7]
    last = 7 + round(hz * .4) + 1
    for tick in range(8, last + 1):
        advance(scene, tick)
        if recorder.capture(scene, phase='autonomous', mode_time=(tick - 7) / hz):
            expected_ticks.append(tick)
        # Repeated UI reads/paused wall time never append samples.
        assert not recorder.capture(scene, phase='autonomous', mode_time=(tick - 7) / hz)
    clip = recorder.finish(scene, phase='disabled', mode_time=.4, reason='completed')
    if expected_ticks[-1] != last:
        expected_ticks.append(last)
    assert clip.frame_count == len(expected_ticks)
    assert clip.duration == (last - 7) * scene.dt
    assert clip.scene.row_keys == source_rows
    np.testing.assert_array_equal(clip.frame_at(0).poses, initial)
    assert clip.frame_at(0).observed['pose'] == (1., 2., 3., 4.)
    for index, tick in enumerate(expected_ticks[1:], 1):
        frame = clip.frame_at(index)
        assert frame.tick == tick and frame.time_s == (tick - 7) * scene.dt
        assert frame.poses[1, 0] == tick / 10 + 10
        assert frame.poses[3, 0] == tick / 10 + 20
        np.testing.assert_array_equal(frame.poses[4], initial[4])
    assert clip.frame_at(clip.frame_count - 1).phase == 'disabled'
    clip.close()
    assert list(tmp_path.iterdir()) == []


def test_clip_survives_reset_and_frame_arrays_cannot_mutate_history(tmp_path):
    scene = world()
    recorder = ReplayRecorder.start(scene, mode='skills_auto', duration=1, cache_root=tmp_path)
    advance(scene, 8)
    observed = {'pose': [1., 2., 3., 4.], 'imu': {'heading': 10.}, 'diagnostics': ['not retained']}
    old_chunk = recorder.chunks[0]
    clip = recorder.finish(scene, phase='disabled', mode_time=8 / 120, observed=observed)
    before = clip.frame_at(1).poses.tobytes()
    old_chunk[:] = 9000
    scene._physics.poses.clear()
    scene._physics = FakePhysics()
    scene.client = 123
    observed['pose'][0] = 999
    frame = clip.frame_at(1)
    assert frame.poses.tobytes() == before and frame.observed['pose'][0] == 1
    assert 'diagnostics' not in frame.observed
    with pytest.raises(AttributeError):
        clip.mode = 'changed'
    with pytest.raises(ValueError):
        frame.poses.setflags(write=True)
    with pytest.raises(TypeError):
        frame.observed['pose'] = (0, 0, 0, 0)
    with pytest.raises(ValueError):
        clip.scene.initial_poses.setflags(write=True)
    clip.close()


def test_initialization_clock_is_physical_elapsed_time_and_seek_holds_preceding_sample(tmp_path):
    scene = world(tick=1)
    recorder = ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path)
    for tick in range(2, 14):
        advance(scene, tick)
        recorder.capture(scene, phase='initialize', mode_time=0)
    clip = recorder.finish(scene, phase='autonomous', mode_time=0)
    assert clip.duration == .1
    assert [clip.frame_at(i).tick for i in range(clip.frame_count)] == [1, 5, 9, 13]
    assert clip.index_at(.049) == 1 and clip.index_at(1 / 15) == 2
    assert clip.index_at(-1) == 0 and clip.index_at(100) == 3
    assert clip.frame_at(3).mode_time == 0
    clip.close()


def test_budget_changes_recording_stride_only_and_keeps_both_endpoints(tmp_path):
    scene = world()
    # Five initial rows, four moving rows; ten conservative frame slots.
    budget = 5 * 7 * 8 + 10 * (4 * 7 * 8 + 1024)
    recorder = ReplayRecorder.start(scene, mode='driver_auto', duration=90, cache_root=tmp_path,
                                    budget_bytes=budget)
    assert recorder.max_frames == 10 and recorder.effective_hz < 1
    for tick in range(1, 90 * 120 + 1):
        advance(scene, tick)
        recorder.capture(scene, phase='opcontrol', mode_time=tick / 120)
    clip = recorder.finish(scene, phase='disabled', mode_time=90)
    assert clip.frame_count <= 10 and clip.duration == 90
    assert clip.frame_at(0).tick == 0 and clip.frame_at(clip.frame_count - 1).tick == 90 * 120
    assert clip.bytes_used <= budget
    assert scene.dt == 1 / 120 and scene.tick == 90 * 120
    clip.close()


def test_empty_failure_budget_failure_and_live_replacement_cleanup_owned_assets(tmp_path):
    scene = world()
    with pytest.raises(ValueError, match='budget'):
        ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path, budget_bytes=1)
    assert not list(tmp_path.iterdir())
    recorder = ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path)
    assert recorder.finish(scene, phase='disabled', mode_time=0) is None
    assert not list(tmp_path.iterdir())
    recorder = ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path)
    scene._physics = FakePhysics()
    scene.tick = 5
    with pytest.raises(ValueError, match='changed'):
        recorder.capture(scene, phase='autonomous', mode_time=5 / 120)
    recorder.close()
    assert not list(tmp_path.iterdir())


def test_base_inertial_conversion_retains_world_link_geometry_without_float32_roundtrip():
    angle = math.pi / 3
    link_q = np.array([0., 0., math.sin(angle / 2), math.cos(angle / 2)])
    inertial_q = np.array([math.sin(.3), 0., 0., math.cos(.3)])
    link_position = np.array([1.123456789123, -.987654321987, .543219876543])
    inertial_position = np.array([.17, -.23, .08])
    com_position = link_position + _rotate(link_q, inertial_position)
    com_q = _quat_product(link_q, inertial_q)
    actual = _link_from_com(com_position, com_q, inertial_position, inertial_q)
    np.testing.assert_allclose(actual, [*link_position, *link_q], rtol=0, atol=3e-16)
    assert abs(actual[0] - np.float32(actual[0])) > 1e-9


def test_physx_capture_reads_existing_snapshot_rows_without_pose_rpc_or_noise(tmp_path):
    scene = world()
    api = scene._physics
    keys = tuple(api.poses)
    api.keys = {key: row for row, key in enumerate(keys) if key != (30, -1)}
    api.actors = {key: {'position': pose[:3].tolist(), 'orientation': pose[3:].tolist()}
                  for key, pose in api.poses.items()}
    api._state = {'poses': np.asarray([api.poses[key] for key in api.keys], np.float32)}
    api._actor_pose = lambda *args: None
    recorder = ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path)
    assert api.calls == []
    scene.tick = 1
    api._state['poses'][2, :3] = [4.1234567, .2, .8]
    original = api._state['poses'].copy()
    clip = recorder.finish(scene, phase='disabled', mode_time=1 / 120)
    assert clip.frame_at(1).poses[2].tobytes() == original[2].astype(np.float64).tobytes()
    np.testing.assert_array_equal(api._state['poses'], original)
    assert api.calls == []
    clip.close()


def test_pinned_obj_material_texture_tree_is_portable_and_survives_source_changes(tmp_path):
    source = tmp_path / 'source with spaces'
    source.mkdir()
    texture = source / 'surface color.png'
    texture.write_bytes(b'fake PNG bytes')
    material = source / 'paint.mtl'
    material.write_text('newmtl paint\nmap_Kd -s 1 1 1 "surface color.png"\n')
    mesh = source / 'robot.obj'
    mesh.write_text('mtllib paint.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
    assets = _Assets(tmp_path / 'cache')
    pinned = assets.pin(mesh)
    assert pinned == assets.pin(mesh) and len(assets.files) == 3
    mtl = (pinned.parent / pinned.read_text().splitlines()[0].split(maxsplit=1)[1]).resolve()
    image = (mtl.parent / mtl.read_text().splitlines()[1].split()[-1]).resolve()
    assert assets.path in image.parents and image.read_bytes() == b'fake PNG bytes'
    texture.write_bytes(b'changed later')
    assert image.read_bytes() == b'fake PNG bytes'
    assets.close()
    assert texture.read_bytes() == b'changed later' and not pinned.exists()


def test_urdf_texture_resolves_global_material_and_never_reuses_live_texture_id(tmp_path):
    image = tmp_path / 'skin.png'
    image.write_bytes(b'fake texture')
    urdf = tmp_path / 'robot.urdf'
    urdf.write_text('''<robot name="r"><material name="skin"><texture filename="skin.png"/></material>
      <link name="base"><visual><material name="skin"/><geometry><box size=".1 .2 .3"/></geometry></visual></link>
      <link name="arm"><visual><geometry><box size=".1 .2 .3"/></geometry></visual></link>
      <joint name="j" type="fixed"><parent link="base"/><child link="arm"/></joint></robot>''')
    scene = world()
    scene.config.robot_urdf = str(urdf)
    shape = list(scene._physics.shapes[10][0])
    shape[8] = 742
    scene._physics.shapes[10][0] = tuple(shape)
    recorder = ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path / 'cache')
    descriptor = recorder.source.scene.visuals[0]
    assert descriptor.texture_path and Path(descriptor.texture_path).read_bytes() == image.read_bytes()
    assert not hasattr(descriptor, 'texture_id')
    recorder.close()
    scene.config.robot_urdf = ''
    with pytest.raises(ValueError, match='texture'):
        ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path / 'cache')


def test_pure_cursor_pause_seek_speed_and_end_restart():
    cursor = ReplayCursor(2.)
    assert cursor.advance(10) == 0
    cursor.play()
    assert cursor.advance(.2) == .2
    cursor.pause()
    assert cursor.advance(100) == .2
    assert cursor.seek(1.5) == 1.5 and not cursor.playing
    cursor.play()
    assert cursor.advance(1, speed=2) == 2 and not cursor.playing
    cursor.play()
    assert cursor.position_s == 0 and cursor.playing
    with pytest.raises(ValueError):
        cursor.seek(float('nan'))
    with pytest.raises(ValueError):
        cursor.advance(-1)


def test_collada_images_pin_and_missing_dependency_cleans_failed_capture(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    texture = source / 'image.png'
    texture.write_bytes(b'image')
    mesh = source / 'visual.dae'
    mesh.write_text('''<COLLADA xmlns="urn:collada"><library_images><image id="img">
      <init_from>image.png</init_from></image></library_images></COLLADA>''')
    assets = _Assets(tmp_path / 'cache')
    pinned = assets.pin(mesh)
    assert b'../' in pinned.read_bytes() and len(assets.files) == 2
    assets.close()
    mesh.write_text('''<COLLADA><library_images><image><init_from>missing.png</init_from>
      </image></library_images></COLLADA>''')
    scene = world()
    shape = list(scene._physics.shapes[10][0])
    shape[2], shape[4] = 5, str(mesh).encode()
    scene._physics.shapes[10][0] = tuple(shape)
    with pytest.raises(ValueError, match='missing'):
        ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path / 'cache')
    assert not list((tmp_path / 'cache').iterdir())


@pytest.mark.parametrize('kind,filename', [(6, b''), (9, b''), (5, b'')])
def test_unsupported_or_in_memory_visuals_fail_before_recording(tmp_path, kind, filename):
    scene = world()
    shape = list(scene._physics.shapes[10][0])
    shape[2], shape[4] = kind, filename
    scene._physics.shapes[10][0] = tuple(shape)
    with pytest.raises(ValueError, match='Replay'):
        ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path)
    assert not list(tmp_path.iterdir())


def test_relative_custom_mesh_pins_before_urdf_replacement_and_capture_deadlines_are_read_only(tmp_path):
    source = tmp_path / 'custom robot'
    (source / 'meshes').mkdir(parents=True)
    mesh = source / 'meshes/part.obj'
    mesh.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
    urdf = source / 'robot.urdf'
    urdf.write_text('<robot name="custom"><link name="base"/></robot>')
    scene = world()
    scene.config.robot_urdf = str(urdf)
    scene._dynamic_field_bodies.clear()  # Loaded child masses remain authoritative.
    shape = list(scene._physics.shapes[10][0])
    shape[2], shape[4] = 5, b'meshes/part.obj'
    scene._physics.shapes[10][0] = tuple(shape)
    recorder = ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path / 'cache')
    assert (20, 0) in [recorder.source.rows[i] for i in recorder.source.moving]
    scene._physics.calls.clear()
    advance(scene, 1)
    assert not recorder.capture(scene, phase='autonomous', mode_time=1 / 120)
    assert not scene._physics.calls
    clip = recorder.finish(scene, phase='disabled', mode_time=1 / 120)
    pinned = Path(clip.scene.visuals[0].mesh_path)
    assert pinned.read_text() == mesh.read_text()
    urdf.unlink()
    mesh.write_text('replaced for the next experiment')
    assert pinned.read_text().startswith('v 0 0 0')
    assert clip.frame_at(1).tick == 1
    clip.close()


def test_clip_close_releases_poses_on_asset_failure_and_allows_cleanup_retry(tmp_path, monkeypatch):
    scene = world()
    recorder = ReplayRecorder.start(scene, mode='match_auto', duration=1, cache_root=tmp_path)
    advance(scene, 1)
    clip = recorder.finish(scene, phase='disabled', mode_time=1 / 120)
    assets = clip.scene._assets
    close_assets = assets.close
    calls = []

    def temporarily_locked():
        calls.append(True)
        if len(calls) == 1:
            raise PermissionError('temporary mesh lock')
        close_assets()

    monkeypatch.setattr(assets, 'close', temporarily_locked)
    assert clip._chunks and not clip.closed
    with pytest.raises(PermissionError, match='temporary'):
        clip.close()
    assert clip.closed and clip._chunks == ()
    with pytest.raises(RuntimeError, match='closed'):
        clip.frame_at(0)
    assert assets.path.is_dir() and not assets.closed
    clip.close()
    assert assets.closed and not assets.path.exists()
    clip.close()
    assert clip._chunks == () and clip.closed
