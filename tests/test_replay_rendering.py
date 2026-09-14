"""Replay isolation checks; no Bullet connection, physics, GUI, or GL calls."""
from types import SimpleNamespace
import threading

import numpy as np
import pytest

from eersim import replay_rendering as rendering


class FakeBullet:
    DIRECT = 2
    GEOM_BOX, GEOM_SPHERE, GEOM_CYLINDER, GEOM_CAPSULE, GEOM_MESH = range(3, 8)
    ER_TINY_RENDERER, ER_NO_SEGMENTATION_MASK = 10, 11

    def __init__(self):
        self.calls, self.bodies, self.visuals = [], {}, {}
        self.connected, self.fail_shape, self.fail_reset = False, False, False

    def _record(self, name, kwargs):
        assert kwargs["physicsClientId"] == 91, "Touched an unrelated client"
        self.calls.append((name, kwargs))

    def connect(self, mode):
        assert mode == self.DIRECT
        self.connected = True
        self.calls.append(("connect", {}))
        return 91

    def createVisualShape(self, **kwargs):
        self._record("visual", kwargs)
        if self.fail_shape:
            return -1
        key = len(self.visuals) + 200
        self.visuals[key] = kwargs
        return key

    def setPhysicsEngineParameter(self, **kwargs):
        self._record("parameters", kwargs)
        assert kwargs == {"enableFileCaching": 0, "physicsClientId": 91}

    def createMultiBody(self, **kwargs):
        self._record("body", kwargs)
        assert kwargs["baseMass"] == 0 and kwargs["baseCollisionShapeIndex"] == -1
        key = len(self.bodies) + 700
        self.bodies[key] = kwargs
        return key

    def resetBasePositionAndOrientation(self, body, position, orientation, **kwargs):
        self._record("pose", dict(kwargs, body=body, position=position, orientation=orientation))
        assert body in self.bodies
        if self.fail_reset:
            raise RuntimeError("Owned visual reset failed")
        self.bodies[body].update(basePosition=position, baseOrientation=orientation)

    def loadTexture(self, path, **kwargs):
        self._record("texture", dict(kwargs, path=path))
        return 4321

    def changeVisualShape(self, body, link, **kwargs):
        assert body in self.bodies and link == -1
        self._record("material", dict(kwargs, body=body))

    def computeViewMatrixFromYawPitchRoll(self, *args):
        self.calls.append(("view", args))
        return (1.,) * 16

    def computeProjectionMatrixFOV(self, *args):
        self.calls.append(("projection", args))
        return (2.,) * 16

    def getCameraImage(self, width, height, view, projection, **kwargs):
        self._record("image", kwargs)
        pixels = np.full((height, width, 4), 80, dtype=np.uint8)
        depth = np.zeros((height, width))
        depth[0, 0] = 1.
        return width, height, pixels, depth

    def disconnect(self, **kwargs):
        self._record("disconnect", kwargs)
        self.connected = False

    def __getattr__(self, name):
        raise AssertionError(f"Unexpected Bullet operation: {name}")


class FakeBackend:
    instances = []
    fail_close = False

    def __init__(self, client, mode, *, base_inertial_frames, physics_api):
        self.client, self.mode = client, mode
        self.inertials, self.provider = base_inertial_frames, physics_api
        self.status = {"backend": "cpu", "reason": "GPU fallback: test driver unavailable"}
        self.calls, self.closed, self.pixels = [], False, None
        self.instances.append(self)

    def render(self, width, height, **camera):
        self.calls.append((width, height, camera))
        return self.pixels

    def close(self):
        self.closed = True
        if self.fail_close:
            raise RuntimeError("Test renderer close failed")


@pytest.fixture
def fake(monkeypatch):
    # Existing tests exercise the unchanged eager explicit-CPU path. GPU tests
    # below opt into descriptor mode without creating a real graphics context.
    monkeypatch.setenv("EERSIM_RENDERER", "cpu")
    bullet = FakeBullet()
    monkeypatch.setattr(rendering, "bullet", bullet)
    monkeypatch.setattr(rendering, "RenderBackend", FakeBackend)
    monkeypatch.setattr(FakeBackend, "instances", [])
    monkeypatch.setattr(FakeBackend, "fail_close", False)
    return bullet


def visual(fake, **changes):
    data = dict(row=0, kind=fake.GEOM_BOX, dimensions=(2., 4., 6.), mesh_path=None,
                local_position=(.5, 0., 0.), local_orientation=(0., 0., 0., 1.),
                rgba=(.1, .2, .3, .4), texture_path=None)
    data.update(changes)
    return SimpleNamespace(**data)


def clip(fake, *, visuals=None, poses=None):
    if poses is None:
        poses = np.array(((4., 5., 6., 0., 0., 0., 1.),
                          (7., 8., 9., 0., 0., 0., 1.)))
    scene = SimpleNamespace(row_keys=((12, -1), (12, 8)),
                            visuals=tuple(visuals or (visual(fake),)), initial_poses=poses)
    return SimpleNamespace(scene=scene, closed=False)


def test_private_visual_client_survives_live_world_close_and_updates_both_renderers(fake):
    source = clip(fake, visuals=[visual(fake), visual(fake, row=1), visual(fake, row=1)])
    renderer = rendering.ReplayRenderer(source)
    backend = FakeBackend.instances[-1]
    assert set(fake.bodies) == {700, 701, 702}
    assert renderer.client == backend.client == 91
    assert [kwargs for name, kwargs in fake.calls if name == "parameters"] == [
        {"enableFileCaching": 0, "physicsClientId": 91}]
    assert [name for name, _ in fake.calls].index("parameters") < [name for name, _ in fake.calls].index("visual")
    # The source world's IDs 12/8 never identify private bodies or links.
    assert all(body not in (12, 8) for body in backend.inertials)
    assert set(backend.inertials.values()) == {((0., 0., 0.), (0., 0., 0., 1.))}
    # A clip contains only data; disposing the unrelated live world cannot be read.
    del source.scene.initial_poses
    poses = np.array(((4., 5., 6., 0., 0., 0., 1.),
                      (11., 12., 13., 0., 0., 1., 0.)))
    renderer.set_frame(SimpleNamespace(poses=poses))
    updates = [kwargs for name, kwargs in fake.calls if name == "pose"]
    assert [item["body"] for item in updates] == [701, 702]
    np.testing.assert_array_equal(backend.provider.getBasePositionAndOrientation(701, physicsClientId=91)[0],
                                  (11., 12., 13.))
    assert fake.bodies[701]["basePosition"] == (11., 12., 13.)
    # Retain an independent pose array even if a non-clip caller mutates its frame.
    poses[1, :3] = -999
    assert backend.provider.getBasePositionAndOrientation(701, physicsClientId=91)[0] == (11., 12., 13.)
    with pytest.raises(ValueError, match="different Bullet client"):
        backend.provider.getBasePositionAndOrientation(701, physicsClientId=12)
    renderer.close()
    renderer.close()
    assert backend.closed and not fake.connected and not source.closed
    assert sum(name == "disconnect" for name, _ in fake.calls) == 1


def test_link_origin_visual_transform_not_original_com_offset(fake):
    # A 90-degree world Z rotation puts local X visual offset on world Y.
    half = np.sqrt(.5)
    poses = np.array(((1., 2., 3., 0., 0., half, half), (0., 0., 0., 0., 0., 0., 1.)))
    renderer = rendering.ReplayRenderer(clip(fake, poses=poses))
    body = fake.bodies[700]
    shape = fake.visuals[body["baseVisualShapeIndex"]]
    assert shape["halfExtents"] == (1., 2., 3.)
    assert body["baseInertialFramePosition"] == (0., 0., 0.)
    assert body["baseInertialFrameOrientation"] == (0., 0., 0., 1.)
    assert body["basePosition"] == (1., 2., 3.)
    assert body["baseOrientation"] == (0., 0., half, half)
    assert shape["visualFramePosition"] == (.5, 0., 0.)
    # The CPU body's transform and GPU provider must yield this same visual origin.
    world_rotation = np.array(((0., -1., 0.), (1., 0., 0.), (0., 0., 1.)))
    cpu_point = np.array(body["basePosition"]) + world_rotation @ shape["visualFramePosition"]
    gpu_position, _ = FakeBackend.instances[-1].provider.getBasePositionAndOrientation(700, physicsClientId=91)
    np.testing.assert_allclose(cpu_point, (1., 2.5, 3.))
    np.testing.assert_allclose(np.array(gpu_position) + world_rotation @ shape["visualFramePosition"], cpu_point)
    renderer.close()


def test_mesh_scale_material_and_texture_are_owned_by_private_client(fake, tmp_path):
    mesh, texture = tmp_path / "custom body.obj", tmp_path / "custom texture.png"
    mesh.write_text("# fixture asset, never opened by real Bullet\n")
    texture.write_bytes(b"fixture")
    item = visual(fake, kind=fake.GEOM_MESH, dimensions=(-2., 3., .25),
                  mesh_path=str(mesh), texture_path=str(texture))
    renderer = rendering.ReplayRenderer(clip(fake, visuals=[item, item]))
    assert fake.visuals[200]["meshScale"] == (-2., 3., .25)
    assert fake.visuals[200]["fileName"] == str(mesh)
    assert fake.visuals[200]["rgbaColor"] == item.rgba
    assert sum(name == "texture" for name, _ in fake.calls) == 1
    materials = [kwargs for name, kwargs in fake.calls if name == "material"]
    assert len(materials) == 2
    assert all(item["textureUniqueId"] == 4321 and item["physicsClientId"] == 91 for item in materials)
    renderer.close()


@pytest.mark.parametrize("kind,dimensions,expected", [
    (FakeBullet.GEOM_SPHERE, (.23, 0., 0.), {"radius": .23}),
    (FakeBullet.GEOM_CYLINDER, (.8, .15, 0.), {"length": .8, "radius": .15}),
    (FakeBullet.GEOM_CAPSULE, (0., .15, 0.), {"length": 0., "radius": .15}),
])
def test_primitive_dimension_conventions(fake, kind, dimensions, expected):
    renderer = rendering.ReplayRenderer(clip(fake, visuals=[visual(fake, kind=kind, dimensions=dimensions)]))
    for key, value in expected.items():
        assert fake.visuals[200][key] == value
    renderer.close()


@pytest.mark.parametrize("change,message", [
    ({"kind": 999}, "does not support visual geometry"),
    ({"row": 2}, "unknown pose row"),
    ({"dimensions": (1., 0., 1.)}, "positive full dimensions"),
    ({"local_orientation": (0., 0., 0., 0.)}, "unit quaternion"),
    ({"kind": FakeBullet.GEOM_MESH, "mesh_path": None}, "pinned asset"),
    ({"texture_path": "missing-replay-texture-for-test.png"}, "pinned asset"),
])
def test_unsupported_or_missing_visuals_fail_before_connect(fake, change, message):
    with pytest.raises(ValueError, match=message):
        rendering.ReplayRenderer(clip(fake, visuals=[visual(fake, **change)]))
    assert not fake.calls


@pytest.mark.parametrize("failure", ["shape", "reset", "close"])
def test_owned_client_is_disconnected_after_render_resource_failure(fake, failure, monkeypatch):
    if failure == "shape":
        fake.fail_shape = True
        with pytest.raises(RuntimeError, match="recorded replay visual"):
            rendering.ReplayRenderer(clip(fake))
    else:
        renderer = rendering.ReplayRenderer(clip(fake))
        if failure == "reset":
            fake.fail_reset = True
            poses = np.array(clip(fake).scene.initial_poses)
            poses[0, 0] += 1
            with pytest.raises(RuntimeError, match="reset failed"):
                renderer.set_frame(SimpleNamespace(poses=poses))
        else:
            monkeypatch.setattr(FakeBackend, "fail_close", True)
            with pytest.raises(RuntimeError, match="close failed"):
                renderer.close()
    assert not fake.connected
    assert sum(name == "disconnect" for name, _ in fake.calls) == 1


@pytest.mark.parametrize("bad", [np.zeros((1, 7)), np.full((2, 7), np.nan), np.zeros((2, 7))])
def test_bad_frame_is_rejected_before_any_pose_write(fake, bad):
    renderer = rendering.ReplayRenderer(clip(fake))
    before = len(fake.calls)
    with pytest.raises(ValueError, match="Replay pose"):
        renderer.set_frame(SimpleNamespace(poses=bad))
    assert len(fake.calls) == before and fake.connected
    renderer.close()


def test_camera_orbit_and_cpu_fallback_preserve_background_and_status(fake):
    renderer = rendering.ReplayRenderer(clip(fake))
    renderer.set_camera(yaw=71., pitch=-23., distance=2.5, target=(1., 2., 3.))
    pixels = renderer.render(20, 16)
    assert pixels.shape == (16, 20, 4)
    assert tuple(pixels[0, 0]) == (9, 20, 32, 255)
    assert tuple(pixels[1, 1]) == (80, 80, 80, 80)
    assert ("view", ((1., 2., 3.), 2.5, 71., -23., 0, 2)) in fake.calls
    assert renderer.renderer_status["reason"] == "GPU fallback: test driver unavailable"
    # A valid hardware frame uses the same camera and does not touch TinyRenderer.
    backend = FakeBackend.instances[-1]
    backend.pixels = np.ones((16, 20, 4), dtype=np.uint8)
    backend.status = {"backend": "opengl", "hardware_accelerated": True}
    before = len(fake.calls)
    assert renderer.render(20, 16, yaw=72.) is backend.pixels
    assert len(fake.calls) == before
    assert backend.calls[-1][2]["yaw"] == 72.
    assert renderer.renderer_status["hardware_accelerated"] is True
    renderer.close()


def test_invalid_camera_and_wrong_thread_do_not_mutate_or_render(fake):
    renderer = rendering.ReplayRenderer(clip(fake))
    with pytest.raises(ValueError, match="finite"):
        renderer.set_camera(yaw=45., pitch=float("nan"))
    assert renderer._camera["yaw"] == -45.
    with pytest.raises(ValueError, match="dimensions"):
        renderer.render(1, 20)
    errors = []

    def outside_owner():
        for action in (lambda: renderer.render(20, 16), renderer.close):
            try:
                action()
            except RuntimeError as error:
                errors.append(str(error))

    thread = threading.Thread(target=outside_owner)
    thread.start()
    thread.join(timeout=2)
    assert not thread.is_alive() and len(errors) == 2
    assert all("owner thread" in error for error in errors)
    assert fake.connected
    renderer.close()
    with pytest.raises(RuntimeError, match="closed"):
        renderer.render(20, 16)
