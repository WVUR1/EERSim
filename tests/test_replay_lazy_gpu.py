"""Descriptor-only replay coverage; all Bullet/VTK operations are mocked."""
from types import ModuleType, SimpleNamespace
import sys

import numpy as np
import pytest

from eersim import replay_rendering, render_backend, rendering
import test_replay_rendering as existing


class DescriptorBackend(existing.FakeBackend):
    def __init__(self, client, mode, *, base_inertial_frames, physics_api, visual_shapes=None):
        self.visual_shapes = visual_shapes
        super().__init__(client, mode, base_inertial_frames=base_inertial_frames, physics_api=physics_api)


@pytest.fixture
def lazy(monkeypatch):
    fake = existing.FakeBullet()
    monkeypatch.setenv("EERSIM_RENDERER", "auto")
    monkeypatch.setattr(replay_rendering, "bullet", fake)
    monkeypatch.setattr(replay_rendering, "RenderBackend", DescriptorBackend)
    monkeypatch.setattr(DescriptorBackend, "instances", [])
    monkeypatch.setattr(DescriptorBackend, "fail_close", False)
    return fake


def test_gpu_replay_uses_exact_descriptors_and_never_imports_or_resets_bullet(lazy, tmp_path):
    mesh = tmp_path / "owned.obj"
    mesh.write_text("# owned fixture\n")
    visuals = [existing.visual(lazy), existing.visual(lazy, row=1, kind=lazy.GEOM_MESH,
        dimensions=(-2., 3., .25), mesh_path=str(mesh), rgba=(.8, .6, .2, 1.))]
    source = existing.clip(lazy, visuals=visuals)
    replay = replay_rendering.ReplayRenderer(source)
    backend = DescriptorBackend.instances[-1]
    assert lazy.calls == [] and replay.client == backend.client == -1
    assert backend.visual_shapes == (
        (0, -1, lazy.GEOM_BOX, (2., 4., 6.), "", (.5, 0., 0.), (0., 0., 0., 1.), (.1, .2, .3, .4), -1),
        (1, -1, lazy.GEOM_MESH, (-2., 3., .25), str(mesh), (.5, 0., 0.), (0., 0., 0., 1.), (.8, .6, .2, 1.), -1))
    backend.pixels = np.ones((16, 20, 4), dtype=np.uint8)
    backend.status = {"backend": "opengl", "hardware_accelerated": True}
    later = source.scene.initial_poses.copy()
    later[1, :3] = (10, 11, 12)
    replay.set_frame(SimpleNamespace(poses=later))
    assert backend.provider.getBasePositionAndOrientation(1, physicsClientId=-1)[0] == (10, 11, 12)
    later[1, 0] = -999
    assert replay.render(20, 16) is backend.pixels
    assert lazy.calls == []
    assert replay.renderer_status["replay_load"]["cpu_visual_scene_created"] is False
    replay.close()
    replay.close()
    assert backend.closed and lazy.calls == [] and source.closed is False


def test_gpu_failure_builds_cpu_scene_once_at_latest_seek_and_preserves_camera(lazy):
    source = existing.clip(lazy, visuals=[existing.visual(lazy), existing.visual(lazy, row=1)])
    replay = replay_rendering.ReplayRenderer(source)
    backend = DescriptorBackend.instances[-1]
    backend.pixels = np.ones((16, 20, 4), dtype=np.uint8)
    replay.render(20, 16)
    assert lazy.calls == []
    later = source.scene.initial_poses.copy()
    later[0, :3], later[1, :3] = (1, 2, 3), (4, 5, 6)
    replay.set_frame(SimpleNamespace(poses=later))
    backend.pixels = None
    backend.status = {"backend": "cpu", "reason": "GPU fallback: lost context"}
    pixels = replay.render(20, 16, yaw=11., target=(1., 2., 3.))
    assert tuple(pixels[0, 0]) == (9, 20, 32, 255)
    assert lazy.bodies[700]["basePosition"] == (1., 2., 3.)
    assert lazy.bodies[701]["basePosition"] == (4., 5., 6.)
    assert replay.renderer_status["reason"] == "GPU fallback: lost context"
    assert replay.renderer_status["replay_load"]["cpu_visual_scene_created"] is True
    assert replay.renderer_status["replay_load"]["cpu_visual_construction_seconds"] >= 0
    replay.set_frame(SimpleNamespace(poses=source.scene.initial_poses))
    replay.render(20, 16)
    assert sum(name == "connect" for name, _ in lazy.calls) == 1
    assert lazy.bodies[700]["basePosition"] == tuple(source.scene.initial_poses[0, :3])
    replay.close()
    assert sum(name == "disconnect" for name, _ in lazy.calls) == 1


def test_texture_descriptor_signals_existing_gpu_limit_then_cpu_owns_texture(lazy, tmp_path):
    texture = tmp_path / "texture.png"
    texture.write_bytes(b"fixture")
    item = existing.visual(lazy, texture_path=str(texture))
    replay = replay_rendering.ReplayRenderer(existing.clip(lazy, visuals=[item, item]))
    backend = DescriptorBackend.instances[-1]
    assert all(shape[8] >= 0 for shape in backend.visual_shapes)
    assert not lazy.calls
    replay.render(20, 16)
    assert sum(name == "texture" for name, _ in lazy.calls) == 1
    assert sum(name == "material" for name, _ in lazy.calls) == 2
    replay.close()


def test_deferred_cpu_shape_failure_closes_backend_and_owned_client(lazy):
    replay = replay_rendering.ReplayRenderer(existing.clip(lazy))
    backend = DescriptorBackend.instances[-1]
    lazy.fail_shape = True
    with pytest.raises(RuntimeError, match="recorded replay visual"):
        replay.render(20, 16)
    assert backend.closed and not lazy.connected
    assert sum(name == "disconnect" for name, _ in lazy.calls) == 1
    replay.close()


@pytest.mark.parametrize("failure", ["unavailable", "initialization", "render"])
def test_real_backend_fallback_uses_private_cpu_scene_without_retrying_gpu(lazy, monkeypatch, failure):
    monkeypatch.setattr(replay_rendering, "RenderBackend", render_backend.RenderBackend)
    probes = []
    monkeypatch.setattr(render_backend, "probe_gpu", lambda: (probes.append(True) or
        {"available": failure != "unavailable", "hardware_accelerated": failure != "unavailable",
         "error": "mock driver unavailable"}))
    created = []
    class FailingHardware:
        def __init__(self, *args, **kwargs):
            created.append(self)
            self.closed = False
            if failure == "initialization":
                raise RuntimeError("mock setup failure")
        def render(self, *args):
            raise RuntimeError("mock draw failure")
        def close(self):
            self.closed = True
    monkeypatch.setattr(rendering, "HardwareRenderer", FailingHardware)
    replay = replay_rendering.ReplayRenderer(existing.clip(lazy))
    assert lazy.calls == []
    assert replay.render(20, 16).shape == (16, 20, 4)
    replay.render(20, 16)
    assert len(probes) == 1 and sum(name == "connect" for name, _ in lazy.calls) == 1
    assert replay.renderer_status["backend"] == "cpu"
    assert "GPU fallback:" in replay.renderer_status["reason"]
    if failure == "render":
        assert created[0].closed
    replay.close()


def test_render_backend_descriptor_mode_never_enumerates_bullet(monkeypatch):
    monkeypatch.delenv("EERSIM_RENDERER", raising=False)
    probes, created = [], []
    monkeypatch.setattr(render_backend, "probe_gpu", lambda: (probes.append(True) or
        {"available": True, "hardware_accelerated": True, "device": "test GPU"}))
    class NoBullet(ModuleType):
        def __getattr__(self, name):
            # Python and Qt import hooks inspect module metadata. Missing metadata
            # follows the normal module protocol; every Bullet API stays forbidden.
            if name.startswith("__") and name.endswith("__"):
                raise AttributeError(name)
            raise AssertionError("Descriptor render touched Bullet: " + name)
    blocked = NoBullet("pybullet")
    with pytest.raises(AssertionError, match="getNumBodies"):
        blocked.getNumBodies()
    monkeypatch.setitem(sys.modules, "pybullet", blocked)
    class Hardware:
        def __init__(self, client, bodies, **kwargs):
            created.append((client, bodies, kwargs))
            self.status = {"hardware": True, "renderer": "mock GPU"}
        def render(self, *args):
            return np.zeros((16, 20, 4), dtype=np.uint8)
        def close(self):
            pass
    monkeypatch.setattr(rendering, "HardwareRenderer", Hardware)
    shapes = ((5, -1, 3, (1, 1, 1), "", (0, 0, 0), (0, 0, 0, 1), (1, 1, 1, 1), -1),)
    backend = render_backend.RenderBackend(-1, "auto", base_inertial_frames={5: replay_rendering._IDENTITY},
        physics_api=object(), visual_shapes=shapes)
    pixels = backend.render(20, 16, 0, -45, 5, (0, 0, 0))
    assert pixels is not None, backend.status.get("reason")
    assert pixels.shape == (16, 20, 4)
    assert created[0][0:2] == (-1, [5]) and created[0][2]["visual_shapes"] == shapes
    assert len(probes) == 1
    backend.close()


def test_hardware_descriptor_loader_never_reads_bullet_scene(monkeypatch):
    calls = []
    class Node:
        def __getattr__(self, name):
            if name == "GetActiveCamera":
                return lambda: Node()
            return lambda *args: None
    vtk = ModuleType("vtk")
    for name in ("vtkRenderer", "vtkRenderWindow", "vtkWindowToImageFilter", "vtkLight"):
        setattr(vtk, name, Node)
    support = ModuleType("vtk.util.numpy_support")
    support.vtk_to_numpy = lambda value: value
    monkeypatch.setitem(sys.modules, "vtk", vtk)
    monkeypatch.setitem(sys.modules, "vtk.util", ModuleType("vtk.util"))
    monkeypatch.setitem(sys.modules, "vtk.util.numpy_support", support)
    class MathOnly:
        def getMatrixFromQuaternion(self, value):
            assert tuple(value) == (0., 0., 0., 1.)
            return np.eye(3).ravel()
        def __getattr__(self, name):
            raise AssertionError("Unexpected Bullet scene access: " + name)
    monkeypatch.setattr(rendering, "p", MathOnly())
    monkeypatch.setattr(rendering.HardwareRenderer, "_add_shape", lambda self, shape: calls.append(shape))
    shapes = ((8, -1, 3, (1, 2, 3), "", (0, 0, 0), (0, 0, 0, 1), (1, 1, 1, 1), -1),)
    hardware = rendering.HardwareRenderer(-1, [8], base_inertial_frames={8: replay_rendering._IDENTITY},
        physics_api=object(), visual_shapes=shapes)
    assert calls == list(shapes)
    np.testing.assert_array_equal(hardware._base_inertials[8], np.eye(4))
    hardware.close()
