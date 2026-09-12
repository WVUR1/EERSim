"""Live render transforms follow the selected solver, without a GUI or GL frame."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('pybullet')
pytest.importorskip('vtk')
import pybullet as p

from eersim.rendering import HardwareRenderer, _matrix
from eersim.render_backend import RenderBackend


def test_live_base_and_link_poses_override_static_bullet_shadow(tmp_path):
    urdf = tmp_path / 'two_link.urdf'
    urdf.write_text('''<robot name="pose_provider">
      <link name="base"><inertial><origin xyz=".1 -.03 .07" rpy=".2 .3 -.1"/>
        <mass value="1"/><inertia ixx=".1" iyy=".2" izz=".3" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><origin xyz=".02 .04 .06" rpy=".1 -.2 .3"/><geometry><box size=".1 .2 .3"/></geometry></visual></link>
      <link name="arm"><inertial><mass value=".2"/><inertia ixx=".01" iyy=".02" izz=".03" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><origin xyz="-.01 .03 .02" rpy="-.2 .1 .4"/><geometry><box size=".2 .1 .1"/></geometry></visual></link>
      <joint name="spin" type="continuous"><parent link="base"/><child link="arm"/>
        <origin xyz="0 0 .3"/><axis xyz="0 1 0"/><limit effort="2" velocity="20"/></joint></robot>''')
    client = p.connect(p.DIRECT)
    renderer = None
    try:
        body = p.loadURDF(str(urdf), flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=client)
        original_base = p.getBasePositionAndOrientation(body, physicsClientId=client)
        original_joint = p.getJointState(body, 0, physicsClientId=client)
        inertia = p.getDynamicsInfo(body, -1, physicsClientId=client)
        base_link = ([2., -1., .8], p.getQuaternionFromEuler([.4, -.2, .7]))
        base_com = p.multiplyTransforms(*base_link, inertia[3], inertia[4])
        arm_link = ([-.5, 1.2, 2.3], p.getQuaternionFromEuler([-.5, .3, -.8]))
        calls = []
        class Provider:
            def getBasePositionAndOrientation(self, requested, **kwargs):
                assert requested == body
                calls.append(('base', requested))
                return base_com

            def getLinkState(self, requested, index, **kwargs):
                assert (requested, index) == (body, 0)
                calls.append(('link', requested, index))
                return (None, None, None, None, *arm_link)
        provider = Provider()
        renderer = HardwareRenderer(client, [body], physics_api=provider)
        renderer._update_poses()
        assert calls == [('base', body), ('link', body, 0)]
        for visual in renderer._visuals:
            matrix = visual.actor.GetMatrix()
            actual = np.array([[matrix.GetElement(i, j) for j in range(4)] for i in range(4)])
            expected = _matrix(*(base_link if visual.link == -1 else arm_link)) @ visual.local
            np.testing.assert_allclose(actual, expected, atol=3e-7)
        # Reading/rendering the live authority never teleports or advances the
        # stored geometry scene; its state is intentionally stale in this test.
        assert p.getBasePositionAndOrientation(body, physicsClientId=client) == original_base
        assert p.getJointState(body, 0, physicsClientId=client) == original_joint
    finally:
        if renderer is not None:
            renderer.close()
        p.disconnect(client)


def test_backend_passes_provider_to_renderer_without_a_graphics_context(monkeypatch):
    from eersim import rendering, render_backend
    monkeypatch.delenv('EERSIM_RENDERER', raising=False)
    monkeypatch.setattr(render_backend, 'probe_gpu', lambda: {'available': True, 'hardware_accelerated': True})
    provider = object()
    received = []
    class Renderer:
        def __init__(self, client, bodies, **kwargs):
            received.append(kwargs['physics_api'])
            self.status = {'hardware': True, 'renderer': 'fixture'}

        def render(self, width, height, *args):
            return np.zeros((height, width, 4), dtype=np.uint8)

        def close(self):
            pass
    monkeypatch.setattr(rendering, 'HardwareRenderer', Renderer)
    client = p.connect(p.DIRECT)
    try:
        backend = RenderBackend(client, 'gpu', physics_api=provider)
        assert backend.render(32, 16, 0, 0, 1, [0, 0, 0]).shape == (16, 32, 4)
        assert received == [provider]
        backend.close()
    finally:
        p.disconnect(client)
