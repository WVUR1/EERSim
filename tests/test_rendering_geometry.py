"""GPU scene geometry agrees with real Bullet URDF frames without rendering.

No OpenGL frame or graphics probe is needed: VTK actor matrices and polydata
can be inspected before a render window creates its graphics context.
"""
from __future__ import annotations

import math

import eersim  # initializes native DLL paths for unactivated portable environments
import numpy as np
import pytest
import pybullet as p

pytest.importorskip("vtk")
from eersim.rendering import HardwareRenderer


def transform(position, rpy):
    """Independent URDF fixed-axis rotation calculation."""
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    result = np.eye(4)
    result[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                      [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                      [-sp, cp * sr, cp * cr]]
    result[:3, 3] = position
    return result


def actor_matrix(actor):
    matrix = actor.GetMatrix()
    return np.array([[matrix.GetElement(row, column) for column in range(4)] for row in range(4)])


@pytest.fixture
def bullet_client():
    client = p.connect(p.DIRECT)
    try:
        yield client
    finally:
        p.disconnect(client)


def test_offset_inertia_rotated_joint_visual_origin_and_nonuniform_mesh_scale(tmp_path, bullet_client):
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3]], dtype=float)
    (tmp_path / "tetra.obj").write_text(
        "v 0 0 0\nv 1 0 0\nv 0 2 0\nv 0 0 3\n"
        "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n")
    urdf = tmp_path / "offset_frames.urdf"
    urdf.write_text('''<robot name="offset_frames">
      <link name="base">
        <inertial><origin xyz="-.09 .13 .17" rpy="-.3 .2 .5"/><mass value="2"/>
          <inertia ixx=".12" iyy=".15" izz=".18" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><origin xyz=".18 -.11 .07" rpy=".4 -.25 .6"/>
          <geometry><box size=".2 .3 .4"/></geometry>
          <material name="base_tint"><color rgba=".1 .2 .3 .4"/></material></visual>
      </link>
      <link name="arm">
        <inertial><origin xyz=".07 -.08 .1" rpy="-.6 .2 .15"/><mass value="1"/>
          <inertia ixx=".05" iyy=".06" izz=".07" ixy="0" ixz="0" iyz="0"/></inertial>
        <visual><origin xyz=".04 -.05 .06" rpy=".2 -.4 .3"/>
          <geometry><mesh filename="tetra.obj" scale=".12 .05 .03"/></geometry>
          <material name="arm_tint"><color rgba=".7 .3 .15 .55"/></material></visual>
      </link>
      <joint name="arm_spin" type="continuous"><parent link="base"/><child link="arm"/>
        <origin xyz=".32 -.22 .24" rpy=".15 .55 -.2"/><axis xyz="0 1 0"/>
        <limit effort="10" velocity="5"/></joint>
    </robot>''')
    position, rpy = [1.2, -.7, .8], [.4, -.3, 1.0]
    body = p.loadURDF(str(urdf), position, p.getQuaternionFromEuler(rpy),
                      flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=bullet_client)
    renderer = HardwareRenderer(bullet_client, [body])
    try:
        base = transform(position, rpy)
        expected_base_visual = base @ transform([.18, -.11, .07], [.4, -.25, .6])
        scaling = np.diag([.12, .05, .03, 1.])
        for angle in [-.7, 0, .65]:
            p.resetJointState(body, 0, angle, physicsClientId=bullet_client)
            renderer._update_poses()
            by_link = {visual.link: visual for visual in renderer._visuals}
            np.testing.assert_allclose(actor_matrix(by_link[-1].actor), expected_base_visual, atol=2e-7)
            expected_arm = (base @ transform([.32, -.22, .24], [.15, .55, -.2])
                            @ transform([0, 0, 0], [0, angle, 0])
                            @ transform([.04, -.05, .06], [.2, -.4, .3]) @ scaling)
            actual_arm = actor_matrix(by_link[0].actor)
            np.testing.assert_allclose(actual_arm, expected_arm, atol=2e-7)
            # Probe noncentral mesh points: this detects incorrect scale order
            # and scaling the visual-frame translation along with its mesh.
            homogeneous = np.column_stack([vertices, np.ones(len(vertices))])
            np.testing.assert_allclose(homogeneous @ actual_arm.T,
                                       homogeneous @ expected_arm.T, atol=2e-7)
        for visual, shape in zip(renderer._visuals, p.getVisualShapeData(body, physicsClientId=bullet_client)):
            prop = visual.actor.GetProperty()
            np.testing.assert_allclose(prop.GetColor(), shape[7][:3])
            assert prop.GetOpacity() == pytest.approx(shape[7][3])
    finally:
        renderer.close()


@pytest.mark.parametrize("kind", [p.GEOM_CYLINDER, p.GEOM_CAPSULE])
def test_bullet_cylinder_and_capsule_use_z_axis_without_interior_caps(bullet_client, kind):
    radius, length = .2, .6
    visual = p.createVisualShape(kind, radius=radius, length=length,
                                 rgbaColor=[.1, .3, .6, .4], physicsClientId=bullet_client)
    body = p.createMultiBody(baseVisualShapeIndex=visual, physicsClientId=bullet_client)
    shape = p.getVisualShapeData(body, physicsClientId=bullet_client)[0]
    assert shape[3][:2] == pytest.approx([length, radius])
    renderer = HardwareRenderer(bullet_client, [body])
    try:
        geometry = next(iter(renderer._geometry.values()))
        bounds = geometry.GetBounds()
        half_height = length / 2 + (radius if kind == p.GEOM_CAPSULE else 0)
        np.testing.assert_allclose(bounds, [-radius, radius, -radius, radius, -half_height, half_height], atol=1e-6)
        if kind == p.GEOM_CAPSULE:
            for index in range(geometry.GetNumberOfCells()):
                cell = geometry.GetCell(index)
                z = [cell.GetPoints().GetPoint(i)[2] for i in range(cell.GetNumberOfPoints())]
                # Hemispheres join an open cylinder; there must be no hidden
                # discs that become visible through a transparent capsule.
                assert not (len(z) >= 3 and max(z) - min(z) < 1e-8
                            and abs(abs(z[0]) - length / 2) < 1e-8)
    finally:
        renderer.close()


def test_multimaterial_obj_uses_bullets_resolved_urdf_tint(tmp_path, bullet_client):
    (tmp_path / "palette.mtl").write_text("newmtl red\nKd 1 0 0\nnewmtl green\nKd 0 1 0\n")
    (tmp_path / "palette.obj").write_text(
        "mtllib palette.mtl\nv -.8 -.4 0\nv -.1 -.4 0\nv -.1 .4 0\nv -.8 .4 0\n"
        "v .1 -.4 0\nv .8 -.4 0\nv .8 .4 0\nv .1 .4 0\n"
        "usemtl red\nf 1 2 3\nf 1 3 4\nusemtl green\nf 5 6 7\nf 5 7 8\n")
    urdf = tmp_path / "palette.urdf"
    urdf.write_text('''<robot name="palette"><link name="base">
      <inertial><mass value="1"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial>
      <visual><geometry><mesh filename="palette.obj"/></geometry>
        <material name="tint"><color rgba=".2 .3 .8 .5"/></material></visual>
    </link></robot>''')
    body = p.loadURDF(str(urdf), useFixedBase=True, physicsClientId=bullet_client)
    view = p.computeViewMatrix([0, 0, 1.8], [0, 0, 0], [0, 1, 0])
    projection = p.computeProjectionMatrixFOV(60, 1, .01, 10)
    result = p.getCameraImage(64, 64, view, projection, lightAmbientCoeff=1,
                              lightDiffuseCoeff=0, lightSpecularCoeff=0,
                              renderer=p.ER_TINY_RENDERER, physicsClientId=bullet_client)
    pixels, labels = np.asarray(result[2]), np.asarray(result[4])
    cpu_colors = np.unique(pixels[labels == body, :3], axis=0)
    assert cpu_colors.shape == (1, 3)
    np.testing.assert_allclose(cpu_colors[0], [51, 76, 204], atol=1)
    renderer = HardwareRenderer(bullet_client, [body])
    try:
        actor = renderer._visuals[0].actor
        assert not actor.GetMapper().GetScalarVisibility()
        np.testing.assert_allclose(np.array(actor.GetProperty().GetColor()) * 255,
                                   cpu_colors[0], atol=1)
        assert actor.GetProperty().GetOpacity() == pytest.approx(.5)
    finally:
        renderer.close()


def test_maximal_body_uses_captured_inertia_and_matches_cpu_silhouette(tmp_path, bullet_client, monkeypatch):
    """Rigid-body dynamics metadata has uninitialized frame fields in Bullet.

    Recover the actual inertial frame at load time and preserve it across the
    renderer boundary. An offset, rotated inertia and global scale ensure an
    identity fallback cannot accidentally pass this regression.
    """
    urdf = tmp_path / "rigid_offset.urdf"
    urdf.write_text('''<robot name="rigid_offset"><link name="base">
      <inertial><origin xyz=".11 -.07 .19" rpy=".2 -.35 .45"/><mass value="1"/>
        <inertia ixx=".04" iyy=".05" izz=".06" ixy="0" ixz="0" iyz="0"/></inertial>
      <visual><origin xyz="-.13 .05 .09" rpy="-.3 .6 -.2"/>
        <geometry><box size=".34 .21 .14"/></geometry>
        <material name="tint"><color rgba=".2 .6 .8 1"/></material></visual>
    </link></robot>''')
    position, rpy, scale = [.2, -.1, .35], [.15, -.25, .45], 1.7
    orientation = p.getQuaternionFromEuler(rpy)
    body = p.loadURDF(str(urdf), position, orientation, globalScaling=scale,
                      useMaximalCoordinates=True,
                      flags=p.URDF_USE_INERTIA_FROM_FILE | p.URDF_ENABLE_SLEEPING,
                      physicsClientId=bullet_client)
    captured = p.multiplyTransforms(*p.invertTransform(position, orientation),
                                    *p.getBasePositionAndOrientation(body, physicsClientId=bullet_client))
    np.testing.assert_allclose(captured[0], np.array([.11, -.07, .19]) * scale, atol=1e-7)
    original_dynamics = p.getDynamicsInfo

    def stale_inertial_fields(*args, **kwargs):
        data = list(original_dynamics(*args, **kwargs))
        assert data[10] == 1  # actual maximal-coordinate rigid body
        data[3], data[4] = (123., 456., 789.), (0., 1., float("nan"), 1.)
        return tuple(data)

    monkeypatch.setattr(p, "getDynamicsInfo", stale_inertial_fields)
    with pytest.raises(ValueError, match="captured base inertial frame"):
        HardwareRenderer(bullet_client, [body])
    renderer = HardwareRenderer(bullet_client, [body], base_inertial_frames={body: captured})
    try:
        renderer._update_poses()
        actor = renderer._visuals[0].actor
        expected = transform(position, rpy) @ transform(np.array([-.13, .05, .09]) * scale, [-.3, .6, -.2])
        matrix = actor_matrix(actor)
        np.testing.assert_allclose(matrix, expected, atol=2e-7)
        assert np.isfinite(matrix).all()

        width, height = 160, 120
        view = p.computeViewMatrixFromYawPitchRoll(position, 1.8, 45, -35, 0, 2)
        projection = p.computeProjectionMatrixFOV(60, width / height, .01, 10)
        result = p.getCameraImage(width, height, view, projection,
                                  renderer=p.ER_TINY_RENDERER, physicsClientId=bullet_client)
        rows, columns = np.nonzero(np.asarray(result[4]) == body)
        assert len(rows) > 100
        geometry = actor.GetMapper().GetInput()
        points = np.array([(*geometry.GetPoint(i), 1.) for i in range(geometry.GetNumberOfPoints())])
        camera = np.array(projection).reshape(4, 4, order="F") @ np.array(view).reshape(4, 4, order="F")
        clip = points @ matrix.T @ camera.T
        ndc = clip[:, :2] / clip[:, 3:4]
        projected = np.column_stack([(ndc[:, 0] + 1) * width / 2, (1 - ndc[:, 1]) * height / 2])
        # Pixel coverage differs by at most a couple of rasterized edge pixels;
        # a missing/garbage inertial transform moves the object substantially.
        np.testing.assert_allclose([columns.min(), columns.max(), rows.min(), rows.max()],
                                   [projected[:, 0].min(), projected[:, 0].max(),
                                    projected[:, 1].min(), projected[:, 1].max()], atol=2)
    finally:
        renderer.close()
