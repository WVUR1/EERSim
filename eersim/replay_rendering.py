"""Recorded-pose rendering in an owned, collision-free Bullet client.

The clip supplies world link-origin poses and link-local visuals. The private
scene uses identity inertial frames, so no live-world COM transform is applied
again. No simulation step or collision shape is created by this module.
"""
from __future__ import annotations

from pathlib import Path
import threading

import numpy as np
import pybullet as bullet

from .render_backend import RenderBackend


_IDENTITY = ((0., 0., 0.), (0., 0., 0., 1.))


def _vector(values, size, label):
    result = tuple(float(value) for value in values)
    if len(result) != size or not np.isfinite(result).all():
        raise ValueError(f"Replay {label} requires {size} finite values")
    return result


def _quaternion(values, label):
    result = _vector(values, 4, label)
    if not np.isclose(np.linalg.norm(result), 1., rtol=0., atol=1e-5):
        raise ValueError(f"Replay {label} must be a unit quaternion")
    return result


def _poses(values, count):
    result = np.array(values, dtype=np.float64, copy=True)
    if result.shape != (count, 7) or not np.isfinite(result).all():
        raise ValueError(f"Replay poses must be a finite ({count}, 7) array")
    if not np.allclose(np.linalg.norm(result[:, 3:], axis=1), 1., rtol=0., atol=1e-5):
        raise ValueError("Replay pose orientations must be unit quaternions")
    result.setflags(write=False)
    return result


class _RecordedPoses:
    """The hardware renderer reads only this private scene's recorded poses."""
    def __init__(self, client, body_rows, poses):
        self.client, self.body_rows, self.poses = client, body_rows, poses

    def getBasePositionAndOrientation(self, body, *, physicsClientId):
        if physicsClientId != self.client:
            raise ValueError("Replay pose query used a different Bullet client")
        row = self.poses[self.body_rows[body]]
        return tuple(row[:3]), tuple(row[3:])


class ReplayRenderer:
    """Render a clip independently of its original world, on one owner thread.

    The caller owns the clip and closes this renderer before disposing the clip.
    ``set_frame`` accepts a ReplayFrame; ``render`` accepts the same orbit camera
    parameters as World.render. GPU setup is deferred until the first render,
    using the existing visible TinyRenderer fallback on unsupported graphics.
    """
    def __init__(self, clip, *, renderer="auto"):
        self._owner = threading.get_ident()
        self._closed = False
        self._clip = clip
        self.client = -1
        self._backend = None
        self._body_rows = {}
        self._camera = {"yaw": -45., "pitch": -55., "distance": 5., "target": (0., 0., 0.)}
        scene = clip.scene
        self._row_count = len(scene.row_keys)
        poses = _poses(scene.initial_poses, self._row_count)
        # Validate descriptors before creating any Bullet resources.
        shapes = [self._shape_arguments(visual) for visual in scene.visuals]
        try:
            self.client = bullet.connect(bullet.DIRECT)
            if self.client < 0:
                raise RuntimeError("Could not create the private replay renderer client")
            # Pinned assets belong to this clip, not a process-wide file cache
            # that can outlive many discarded recordings.
            bullet.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=self.client)
            textures = {}
            for visual, arguments in zip(scene.visuals, shapes):
                shape = bullet.createVisualShape(**arguments, physicsClientId=self.client)
                if shape < 0:
                    raise RuntimeError("Bullet could not load a recorded replay visual")
                row = poses[visual.row]
                body = bullet.createMultiBody(
                    baseMass=0., baseCollisionShapeIndex=-1, baseVisualShapeIndex=shape,
                    basePosition=tuple(row[:3]), baseOrientation=tuple(row[3:]),
                    baseInertialFramePosition=_IDENTITY[0], baseInertialFrameOrientation=_IDENTITY[1],
                    physicsClientId=self.client)
                if body < 0:
                    raise RuntimeError("Bullet could not create a private replay visual body")
                self._body_rows[body] = visual.row
                if visual.texture_path:
                    path = str(Path(visual.texture_path))
                    if path not in textures:
                        texture = bullet.loadTexture(path, physicsClientId=self.client)
                        if texture < 0:
                            raise RuntimeError("Bullet could not load a recorded replay texture")
                        textures[path] = texture
                    bullet.changeVisualShape(body, -1, textureUniqueId=textures[path],
                                             rgbaColor=tuple(visual.rgba), physicsClientId=self.client)
            self._pose_provider = _RecordedPoses(self.client, self._body_rows, poses)
            self._backend = RenderBackend(
                self.client, renderer,
                base_inertial_frames={body: _IDENTITY for body in self._body_rows},
                physics_api=self._pose_provider)
            self.renderer_status = dict(self._backend.status)
        except BaseException:
            self.close()
            raise

    def _shape_arguments(self, visual):
        if not isinstance(visual.row, (int, np.integer)) or not 0 <= visual.row < self._row_count:
            raise ValueError("Replay visual refers to an unknown pose row")
        dimensions = tuple(float(value) for value in visual.dimensions)
        if not np.isfinite(dimensions).all():
            raise ValueError("Replay visual dimensions must be finite")
        result = {
            "shapeType": visual.kind,
            "visualFramePosition": _vector(visual.local_position, 3, "visual position"),
            "visualFrameOrientation": _quaternion(visual.local_orientation, "visual orientation"),
            "rgbaColor": _vector(visual.rgba, 4, "visual color"),
        }
        if visual.kind == bullet.GEOM_BOX:
            if len(dimensions) != 3 or min(dimensions) <= 0:
                raise ValueError("Replay boxes require three positive full dimensions")
            result["halfExtents"] = tuple(value / 2 for value in dimensions)
        elif visual.kind == bullet.GEOM_SPHERE:
            if not dimensions or dimensions[0] <= 0:
                raise ValueError("Replay spheres require a positive radius")
            result["radius"] = dimensions[0]
        elif visual.kind in (bullet.GEOM_CYLINDER, bullet.GEOM_CAPSULE):
            if len(dimensions) < 2 or dimensions[0] < 0 or dimensions[1] <= 0:
                raise ValueError("Replay cylinders and capsules require valid length and radius")
            result.update(length=dimensions[0], radius=dimensions[1])
        elif visual.kind == bullet.GEOM_MESH:
            if len(dimensions) != 3 or any(value == 0 for value in dimensions):
                raise ValueError("Replay mesh scale requires three nonzero values")
            if not visual.mesh_path or not Path(visual.mesh_path).is_file():
                raise ValueError("Replay mesh requires its pinned asset file")
            result.update(fileName=str(Path(visual.mesh_path)), meshScale=dimensions)
        else:
            # Bullet visual data does not expose a plane's normal or heightfield
            # samples. Guessing would silently change the recorded geometry.
            raise ValueError(f"Replay does not support visual geometry kind {visual.kind}")
        if visual.texture_path and not Path(visual.texture_path).is_file():
            raise ValueError("Replay texture requires its pinned asset file")
        return result

    def _check_owner(self):
        if threading.get_ident() != self._owner:
            raise RuntimeError("Replay renderer must be used and closed on its owner thread")
        if self._closed:
            raise RuntimeError("Replay renderer is closed")

    def set_frame(self, frame):
        self._check_owner()
        poses = _poses(frame.poses, self._row_count)
        previous = self._pose_provider.poses
        changed = np.any(poses != previous, axis=1)
        try:
            for body, row_index in self._body_rows.items():
                if changed[row_index]:
                    row = poses[row_index]
                    bullet.resetBasePositionAndOrientation(body, tuple(row[:3]), tuple(row[3:]),
                                                           physicsClientId=self.client)
            self._pose_provider.poses = poses
        except BaseException:
            self.close()
            raise

    def set_camera(self, *, yaw=None, pitch=None, distance=None, target=None):
        self._check_owner()
        camera = dict(self._camera)
        for key, value in (("yaw", yaw), ("pitch", pitch), ("distance", distance)):
            if value is not None:
                value = float(value)
                if not np.isfinite(value):
                    raise ValueError("Replay camera values must be finite")
                camera[key] = max(.1, value) if key == "distance" else value
        if target is not None:
            camera["target"] = _vector(target, 3, "camera target")
        self._camera = camera

    def render(self, width=960, height=720, *, yaw=None, pitch=None, distance=None, target=None):
        self._check_owner()
        width, height = int(width), int(height)
        if not (16 <= width <= 4096 and 16 <= height <= 4096):
            raise ValueError("Replay image dimensions must be between 16 and 4096")
        self.set_camera(yaw=yaw, pitch=pitch, distance=distance, target=target)
        pixels = self._backend.render(width, height, **self._camera)
        self.renderer_status = dict(self._backend.status)
        if pixels is not None:
            return pixels
        camera = self._camera
        view = bullet.computeViewMatrixFromYawPitchRoll(camera["target"], camera["distance"],
                                                        camera["yaw"], camera["pitch"], 0, 2)
        projection = bullet.computeProjectionMatrixFOV(52, width / height, .02, 30)
        image = bullet.getCameraImage(
            width, height, view, projection, renderer=bullet.ER_TINY_RENDERER,
            lightDirection=[-2, -3, 6], shadow=1, flags=bullet.ER_NO_SEGMENTATION_MASK,
            physicsClientId=self.client)
        pixels = np.asarray(image[2], dtype=np.uint8).reshape(height, width, 4).copy()
        depth = np.asarray(image[3]).reshape(height, width)
        pixels[depth >= .99999] = [9, 20, 32, 255]
        return pixels

    def close(self):
        if threading.get_ident() != self._owner:
            raise RuntimeError("Replay renderer must be closed on its owner thread")
        if self._closed:
            return
        self._closed = True
        try:
            if self._backend is not None:
                self._backend.close()
        finally:
            if self.client >= 0:
                bullet.disconnect(physicsClientId=self.client)
                self.client = -1
            self._clip = None
