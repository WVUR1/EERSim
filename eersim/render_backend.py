"""Select GPU rendering independently of the fixed-step physics backend."""
from __future__ import annotations

import os

from .gpu_probe import probe_gpu


class RenderBackend:
    def __init__(self, client: int, mode: str = "auto", base_inertial_frames: dict | None = None, physics_api=None,
                 visual_shapes=None):
        # The environment override also gives headless CI a deliberate CPU path.
        mode = os.environ.get("EERSIM_RENDERER", mode).lower()
        if mode not in ("auto", "gpu", "cpu"):
            raise ValueError("renderer must be auto, gpu, or cpu")
        self.client, self.mode = client, mode
        self.base_inertial_frames = base_inertial_frames
        self.physics_api = physics_api
        # Recorded scenes already own complete visual descriptors. They do not
        # need a Bullet client unless this backend falls back to TinyRenderer.
        self.visual_shapes = None if visual_shapes is None else tuple(visual_shapes)
        self.renderer = None
        self.initialized = False
        self.status = {"backend": "cpu", "device": "Bullet TinyRenderer", "hardware_accelerated": False,
                       "reason": "CPU renderer selected" if mode == "cpu" else "Graphics initialization pending"}

    def render(self, width, height, yaw, pitch, distance, target):
        if not self.initialized:
            self.initialized = True
            if self.mode != "cpu":
                info = probe_gpu()
                if info.get("hardware_accelerated") or (self.mode == "gpu" and info.get("available")):
                    try:
                        import pybullet as bullet
                        from .rendering import HardwareRenderer
                        arguments = {"base_inertial_frames": self.base_inertial_frames,
                                     "physics_api": self.physics_api}
                        if self.visual_shapes is None:
                            bodies = [bullet.getBodyUniqueId(i, physicsClientId=self.client)
                                      for i in range(bullet.getNumBodies(physicsClientId=self.client))]
                        else:
                            bodies = list(dict.fromkeys(shape[0] for shape in self.visual_shapes))
                            arguments["visual_shapes"] = self.visual_shapes
                        self.renderer = HardwareRenderer(self.client, bodies, **arguments)
                        self.status = {"backend": "opengl", "device": info.get("device", "OpenGL"),
                                       "hardware_accelerated": info.get("hardware_accelerated", False),
                                       "version": info.get("version", ""), "reason": ""}
                    except Exception as exc:
                        self._fallback(str(exc))
                else:
                    self._fallback(info.get("error") or f"No hardware OpenGL context: {info.get('device', 'unavailable')}")
        if self.renderer is not None:
            try:
                pixels = self.renderer.render(width, height, yaw, pitch, distance, target)
                actual = self.renderer.status
                hardware = bool(actual.get("hardware", False))
                device = actual.get("renderer", "OpenGL")
                if self.mode == "auto" and not hardware:
                    self._fallback(f"The render context uses software graphics: {device}")
                    return None
                self.status = {"backend": "opengl", "device": device,
                               "hardware_accelerated": hardware,
                               "vendor": actual.get("vendor", ""), "version": actual.get("version", ""),
                               "reason": "" if hardware else "OpenGL software rendering"}
                return pixels
            except Exception as exc:
                self._fallback(str(exc))
        return None

    def _fallback(self, reason):
        self.close()
        self.status = {"backend": "cpu", "device": "Bullet TinyRenderer", "hardware_accelerated": False,
                       "reason": f"GPU fallback: {reason}"}

    def close(self):
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
