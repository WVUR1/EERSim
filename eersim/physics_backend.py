"""Select authoritative physics separately from graphics rendering."""
from __future__ import annotations

from importlib.util import find_spec

from .config import SimConfig
from .physics import World
from .physx_worker import cuda_device_info


def create_world(config=None):
    values = SimConfig.from_dict(config).to_dict()
    selected = values.get("physics_backend", "auto")
    if selected == "bullet":
        return World(values)
    if selected in ("auto", "mujoco"):
        reason = ""
        if find_spec("mujoco") is None:
            reason = "MuJoCo runtime is not installed; rerun setup or install the simulator dependencies"
        else:
            try:
                from .mujoco_world import MuJoCoWorld
                return MuJoCoWorld(values)
            except Exception as exc:
                if selected == "mujoco":
                    raise
                reason = f"MuJoCo initialization failed: {exc}"
        if selected == "mujoco":
            raise RuntimeError(reason)
        world = World(values)
        world.physics_status["reason"] = reason
        return world
    if find_spec("ovphysx") is None or find_spec("ovstage") is None:
        raise RuntimeError("PhysX runtime is not installed; install the simulator GPU dependencies")
    if selected == "physx_gpu":
        device = cuda_device_info(int(values.get("physics_device", 0)))
        if not device["available"]:
            raise RuntimeError(device["reason"])
    from .physx_world import PhysXWorld
    return PhysXWorld(values)
