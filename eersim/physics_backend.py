"""Select authoritative physics separately from graphics rendering."""
from __future__ import annotations

from importlib.util import find_spec

from .config import SimConfig
from .physics import World
from .physx_worker import cuda_device_info


def create_world(config=None):
    values = SimConfig.from_dict(config).to_dict()
    selected = values.get('physics_backend', 'auto')
    if selected == 'bullet':
        return World(values)
    reason = ''
    if find_spec('ovphysx') is None or find_spec('ovstage') is None:
        reason = 'PhysX runtime is not installed; install the simulator GPU dependencies'
    elif selected in ('auto', 'physx_gpu'):
        device = cuda_device_info(int(values.get('physics_device', 0)))
        if not device['available']:
            reason = device['reason']
    if not reason:
        from .physx_world import PhysXWorld
        values['physics_backend'] = 'physx_gpu' if selected == 'auto' else selected
        try:
            return PhysXWorld(values)
        except Exception as exc:
            if selected != 'auto':
                raise
            reason = f'PhysX initialization failed: {exc}'
    elif selected != 'auto':
        raise RuntimeError(reason)
    values['physics_backend'] = selected
    world = World(values)
    world.physics_status['reason'] = reason
    return world
