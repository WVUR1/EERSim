"""Real CPU regression for zero-wrench waking and sleeping-contact presence.

Runs a tiny two-cube scene in an owned subprocess with a 60-second limit.
No graphics, CUDA, native compiler, or external assets are needed.
"""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


def _scene_text():
    bodies = []
    for name, x in [('left', -2), ('right', 2)]:
        bodies.append('''
    def Cube "%s" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI",
        "PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI", "MaterialBindingAPI"])
    {
        double size = 0.1
        double3 xformOp:translate = (%s, 0, 0.05)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        float physics:mass = 1
        float physxRigidBody:linearDamping = 0
        float physxRigidBody:angularDamping = 0
        int physxRigidBody:solverPositionIterationCount = 100
        int physxRigidBody:solverVelocityIterationCount = 1
        float physxContactReport:threshold = 0
        rel material:binding:physics = </World/material>
    }
''' % (name, x))
    return '''#usda 1.0
(defaultPrim = "World"; metersPerUnit = 1; upAxis = "Z")
def Xform "World"
{
    def PhysicsScene "physicsScene" (prepend apiSchemas = ["PhysxSceneAPI"])
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.80665
        bool physxScene:enableGPUDynamics = false
        token physxScene:broadphaseType = "MBP"
        token physxScene:solverType = "TGS"
        int physxScene:minPositionIterationCount = 100
        int physxScene:maxPositionIterationCount = 100
    }
    def Material "material" (prepend apiSchemas = ["PhysicsMaterialAPI"])
    {
        float physics:staticFriction = 0
        float physics:dynamicFriction = 0
        float physics:restitution = 0
    }
    def Cube "floor" (prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"])
    {
        double size = 1
        double3 xformOp:translate = (0, 0, -0.05)
        double3 xformOp:scale = (20, 20, 0.1)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        rel material:binding:physics = </World/material>
    }
''' + ''.join(bodies) + '}\n'


def _cpu_proof(work):
    # Import only inside the owned child: PhysX has process-global native state.
    import numpy as np
    import ovstage
    from ovphysx import PhysX, PhysXConfig, TensorType
    from eersim.performance import PerformanceRecorder
    from eersim.physx_contacts import ContactReader
    from eersim.physx_worker import _Runtime

    scene = work / 'cubes.usda'
    scene.write_text(_scene_text(), encoding='utf-8')
    PhysX.set_cpu_mode(True)
    physx = PhysX(config=PhysXConfig(num_threads=2,
        cooked_collider_cache_dir=str(work / 'cooked'), carbonite_overrides={
            '/persistent/physics/numThreads': 2, '/physics/physxDispatcher': True,
            '/physics/suppressReadback': False}))
    stage = ovstage.Stage('zero-wrench-regression')
    contacts = None
    bindings = {}
    try:
        ovstage.population.open_usd(stage, str(scene), ordinal=1,
                                   domains=ovstage.PopulationDomain.PHYSICS)
        stage.advance_write_floor(ordinal=1).wait()
        physx.attach_ovstage(stage, read_ordinal=1)
        physx.wait_all()
        paths = ['/World/left', '/World/right']
        manifest = {'robot': {'body_id': 1}, 'actors': [
            {'path': path, 'body_id': i, 'link_index': -1, 'mass_kg': 1 if i < 2 else 0}
            for i, path in enumerate(paths + ['/World/floor'])]}
        contacts = ContactReader(physx, manifest, capacity=64)
        for name, kind in [('poses', TensorType.RIGID_BODY_POSE),
                           ('velocities', TensorType.RIGID_BODY_VELOCITY),
                           ('body_wrenches', TensorType.RIGID_BODY_WRENCH)]:
            bindings[name] = physx.create_tensor_binding(tensor_type=kind,
                prim_paths=paths, raise_if_empty=True)
            assert bindings[name].prim_paths == paths
        runtime = _Runtime.__new__(_Runtime)
        runtime.np, runtime.physx, runtime.dt = np, physx, 1 / 240
        runtime.performance = PerformanceRecorder()
        runtime.contacts, runtime.manifest = contacts, manifest
        runtime.bindings = bindings
        runtime.buffers = {name: np.zeros(binding.shape, dtype=np.float32)
                           for name, binding in bindings.items()}
        runtime.actor_paths = paths
        runtime.regular_paths = paths
        runtime.actor_rows = {path: i for i, path in enumerate(paths)}
        controls = {'wrenches': np.zeros((2, 9), dtype=np.float32)}
        physx.step_sync(runtime.dt)
        report = {'backend': 'CPU', 'physics_hz': 240, 'position_iterations': 100}

        # The old full-row write continually resets wake counters. Genuine zero
        # contact forces still give active contact reports, with no body motion.
        zero = controls['wrenches']
        for _ in range(720):
            bindings['body_wrenches'].write(zero)
            physx.step_sync(runtime.dt)
            state = runtime.state()
        report['forced_zero_contact_reports'] = int(contacts._counts.sum())
        assert report['forced_zero_contact_reports'] == 8

        # Real _Runtime.step must skip inactive rows. At native rest, fresh
        # contact reports cease; geometric body-pair presence must persist.
        for _ in range(720):
            state = runtime.step(controls)
        report['rest_contact_reports'] = int(contacts._counts.sum())
        report['rest_body_pairs'] = state['contact_body_pairs']
        assert report['rest_contact_reports'] == 0
        assert (0, 2) in state['contact_body_pairs']
        assert state['contacts'] == []  # No invented force/manifold records.
        np.testing.assert_array_equal(state['velocities'], np.zeros((2, 6)))
        resting_pose = state['poses'].copy()

        # Nonzero point coordinates alone are not a force; resting actors must
        # remain asleep even when their wrench application positions are set.
        zero[:, 6:] = resting_pose[:, :3]
        state = runtime.step(controls)
        assert int(contacts._counts.sum()) == 0
        np.testing.assert_array_equal(state['poses'], resting_pose)

        # No epsilon: even 1 nN wakes the selected body, not the other cube.
        zero[0, 0] = 1e-9
        state = runtime.step(controls)
        assert state['velocities'][0, 0] > 0
        np.testing.assert_array_equal(state['velocities'][1], np.zeros(6))
        assert contacts._counts.tolist() == [4, 0]
        zero.fill(0)
        for _ in range(720):
            state = runtime.step(controls)
        assert int(contacts._counts.sum()) == 0

        # Forces auto-clear after their one requested tick: F dt / m = 1 m/s.
        # Frictionless support and zero damping make a residual force visible.
        zero[0, 0] = 240
        zero[0, 6:] = state['poses'][0, :3]
        state = runtime.step(controls)
        first_speed = float(state['velocities'][0, 0])
        assert abs(first_speed - 1) < 1e-5
        zero.fill(0)
        for _ in range(30):
            state = runtime.step(controls)
        report['pulse_velocity_initial'] = first_speed
        report['pulse_velocity_after_30_skipped_steps'] = float(state['velocities'][0, 0])
        assert abs(state['velocities'][0, 0] - first_speed) < 1e-5

        # A moved-away body must immediately lose its cached floor support.
        moved = state['poses'].copy()
        moved[0, 2] = 1
        bindings['poses'].write(moved, indices=np.array([0], dtype=np.int32))
        for _ in range(2):
            state = runtime.step(controls)
        assert state['poses'][0, 2] > .99
        assert (0, 2) not in state['contact_body_pairs']
        report['moved_body_pairs'] = state['contact_body_pairs']
        report['passed'] = True
        (work / 'result.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    finally:
        if contacts is not None:
            contacts.close()
        for binding in bindings.values():
            binding.destroy()
        physx.detach_ovstage()
        physx.release()
        stage.destroy()


@pytest.mark.skipif(any(importlib.util.find_spec(name) is None for name in ('ovphysx', 'ovstage')),
                    reason='The optional pinned PhysX CPU runtime is not installed')
def test_real_cpu_zero_wrench_sleep_force_clear_and_contact_lifecycle(tmp_path):
    import eersim
    env = os.environ.copy()
    env['PYTHONPATH'] = str(Path(eersim.__file__).resolve().parent.parent)
    env['OPENBLAS_NUM_THREADS'] = env['OMP_NUM_THREADS'] = '1'
    result = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--cpu-proof', str(tmp_path)],
        capture_output=True, timeout=60, env=env,
        creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
    assert result.returncode == 0, (result.stdout + result.stderr).decode(errors='replace')[-12000:]
    assert json.loads((tmp_path / 'result.json').read_text())['passed'] is True


if __name__ == '__main__' and len(sys.argv) == 3 and sys.argv[1] == '--cpu-proof':
    _cpu_proof(Path(sys.argv[2]))
