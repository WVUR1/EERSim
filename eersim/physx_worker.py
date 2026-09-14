"""Isolated PhysX execution; one attached stage per spawned process.

The parent owns the PROS controller and cached observations. Each request advances
exactly one fixed tick and returns a complete snapshot. Rendering never steps it.
"""
from __future__ import annotations

import ctypes
import importlib.util
import multiprocessing
import os
from pathlib import Path
import sys
import traceback

from .performance import PerformanceRecorder


def cuda_device_info(index: int = 0) -> dict:
    """Query the CUDA driver without importing PhysX or creating a GL window."""
    library = 'nvcuda.dll' if sys.platform == 'win32' else 'libcuda.so.1'
    try:
        driver = ctypes.WinDLL(library) if sys.platform == 'win32' else ctypes.CDLL(library)
    except (OSError, ImportError):
        # This is supplied by the system NVIDIA driver, not by the application.
        # Frozen ctypes loaders can otherwise misreport it as a missing bundle DLL.
        return {'available': False, 'ordinal': index,
                'reason': f'NVIDIA CUDA driver is unavailable or could not be loaded ({library}). '
                          'Install a compatible NVIDIA driver or select Bullet physics.'}
    try:
        def check(result):
            if result:
                raise RuntimeError(f'CUDA driver returned error {result}')
        check(driver.cuInit(0))
        count = ctypes.c_int()
        check(driver.cuDeviceGetCount(ctypes.byref(count)))
        if not 0 <= index < count.value:
            raise RuntimeError(f'CUDA device {index} is unavailable ({count.value} visible devices)')
        device = ctypes.c_int()
        check(driver.cuDeviceGet(ctypes.byref(device), index))
        name = ctypes.create_string_buffer(256)
        check(driver.cuDeviceGetName(name, len(name), device))
        major, minor = ctypes.c_int(), ctypes.c_int()
        check(driver.cuDeviceComputeCapability(ctypes.byref(major), ctypes.byref(minor), device))
        version = ctypes.c_int()
        check(driver.cuDriverGetVersion(ctypes.byref(version)))
        if major.value < 7 or version.value < 12080:
            raise RuntimeError('PhysX requires compute capability 7.0+ and a CUDA 12.8-compatible driver')
        return {'available': True, 'device': name.value.decode('utf-8', 'replace'),
                'ordinal': index, 'driver_version': version.value,
                'compute_capability': [major.value, minor.value]}
    except (OSError, RuntimeError, AttributeError) as exc:
        return {'available': False, 'reason': str(exc), 'ordinal': index}


class PhysXProcess:
    """Bounded, synchronous control channel to a private physics process."""
    def __init__(self, scene: Path, manifest: dict, config: dict, work: Path):
        self.performance = PerformanceRecorder()
        self.failed = False
        self.work = Path(work)
        self.work.mkdir(parents=True, exist_ok=True)
        self.log_path = self.work / 'physics.log'
        context = multiprocessing.get_context('spawn')
        self.connection, child = context.Pipe()
        self.process = context.Process(target=_worker_main,
            args=(child, str(scene), manifest, config, str(self.work)), daemon=True)
        with self.performance.span("physx.process.spawn"):
            self.process.start()
        child.close()
        try:
            with self.performance.span("physx.process.ready"):
                self.initial = self._receive(180)
        except BaseException:
            self.close()
            raise

    def _receive(self, timeout):
        try:
            return self._receive_checked(timeout)
        except BaseException:
            self.failed = True
            raise

    def _receive_checked(self, timeout):
        if not self.connection.poll(timeout):
            status = 'exited' if not self.process.is_alive() else 'timed out'
            raise RuntimeError(f'PhysX worker {status}; see {self.log_path}')
        try:
            success, payload = self.connection.recv()
        except EOFError as exc:
            raise RuntimeError(f'PhysX worker exited unexpectedly; see {self.log_path}') from exc
        if not success:
            raise RuntimeError(f'{payload}\nPhysX log: {self.log_path}')
        return payload

    def call(self, command: str, payload=None):
        with self.performance.span(f"physx.rpc.{command}"):
            if not self.process.is_alive():
                self.failed = True
                raise RuntimeError(f'PhysX worker is not running; see {self.log_path}')
            try:
                self.connection.send((command, payload))
                return self._receive(60)
            except (EOFError, BrokenPipeError, OSError):
                self.failed = True
                raise RuntimeError(f'PhysX worker connection failed; see {self.log_path}') from None

    def close(self):
        process = getattr(self, 'process', None)
        if process is None:
            return
        # A completed request leaves this serialized channel empty. After a
        # timeout/connection failure, do not send into a possibly blocked pipe.
        if process.is_alive() and not self.failed:
            try:
                self.connection.send(('close', None))
                process.join(5)
            except (EOFError, OSError, ValueError):
                pass
        for stop in (process.terminate, process.kill):
            if not process.is_alive():
                break
            try:
                stop()
                process.join(5)
            except OSError:
                # An exit can race the stop call. Check again, then escalate if
                # needed; a failed terminate must not prevent the kill attempt.
                pass
        if process.is_alive():
            self.failed = True
            # Retain both handles so the owner can retry. In particular, World
            # must not delete this worker's runtime directory while it is live.
            raise RuntimeError(f'PhysX worker did not exit after terminate and kill; see {self.log_path}')
        self.connection.close()


def diagnostic_thread_count(configured=2):
    """Use the saved CPU worker count, with the legacy environment override."""
    from .config import validate_physics_cpu_threads
    configured = validate_physics_cpu_threads(configured)
    value = os.environ.get('EERSIM_PHYSX_THREADS')
    if value is None:
        return configured
    if not value.isascii() or not value.isdecimal() or len(value) > 4 or int(value) > 1024:
        raise ValueError('EERSIM_PHYSX_THREADS must be an integer between 0 and 1024; '
                         '0 runs inline tasks')
    return int(value)


class _Runtime:
    def __init__(self, scene, manifest, config, work, performance=None):
        self.performance = performance if performance is not None else PerformanceRecorder()
        import numpy as np
        import ovstage
        from ovphysx import PhysX, PhysXConfig, TensorType
        from .physx_contacts import ContactReader
        self.np, self.TensorType = np, TensorType
        self.manifest, self.config = manifest, config
        self.dt = 1.0 / config['physics_hz']
        self.bindings, self.buffers = {}, {}
        self.physx = self.stage = self.contacts = None
        threads = diagnostic_thread_count(config.get('physics_cpu_threads', 2))
        gpu = config['physics_backend'] == 'physx_gpu'
        device = int(config.get('physics_device', 0))
        if gpu:
            info = cuda_device_info(device)
            if not info['available']:
                raise RuntimeError(info['reason'])
        else:
            info = {'device': 'PhysX CPU', 'ordinal': None}
        PhysX.set_cpu_mode(not gpu)
        self.physx = PhysX(active_cuda_gpus=str(device) if gpu else None,
            config=PhysXConfig(num_threads=threads, cooked_collider_cache_dir=str(work / 'cooked'),
                carbonite_overrides={'/persistent/physics/numThreads': threads,
                    '/physics/physxDispatcher': True,
                    # Standard readback keeps scene queries and CPU controller
                    # observations current while collision/constraints use CUDA.
                    '/physics/suppressReadback': False}))
        self.stage = ovstage.Stage('eersim-physics')
        ovstage.population.open_usd(self.stage, str(scene), ordinal=1,
                                   domains=ovstage.PopulationDomain.PHYSICS)
        self.stage.advance_write_floor(ordinal=1).wait()
        self.physx.attach_ovstage(self.stage, read_ordinal=1)
        self.physx.wait_all()
        self.contacts = ContactReader(self.physx, manifest, performance=self.performance)
        self._bind('poses', TensorType.RIGID_BODY_POSE, pattern='/World/rigid_*')
        self._bind('velocities', TensorType.RIGID_BODY_VELOCITY, pattern='/World/rigid_*')
        self.actor_paths = self.bindings['poses'].prim_paths
        if self.bindings['velocities'].prim_paths != self.actor_paths:
            raise RuntimeError('PhysX pose and velocity row orders differ')
        self.actor_rows = {path: index for index, path in enumerate(self.actor_paths)}
        expected = {actor['path'] for actor in manifest['actors'] if actor['mass_kg'] > 0}
        if set(self.actor_paths) != expected:
            raise RuntimeError('PhysX did not realize every authored dynamic link')
        root = manifest.get('articulation_root') or manifest.get('robot', {}).get('articulation_root')
        self.root_path = root
        self.dof_names = []
        self.link_names = []
        self.articulation_actor_rows = []
        if root:
            for key, kind in [('q', TensorType.ARTICULATION_DOF_POSITION),
                ('qd', TensorType.ARTICULATION_DOF_VELOCITY),
                ('target_velocity', TensorType.ARTICULATION_DOF_VELOCITY_TARGET),
                ('target_position', TensorType.ARTICULATION_DOF_POSITION_TARGET),
                ('max_force', TensorType.ARTICULATION_DOF_MAX_FORCE),
                ('stiffness', TensorType.ARTICULATION_DOF_STIFFNESS),
                ('damping', TensorType.ARTICULATION_DOF_DAMPING),
                ('effort', TensorType.ARTICULATION_DOF_ACTUATION_FORCE),
                ('joint_friction', TensorType.ARTICULATION_DOF_FRICTION_PROPERTIES),
                ('link_wrenches', TensorType.ARTICULATION_LINK_WRENCH)]:
                self._bind(key, kind, prim_paths=[root])
            self.dof_names = self.bindings['q'].dof_names
            self.link_names = self.bindings['link_wrenches'].body_names
            for name in self.link_names:
                matches = [row for path, row in self.actor_rows.items()
                           if path == name or path.rsplit('/', 1)[-1] == name]
                if len(matches) != 1:
                    raise RuntimeError(f'Cannot resolve articulation link {name}')
                self.articulation_actor_rows.append(matches[0])
        self.regular_paths = [path for path, row in self.actor_rows.items()
                              if row not in self.articulation_actor_rows]
        if self.regular_paths:
            self._bind('body_wrenches', TensorType.RIGID_BODY_WRENCH, prim_paths=self.regular_paths)
            self.regular_paths = self.bindings['body_wrenches'].prim_paths
        # Do not call warmup_gpu(): that performs a real 1 ns step in 0.5.11.
        with self.performance.span("worker.native.bootstrap"):
            self.physx.step_sync(self.dt)
        if self.performance.enabled:
            self.performance.increment("worker.bootstrap_steps")
            self.performance.gauge("scene.dynamic_actors", len(self.actor_paths))
            self.performance.gauge("scene.articulation_links", len(self.articulation_actor_rows))
            self.performance.gauge("scene.dofs", len(self.dof_names))
            self.performance.gauge("scene.collision_shapes", manifest.get("counts", {}).get("collision_shapes", 0))
        verified = {}
        if gpu:
            from .physx_verification import verify_gpu_scene
            verified = verify_gpu_scene(self.physx, manifest['scene_path'])
        if 'joint_friction' in self.bindings:
            friction = self.buffers['joint_friction']
            for joint in manifest['joints']:
                if not joint.get('articulation') or joint['type'] == 'fixed':
                    continue
                name = joint['path'].rsplit('/', 1)[-1]
                indices = [i for i, value in enumerate(self.dof_names) if value in (name, joint['path'], joint['name'])]
                if len(indices) != 1:
                    raise RuntimeError(f'Cannot resolve passive joint friction for {name}')
                friction[0, indices[0]] = [joint.get('friction', 0), joint.get('friction', 0), joint.get('damping', 0)]
            self.bindings['joint_friction'].write(friction)
        self.status = {'backend': 'physx', 'device': verified.get('cuda_device_name') or info['device'],
                       'hardware_accelerated': gpu, 'reason': '', **verified,
                       'requested_cpu_workers': threads,
                       'solver': 'TGS', 'solver_iterations': config.get('physics_solver_iterations', 100),
                       'tensor_cores': False, 'state_exchange': 'batched CPU observations',
                       'initialization_steps': 1, 'motor_torque_observation': 'bounded implicit-drive estimate',
                       'scene_query_identity': 'native opaque IDs, not Bullet body/link IDs'}

    def _bind(self, name, kind, **selection):
        binding = self.physx.create_tensor_binding(tensor_type=kind, raise_if_empty=True, **selection)
        self.bindings[name] = binding
        self.buffers[name] = self.np.zeros(binding.shape, dtype=self.np.float32)

    def state(self):
        with self.performance.span("worker.state.read"):
            reads = 0
            for key in ('poses', 'velocities', 'q', 'qd'):
                if key in self.bindings:
                    self.bindings[key].read(self.buffers[key])
                    reads += 1
                    if not self.np.isfinite(self.buffers[key]).all():
                        raise RuntimeError(f'Non-finite PhysX {key}; simulation stopped')
            result = {'poses': self.buffers['poses'].copy(), 'velocities': self.buffers['velocities'].copy(),
                      'q': self.buffers.get('q', self.np.empty((1, 0))).copy(),
                      'qd': self.buffers.get('qd', self.np.empty((1, 0))).copy()}
        with self.performance.span("worker.contacts.read_convert"):
            contacts = self.contacts.capture_runtime(self.manifest['robot']['body_id'],
                poses_by_actor=dict(zip(self.actor_paths, self.buffers['poses'])))
        result.update(contacts)
        if self.performance.enabled:
            self.performance.increment("worker.state.tensor_reads", reads)
            raw = int(self.contacts._counts.sum())
            self.performance.increment("worker.contacts.raw_reports", raw)
            self.performance.gauge("worker.contacts.raw_reports_last", raw)
            self.performance.gauge("worker.contacts.materialized_last", len(contacts['contacts']))
            self.performance.gauge("worker.contacts.body_pairs_last", len(contacts['contact_body_pairs']))
            self.performance.gauge("worker.contacts.retained_body_pairs_last", self.contacts.retained_pair_count)
        return result

    def diagnostics(self, payload=None):
        payload = {} if payload is None else payload
        if (not isinstance(payload, dict) or set(payload) - {'reset', 'activity'} or
                any(not isinstance(payload.get(key, False), bool) for key in ('reset', 'activity'))):
            raise ValueError('Diagnostics accepts only optional boolean reset and activity flags')
        result = {}
        if payload.get('activity', False):
            from .physx_verification import inspect_scene_activity
            with self.performance.span('worker.diagnostics.scene_activity'):
                result['scene_activity'] = inspect_scene_activity(self.physx, self.manifest['scene_path'])
            # Native names are authoritative. Attach labels only for unambiguous
            # exact prim paths (or their exact unique basename); never guess IDs.
            actors = self.manifest['actors']
            mapped = 0
            for row in result['scene_activity'].get('actors', []):
                name = row.get('name')
                matches = [actor for actor in actors if name and
                           (actor['path'] == name or actor['path'].rsplit('/', 1)[-1] == name)]
                if len(matches) == 1:
                    actor = matches[0]
                    row['manifest_body_id'] = actor['body_id']
                    row['manifest_link_index'] = actor['link_index']
                    row['manifest_label'] = actor.get('label', '')
                    mapped += 1
            result['scene_activity']['manifest_mapped_actor_rows'] = mapped
        result['performance'] = self.performance.snapshot(reset=payload.get('reset', False))
        return result

    def step(self, controls):
        writes = submitted_rows = skipped_rows = 0
        with self.performance.span('worker.control.write'):
            for key in ('target_velocity', 'target_position', 'max_force', 'stiffness', 'damping', 'effort'):
                if key in self.bindings:
                    self.buffers[key][...] = controls[key]
                    self.bindings[key].write(self.buffers[key])
                    writes += 1
            wrenches = controls['wrenches']
            if 'body_wrenches' in self.bindings:
                values = self.buffers['body_wrenches']
                for row, path in enumerate(self.regular_paths):
                    values[row] = wrenches[self.actor_rows[path]]
                # The pinned SDK autowakes every selected regular body even
                # for an all-zero wrench. Forces clear after each step, so
                # submit only active force/torque rows and let idle props sleep.
                # Application-point coordinates alone do not apply a force.
                indices = self.np.flatnonzero(self.np.any(values[:, :6] != 0, axis=1)).astype(self.np.int32)
                submitted_rows = len(indices)
                skipped_rows = len(values) - submitted_rows
                if submitted_rows:
                    # Partial writes still require the binding's full buffer.
                    self.bindings['body_wrenches'].write(values, indices=indices)
                    writes += 1
            if 'link_wrenches' in self.bindings:
                self.buffers['link_wrenches'][0, ...] = wrenches[self.articulation_actor_rows]
                self.bindings['link_wrenches'].write(self.buffers['link_wrenches'])
                writes += 1
        if self.performance.enabled:
            self.performance.increment("worker.control.tensor_writes", writes)
            self.performance.increment("worker.control.regular_wrench_rows_submitted", submitted_rows)
            self.performance.increment("worker.control.regular_wrench_rows_skipped", skipped_rows)
        with self.performance.span("worker.native.step"):
            self.physx.step_sync(self.dt)
        if self.performance.enabled:
            self.performance.increment("worker.steps")
        return self.state()

    def raycast(self, rays):
        result = []
        for start, end in rays:
            start, end = self.np.asarray(start, dtype=float), self.np.asarray(end, dtype=float)
            if start.shape != (3,) or end.shape != (3,) or not self.np.isfinite([start, end]).all():
                raise ValueError('Ray endpoints must contain three finite coordinates')
            length = float(self.np.linalg.norm(end-start))
            hits = self.physx.raycast(start, (end-start)/length, length) if length else []
            result.append(dict(hits[0]) if hits else None)
        return result

    def close(self):
        if self.contacts is not None:
            self.contacts.close()
        for binding in reversed(list(self.bindings.values())):
            binding.destroy()
        if self.physx is not None:
            self.physx.detach_ovstage()
            self.physx.release()
        if self.stage is not None:
            self.stage.destroy()


def _worker_main(connection, scene, manifest, config, work):
    work = Path(work)
    runtime = None
    try:
        for variable, child in [('XDG_CACHE_HOME', 'cache'), ('XDG_CONFIG_HOME', 'config'),
                                ('XDG_DATA_HOME', 'data'), ('CUDA_CACHE_PATH', 'cuda-cache')]:
            path = work / child
            path.mkdir(parents=True, exist_ok=True)
            os.environ[variable] = str(path)
        os.environ['OPENBLAS_NUM_THREADS'] = '1'
        stream = (work / 'physics.log').open('a', encoding='utf-8', buffering=1)
        sys.stdout = sys.stderr = stream
        for descriptor in (1, 2):
            try:
                os.dup2(stream.fileno(), descriptor)
            except OSError:
                pass
        performance = PerformanceRecorder()
        with performance.span("worker.setup"):
            runtime = _Runtime(Path(scene), manifest, config, work, performance=performance)
        connection.send((True, {'state': runtime.state(), 'actor_paths': runtime.actor_paths,
                               'dof_names': runtime.dof_names, 'physics_status': runtime.status}))
        while True:
            command, payload = connection.recv()
            if command == 'close':
                break
            if command == 'step':
                result = runtime.step(payload)
            elif command == 'raycast':
                with runtime.performance.span("worker.sensor.raycast"):
                    result = runtime.raycast(payload)
                if runtime.performance.enabled:
                    runtime.performance.increment("worker.sensor.rays", len(payload))
            elif command == 'contacts':
                # Explicit diagnostics fetch full manifolds from this same
                # completed tick; the normal snapshot sends only robot points.
                with runtime.performance.span("worker.contacts.full_capture"):
                    result = runtime.contacts.capture()
            elif command == 'diagnostics':
                result = runtime.diagnostics(payload)
            else:
                raise ValueError(f'Unknown physics request {command}')
            connection.send((True, result))
    except EOFError:
        pass
    except BaseException as exc:
        traceback.print_exc()
        try:
            connection.send((False, f'{type(exc).__name__}: {exc}'))
        except (EOFError, BrokenPipeError, OSError):
            pass
    finally:
        if runtime is not None:
            try:
                runtime.close()
            except Exception:
                traceback.print_exc()
        connection.close()
