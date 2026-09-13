"""PhysX authority with the shared VEX control and observation model.

Bullet retains imported geometry for inspection and CPU rendering. It never
advances physics here. All live poses, velocities, contacts and joints come from
the isolated PhysX worker; physical forces are submitted back to that worker.
"""
from __future__ import annotations

import math
from pathlib import Path
import shutil
import tempfile

import numpy as np
import pybullet as bullet

from .physics import World
from .physx_worker import PhysXProcess
from .runtime_paths import user_data_dir


def rotation(quaternion):
    return np.asarray(bullet.getMatrixFromQuaternion(quaternion)).reshape(3, 3)


class PhysXAdapter:
    def __init__(self, world, process, manifest):
        self.world, self.process, self.manifest = world, process, manifest
        self.client = world.client
        self.actors = {(a['body_id'], a['link_index']): a for a in manifest['actors']}
        self.rows = {path: row for row, path in enumerate(process.initial['actor_paths'])}
        self.keys = {key: self.rows[a['path']] for key, a in self.actors.items() if a['path'] in self.rows}
        self.joint_records = {(j['body_id'], j['joint_index']): j for j in manifest['joints']}
        names = process.initial['dof_names']
        self.dofs = {}
        for key, joint in self.joint_records.items():
            if not joint.get('articulation') or joint['type'] == 'fixed':
                continue
            matches = [index for index, name in enumerate(names)
                       if name in (joint['path'], joint['path'].rsplit('/', 1)[-1], joint['name'])]
            if len(matches) != 1:
                raise RuntimeError(f"PhysX degree of freedom missing or ambiguous: {joint['name']}")
            self.dofs[key] = matches[0]
        self.controls = {name: np.zeros((1, len(names)), np.float32) for name in
                         ('target_velocity', 'target_position', 'max_force', 'stiffness', 'damping', 'effort')}
        self.controls['wrenches'] = np.zeros((len(self.rows), 9), np.float32)
        self._applied_effort = {key: 0.0 for key in self.dofs}
        self._state = process.initial['state']
        self._invalidate_com_cache()
        self._invalidate_contact_index()
        self._shadow_dirty = True

    def __getattr__(self, name):
        # Only static import metadata and pure math may consult the geometry
        # shadow. Unknown live-state calls must not silently use stale Bullet.
        static = {'getDynamicsInfo', 'getJointInfo', 'getNumJoints', 'getBodyInfo',
                  'getVisualShapeData', 'getCollisionShapeData', 'getBodyUniqueId', 'getNumBodies',
                  'getQuaternionFromEuler', 'getEulerFromQuaternion', 'getMatrixFromQuaternion',
                  'multiplyTransforms', 'invertTransform', 'getAxisAngleFromQuaternion',
                  'getQuaternionFromAxisAngle', 'getDifferenceQuaternion', 'getQuaternionSlerp',
                  'computeViewMatrix', 'computeViewMatrixFromYawPitchRoll',
                  'computeProjectionMatrix', 'computeProjectionMatrixFOV'}
        if name in static:
            return getattr(bullet, name)
        value = getattr(bullet, name, None)
        if name.isupper() and value is not None and not callable(value):
            return value
        if callable(value):
            raise NotImplementedError(f'PhysX runtime API does not implement {name}; the Bullet geometry shadow is read-only')
        raise AttributeError(name)

    def _actor_pose(self, body, link):
        key = (body, link)
        if key in self.keys:
            row = self._state['poses'][self.keys[key]]
            return row[:3], row[3:]
        actor = self.actors[key]
        return np.asarray(actor['position']), np.asarray(actor['orientation'])

    def _invalidate_com_cache(self):
        # Worker states are completed, immutable snapshots. Reset/load creates
        # a new adapter; each successful step replaces the snapshot and clears
        # these derived values. Nothing is cached across physical ticks.
        self._com_cache_state = self._state
        self._com_frames = {}
        self._com_rotations = {}

    def _com_frame(self, body, link):
        # Also handle a snapshot replacement outside stepSimulation. Keep the
        # original Bullet transform, including its precision and principal
        # inertial frame, rather than recomputing it with different arithmetic.
        if self.__dict__.get('_com_cache_state') is not self._state:
            self._invalidate_com_cache()
        key = (body, link)
        frame = self._com_frames.get(key)
        if frame is None:
            point, orient = self._actor_pose(body, link)
            actor = self.actors[key]
            pose = bullet.multiplyTransforms(point, orient, actor['local_com'], actor['principal_axes'])
            frame = (pose, np.asarray(pose[0]))
            self._com_frames[key] = frame
        return frame

    def _com_pose(self, body, link):
        return self._com_frame(body, link)[0]

    def _com_rotation(self, key, quaternion):
        # Call only after _com_frame has checked the current state. LINK_FRAME
        # force, point and torque use this same world-from-COM rotation.
        matrix = self._com_rotations.get(key)
        if matrix is None:
            matrix = rotation(quaternion)
            self._com_rotations[key] = matrix
        return matrix

    def getBasePositionAndOrientation(self, bodyUniqueId, **kwargs):
        return self._com_pose(bodyUniqueId, -1)

    def getBaseVelocity(self, bodyUniqueId, **kwargs):
        row = self.keys.get((bodyUniqueId, -1))
        value = self._state['velocities'][row] if row is not None else np.zeros(6)
        return value[:3].tolist(), value[3:].tolist()

    def getLinkState(self, bodyUniqueId, linkIndex, **kwargs):
        actor = self.actors[(bodyUniqueId, linkIndex)]
        position, quaternion = self._actor_pose(bodyUniqueId, linkIndex)
        com_position, com_quaternion = self._com_pose(bodyUniqueId, linkIndex)
        row = self.keys.get((bodyUniqueId, linkIndex))
        velocity = self._state['velocities'][row] if row is not None else np.zeros(6)
        return (com_position, com_quaternion, actor['local_com'], actor['principal_axes'],
                position, quaternion, velocity[:3], velocity[3:])

    def getJointState(self, bodyUniqueId, jointIndex, **kwargs):
        key = (bodyUniqueId, jointIndex)
        if key in self.dofs:
            column = self.dofs[key]
            return (float(self._state['q'][0, column]), float(self._state['qd'][0, column]),
                    (0.0,) * 6, self._applied_effort[key])
        joint = self.joint_records[key]
        if joint['type'] == 'fixed':
            return (0.0, 0.0, (0.0,) * 6, 0.0)
        parent = joint['parent_link_index']
        child = joint['child_link_index']
        p0, q0 = self._actor_pose(bodyUniqueId, parent)
        p1, q1 = self._actor_pose(bodyUniqueId, child)
        origin = joint.get('origin_xyz', [0, 0, 0])
        origin_q = bullet.getQuaternionFromEuler(joint.get('origin_rpy', [0, 0, 0]))
        frame_point, frame_quat = bullet.multiplyTransforms(p0, q0, origin, origin_q)
        relative_point, relative_quat = bullet.multiplyTransforms(*bullet.invertTransform(frame_point, frame_quat), p1, q1)
        axis = np.asarray(joint['axis'])
        if joint['type'] == 'prismatic':
            position = float(np.dot(relative_point, axis))
        else:
            position = 2 * math.atan2(float(np.dot(relative_quat[:3], axis)), float(relative_quat[3]))
        axis_world = rotation(frame_quat) @ axis
        def motion(link, point):
            centre = np.asarray(self._com_pose(bodyUniqueId, link)[0])
            row = self.keys.get((bodyUniqueId, link))
            velocity = self._state['velocities'][row] if row is not None else np.zeros(6)
            return velocity[:3] + np.cross(velocity[3:], np.asarray(point)-centre), velocity[3:]
        anchor0_velocity, angular0 = motion(parent, frame_point)
        anchor1_velocity, angular1 = motion(child, p1)
        if joint['type'] == 'prismatic':
            relative_velocity = anchor1_velocity-anchor0_velocity-np.cross(angular0, np.asarray(p1)-frame_point)
        else:
            relative_velocity = angular1-angular0
        velocity = float(np.dot(axis_world, relative_velocity))
        return (position, velocity, (0.0,) * 6, 0.0)

    def getJointStates(self, bodyUniqueId, jointIndices, **kwargs):
        return [self.getJointState(bodyUniqueId, index) for index in jointIndices]

    def setJointMotorControl2(self, bodyUniqueId, jointIndex, controlMode, **kwargs):
        key = (bodyUniqueId, jointIndex)
        if key not in self.dofs:
            raise ValueError('Only robot articulation joints can be commanded')
        column = self.dofs[key]
        force = max(0.0, float(kwargs.get('force', 0)))
        for name in ('target_velocity', 'target_position', 'stiffness', 'damping', 'effort'):
            self.controls[name][0, column] = 0
        self.controls['max_force'][0, column] = force
        if controlMode == bullet.TORQUE_CONTROL:
            self.controls['effort'][0, column] = float(kwargs.get('force', 0))
            self.controls['max_force'][0, column] = 0
        elif controlMode in (bullet.VELOCITY_CONTROL, bullet.POSITION_CONTROL):
            target = float(kwargs.get('targetVelocity', 0))
            if controlMode == bullet.POSITION_CONTROL:
                position = self.getJointState(*key)[0]
                target += float(kwargs.get('positionGain', .1)) * (float(kwargs.get('targetPosition', 0)) - position) / self.world.dt
                limit = abs(float(kwargs.get('maxVelocity', self.joint_records[key].get('max_velocity') or 10)))
                target = min(limit, max(-limit, target))
            self.controls['target_velocity'][0, column] = target
            # The implicit force drive saturates at the VEX envelope supplied by
            # World._apply_motors. Finite damping avoids a prescribed velocity.
            self.controls['damping'][0, column] = 10000.0 if force else 0.0
        else:
            raise ValueError(f'Unsupported PhysX control mode: {controlMode}')

    def applyExternalForce(self, objectUniqueId, linkIndex, forceObj, posObj, flags, **kwargs):
        key = (objectUniqueId, linkIndex)
        if key not in self.keys:
            return
        force, point = np.asarray(forceObj), np.asarray(posObj)
        (position, quaternion), centre = self._com_frame(*key)
        if flags == bullet.LINK_FRAME:
            # Bullet's LINK_FRAME is the link COM/principal frame, including
            # the URDF inertial origin. Preserve that API for external forces.
            matrix = self._com_rotation(key, quaternion)
            force = matrix @ force
            point = centre + matrix @ point
        wrench = self.controls['wrenches'][self.keys[key]]
        wrench[:3] += force
        wrench[3:6] += np.cross(point-centre, force)
        wrench[6:] = centre

    def applyExternalTorque(self, objectUniqueId, linkIndex, torqueObj, flags, **kwargs):
        key = (objectUniqueId, linkIndex)
        if key not in self.keys:
            return
        torque = np.asarray(torqueObj)
        (position, quaternion), centre = self._com_frame(*key)
        if flags == bullet.LINK_FRAME:
            torque = self._com_rotation(key, quaternion) @ torque
        wrench = self.controls['wrenches'][self.keys[key]]
        wrench[3:6] += torque
        wrench[6:] = centre

    def _invalidate_contact_index(self):
        self._indexed_contacts = None
        self._contacts_by_body = {}
        self._contacts_by_link = {}
        self._contacts_by_pair = {}
        self._reversed_contacts = {}

    def _index_contacts(self, contacts):
        if self._indexed_contacts is contacts:
            return
        self._invalidate_contact_index()
        for index, contact in enumerate(contacts):
            a, b = contact[1], contact[2]
            left, right = (a, contact[3]), (b, contact[4])
            self._contacts_by_body.setdefault(a, []).append(index)
            if b != a:
                self._contacts_by_body.setdefault(b, []).append(index)
            self._contacts_by_link.setdefault(left, []).append(index)
            if right != left:
                self._contacts_by_link.setdefault(right, []).append(index)
            pair = (a, b) if a <= b else (b, a)
            self._contacts_by_pair.setdefault(pair, []).append(index)
        self._indexed_contacts = contacts

    def has_contact(self, body_a, body_b):
        """Contact presence for subscribed bodies, without materializing points."""
        if 'contact_body_pairs' not in self._state:
            return bool(self.getContactPoints(body_a, body_b))
        if self.__dict__.get('_contact_pairs_state') is not self._state:
            self._contact_pairs = {tuple(pair) for pair in self._state['contact_body_pairs']}
            self._contact_pairs_state = self._state
        pair = (body_a, body_b) if body_a <= body_b else (body_b, body_a)
        return pair in self._contact_pairs

    def getContactPoints(self, bodyA=-1, bodyB=-1, linkIndexA=-2, linkIndexB=-2, **kwargs):
        contacts = self._state['contacts']
        if 'contact_body_pairs' in self._state and bodyA != self.world.robot and bodyB != self.world.robot:
            # Scoring uses has_contact(). Full subscribed manifolds remain
            # available on demand for inspection, without a physics step.
            if self.__dict__.get('_full_contacts_state') is not self._state:
                self._full_contacts = self.process.call('contacts')
                self._full_contacts_state = self._state
            contacts = self._full_contacts
        if bodyA < 0 and bodyB < 0 and linkIndexA == -2 and linkIndexB == -2:
            return list(contacts)
        self._index_contacts(contacts)
        candidates = []
        if bodyA >= 0:
            candidates.append(self._contacts_by_link.get((bodyA, linkIndexA), ()) if linkIndexA != -2
                              else self._contacts_by_body.get(bodyA, ()))
        if bodyB >= 0:
            candidates.append(self._contacts_by_link.get((bodyB, linkIndexB), ()) if linkIndexB != -2
                              else self._contacts_by_body.get(bodyB, ()))
        if bodyA >= 0 and bodyB >= 0:
            pair = (bodyA, bodyB) if bodyA <= bodyB else (bodyB, bodyA)
            candidates.append(self._contacts_by_pair.get(pair, ()))
        indices = min(candidates, key=len) if candidates else range(len(contacts))
        result = []
        for index in indices:
            contact = contacts[index]
            a, b, left, right = contact[1:5]
            if ((bodyA < 0 or a == bodyA) and (bodyB < 0 or b == bodyB)
                    and (linkIndexA == -2 or left == linkIndexA)
                    and (linkIndexB == -2 or right == linkIndexB)):
                result.append(contact)
            elif ((bodyA < 0 or b == bodyA) and (bodyB < 0 or a == bodyB)
                    and (linkIndexA == -2 or right == linkIndexA)
                    and (linkIndexB == -2 or left == linkIndexB)):
                # Inspect IDs before reversing vectors. Most field contacts do
                # not belong to the queried wheel or scoring pair.
                if index not in self._reversed_contacts:
                    self._reversed_contacts[index] = (
                        contact[0], b, a, right, left, contact[6], contact[5],
                        tuple(-x for x in contact[7]), *contact[8:])
                result.append(self._reversed_contacts[index])
        return result

    def rayTest(self, rayFromPosition, rayToPosition, **kwargs):
        return self.rayTestBatch([rayFromPosition], [rayToPosition], **kwargs)

    def rayTestBatch(self, rayFromPositions, rayToPositions, **kwargs):
        unsupported = set(kwargs) - {'physicsClientId', 'numThreads'}
        if unsupported:
            raise NotImplementedError(f'PhysX ray options are unsupported: {sorted(unsupported)}')
        starts, ends = list(rayFromPositions), list(rayToPositions)
        if len(starts) != len(ends):
            raise ValueError('Ray start and end lists must have equal length')
        if not starts:
            return []
        rays = []
        for start, end in zip(starts, ends):
            start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
            if start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all():
                raise ValueError('Ray endpoints must contain three finite coordinates')
            rays.append((start.tolist(), end.tolist()))
        hits = self.process.call('raycast', rays)
        if len(hits) != len(rays):
            raise RuntimeError('PhysX returned an incomplete ray batch')
        result = []
        for (start, end), hit in zip(rays, hits):
            if hit is None:
                result.append((-1, -1, 1.0, (0., 0., 0.), (0., 0., 0.)))
                continue
            # The pinned public API returns an opaque encoded SdfPath for
            # hits; it exposes no safe decoder into our imported Bullet IDs.
            # Preserve that real native ID rather than falsely naming body0.
            # Distance sensors consume only hit/miss and the distance fraction.
            endpoint = (int(hit['rigid_body'] or hit['collision']), -1)
            length = float(np.linalg.norm(np.asarray(end)-start))
            distance = float(hit['distance'])
            if not math.isfinite(distance) or distance < 0 or distance > length + 1e-5 or length == 0:
                raise RuntimeError('PhysX returned an invalid ray hit distance')
            result.append((*endpoint, min(1., distance / length),
                           tuple(hit['position']), tuple(hit['normal'])))
        return result

    def stepSimulation(self, **kwargs):
        # Preserve passive hinge damping using equal and opposite torques.
        for key, joint in self.joint_records.items():
            if joint.get('articulation') or joint['type'] not in ('continuous', 'revolute'):
                continue
            damping = float(joint.get('damping', 0))
            if not damping:
                continue
            _, velocity, _, _ = self.getJointState(*key)
            child = joint['child_link_index']
            axis = rotation(self._actor_pose(key[0], child)[1]) @ np.asarray(joint['axis'])
            torque = -damping * velocity * axis
            self.applyExternalTorque(key[0], child, torque, bullet.WORLD_FRAME)
            self.applyExternalTorque(key[0], joint['parent_link_index'], -torque, bullet.WORLD_FRAME)
        self._state = self.process.call('step', self.controls)
        self._invalidate_com_cache()
        self._invalidate_contact_index()
        for key, column in self.dofs.items():
            force = float(self.controls['effort'][0, column])
            error = float(self.controls['target_velocity'][0, column] - self._state['qd'][0, column])
            bound = float(self.controls['max_force'][0, column])
            force += min(bound, max(-bound, error * float(self.controls['damping'][0, column])))
            self._applied_effort[key] = force
        self.controls['wrenches'].fill(0)
        self._shadow_dirty = True

    def getCameraImage(self, *args, **kwargs):
        # Only TinyRenderer needs a Bullet pose shadow. OpenGL reads live poses.
        self.sync_render_geometry()
        return bullet.getCameraImage(*args, **kwargs)

    def sync_render_geometry(self):
        if not self._shadow_dirty:
            return
        bodies = {key[0] for key in self.actors}
        for body in bodies:
            if (body, -1) in self.actors:
                bullet.resetBasePositionAndOrientation(body, *self.getBasePositionAndOrientation(body), physicsClientId=self.client)
        for key, joint in self.joint_records.items():
            if joint['type'] != 'fixed':
                position, velocity, _, _ = self.getJointState(*key)
                bullet.resetJointState(*key, position, velocity, physicsClientId=self.client)
        self._shadow_dirty = False


class PhysXWorld(World):
    def __init__(self, config):
        self._process = None
        self._physics_directory = None
        super().__init__(config)

    def reset(self, config=None):
        self._release_physics()
        self._physics = bullet
        super().reset(config)
        from .physx_export import export_world
        cache = user_data_dir() / 'physics'
        cache.mkdir(parents=True, exist_ok=True)
        self._physics_directory = Path(tempfile.mkdtemp(prefix='scene-', dir=cache))
        values = self.config.to_dict()
        manifest = export_world(self, self._physics_directory / 'scene',
                                backend='gpu' if values['physics_backend'] == 'physx_gpu' else 'cpu',
                                iterations=values.get('physics_solver_iterations', 100))
        self._process = PhysXProcess(self._physics_directory / 'scene/field.usda', manifest,
                                     values, self._physics_directory / 'runtime')
        self._physics = PhysXAdapter(self, self._process, manifest)
        self.physics_status = self._process.initial['physics_status']
        self.tick = int(self.physics_status['initialization_steps'])
        self.time = self.tick * self.dt
        self._latency.clear()
        self._delivered = None
        self._scoring_cache = None
        self.rng = np.random.default_rng(self.config.seed)
        self._observe()
        return self.snapshot()

    def _has_contact(self, body_a: int, body_b: int) -> bool:
        if isinstance(self._physics, PhysXAdapter):
            return self._physics.has_contact(body_a, body_b)
        return super()._has_contact(body_a, body_b)

    def _deactivate_bounded_rest(self):
        # Bullet-specific rest heuristics do not mutate authoritative PhysX
        # state. PhysX's contact solver owns sleeping and contact wakeup.
        self._rest_sleep_count = 0

    def _release_physics(self):
        failed = bool(self._process is not None and self._process.failed)
        if self._process is not None:
            self._process.close()
            self._process = None
        # Keep diagnostic logs for failed starts. Successful scenes are removed
        # on close/reset; this directory was created and is owned by this World.
        directory = self._physics_directory
        if directory is not None and not failed and isinstance(getattr(self, '_physics', None), PhysXAdapter):
            root = (user_data_dir() / 'physics').resolve()
            resolved = directory.resolve()
            if resolved.parent == root and resolved.name.startswith('scene-'):
                shutil.rmtree(resolved)
        self._physics_directory = None

    def close(self):
        self._release_physics()
        self._physics = bullet
        super().close()
