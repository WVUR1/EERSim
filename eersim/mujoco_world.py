"""In-process MuJoCo authority for EERSim's shared VEX hardware model.

Bullet is an import/metadata and TinyRenderer shadow only. It never advances
physics. MuJoCo owns contacts, joint limits, gravity, motor effort and all live
state. The adapter keeps the small Bullet-shaped API used by World so PROS,
sensor noise, scoring and the passive omniwheel tire model remain shared.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np
import pybullet as bullet

from .physics import World, _cross3


def _xyzw(quaternion):
    return (float(quaternion[1]), float(quaternion[2]),
            float(quaternion[3]), float(quaternion[0]))


class MuJoCoAdapter:
    def __init__(self, world, scene):
        self.world, self.scene = world, scene
        self.model, self.data = scene.model, scene.data
        self.client = world.client
        self.body_ids, self.joint_ids = scene.body_ids, scene.joint_ids
        self.actuator_ids = scene.actuator_ids
        self._joint_meta = {key: bullet.getJointInfo(*key, physicsClientId=self.client)
                            for key in self.joint_ids}
        self._geom_body = np.full(self.model.ngeom, -1, dtype=np.int32)
        self._geom_link = np.full(self.model.ngeom, -1, dtype=np.int32)
        for geom, (body, link) in scene.geom_keys.items():
            self._geom_body[geom], self._geom_link[geom] = body, link
        self._flex_keys = getattr(scene, 'flex_keys', {})
        self._geom_material = np.zeros((self.model.ngeom, 4))
        self._flex_material = np.zeros((self.model.nflex, 4))
        for mapping, materials in ((scene.geom_keys, self._geom_material),
                                   (self._flex_keys, self._flex_material)):
            for native, key in mapping.items():
                dynamics = scene.dynamics[key]
                materials[native] = (dynamics[1], dynamics[7], dynamics[6], dynamics[5])
        self._applied_effort = {key: 0.0 for key in self.joint_ids}
        # Match EERSim's PhysX per-second body damping rates. These are explicit
        # dissipative COM forces and inertia-weighted torques, not velocity
        # resets or Bullet's different nonlinear damping formula.
        self._body_drag = [(native, .02 if body == world.robot else .04)
                           for (body, _), native in self.body_ids.items()
                           if self.model.body_mass[native] > 0 and self.model.body_weldid[native] != 0]
        self._shadow_dirty = True
        # Presence only: native sleeping contacts need not remain in mjData.
        # Never return these records from getContactPoints or reuse their forces.
        self._sleep_contacts = {}
        # Only world-welded rigid flexes can retain their initialized vertices,
        # edge data and BVH. A moving/mocap/deformable flex uses the stock stage.
        flex_bodies = self.model.flex_vertbodyid
        self._static_flex = bool(self.model.nflex and np.all(self.model.flex_rigid)
                                 and not self.model.nmocap
                                 and np.all(self.model.body_weldid[flex_bodies] == 0))
        self._cached_position_stage = self._static_flex and mujoco.__version__ == '3.17.0'
        self._invalidate_state_cache()
        self._apply_contact_materials()
        mujoco.mj_fwdConstraint(self.model, self.data)
        self._check_warnings()

    def __getattr__(self, name):
        static = {'getJointInfo', 'getNumJoints', 'getBodyInfo', 'getVisualShapeData',
                  'getCollisionShapeData', 'getBodyUniqueId', 'getNumBodies',
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
            raise NotImplementedError(f'MuJoCo runtime API does not implement {name}; the Bullet geometry shadow is read-only')
        raise AttributeError(name)

    def getDynamicsInfo(self, bodyUniqueId, linkIndex=-1, **kwargs):
        return self.scene.dynamics[(bodyUniqueId, linkIndex)]

    def _invalidate_state_cache(self):
        self._body_motion = {}
        self._com_quaternions = {}
        self._contact_keys = None
        self._contact_records = {}

    def _com_quaternion(self, body_id):
        if body_id not in self._com_quaternions:
            quaternion = np.empty(4)
            mujoco.mju_mulQuat(quaternion, self.data.xquat[body_id], self.model.body_iquat[body_id])
            self._com_quaternions[body_id] = _xyzw(quaternion)
        return self._com_quaternions[body_id]

    def _motion(self, body_id):
        if body_id not in self._body_motion:
            velocity = np.empty(6)
            # mjOBJ_BODY is the inertial/COM frame; mjOBJ_XBODY would be the
            # URDF link origin. World expects velocity at the COM in world axes.
            mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                     body_id, velocity, 0)
            self._body_motion[body_id] = (velocity[3:].copy(), velocity[:3].copy())
        return self._body_motion[body_id]

    def getBasePositionAndOrientation(self, bodyUniqueId, **kwargs):
        body = self.body_ids[(bodyUniqueId, -1)]
        return tuple(self.data.xipos[body]), self._com_quaternion(body)

    def getBaseVelocity(self, bodyUniqueId, **kwargs):
        return self._motion(self.body_ids[(bodyUniqueId, -1)])

    def getLinkState(self, bodyUniqueId, linkIndex, **kwargs):
        key = (bodyUniqueId, linkIndex)
        body = self.body_ids[key]
        dynamics = self.scene.dynamics[key]
        linear, angular = self._motion(body)
        return (tuple(self.data.xipos[body]), self._com_quaternion(body), dynamics[3], dynamics[4],
                tuple(self.data.xpos[body]), _xyzw(self.data.xquat[body]), linear, angular)

    def getJointState(self, bodyUniqueId, jointIndex, **kwargs):
        key = (bodyUniqueId, jointIndex)
        joint = self.joint_ids.get(key)
        if joint is None:
            if key not in self.body_ids:
                raise ValueError(f'Unknown joint: {key}')
            return (0.0, 0.0, (0.0,) * 6, 0.0)
        return (float(self.data.qpos[self.model.jnt_qposadr[joint]]),
                float(self.data.qvel[self.model.jnt_dofadr[joint]]),
                (0.0,) * 6, self._applied_effort[key])

    def getJointStates(self, bodyUniqueId, jointIndices, **kwargs):
        return [self.getJointState(bodyUniqueId, joint, **kwargs) for joint in jointIndices]

    def setJointMotorControl2(self, bodyUniqueId, jointIndex, controlMode, **kwargs):
        key = (bodyUniqueId, jointIndex)
        if key not in self.actuator_ids:
            raise ValueError(f'Joint has no MuJoCo actuator: {key}')
        actuator = self.actuator_ids[key]
        bound = abs(float(kwargs.get('force', 0)))
        if not math.isfinite(bound):
            raise ValueError('Motor force must be finite')
        self.model.actuator_gainprm[actuator].fill(0)
        self.model.actuator_biasprm[actuator].fill(0)
        self.model.actuator_forcerange[actuator] = (-bound, bound)
        if controlMode == bullet.TORQUE_CONTROL:
            self.model.actuator_gainprm[actuator, 0] = 1
            self.data.ctrl[actuator] = float(kwargs.get('force', 0))
        elif controlMode in (bullet.VELOCITY_CONTROL, bullet.POSITION_CONTROL):
            target = float(kwargs.get('targetVelocity', 0))
            if controlMode == bullet.POSITION_CONTROL:
                position = self.getJointState(*key)[0]
                target += float(kwargs.get('positionGain', .1)) * (float(kwargs.get('targetPosition', 0)) - position) / self.world.dt
                limit = abs(float(kwargs.get('maxVelocity', self._joint_meta[key][11] or 10)))
                target = min(limit, max(-limit, target))
            if not math.isfinite(target):
                raise ValueError('Motor target must be finite')
            # Native affine velocity feedback is integrated implicitly. Its
            # effort is clamped to the shared VEX cartridge/current envelope;
            # no body or joint velocity is prescribed by this controller.
            dof = self.model.jnt_dofadr[self.joint_ids[key]]
            # Use stiff implicit feedback, so passive damping does not create
            # artificial speed droop on light rotors. MuJoCo drops the actuator
            # derivative when force clips; cap the gain by this tick's available
            # torque/error so that derivative remains active. Merely setting a
            # large fixed gain with a force limit can overshoot on light joints.
            gain = 1000.0 if bound else 0.0
            velocity = float(self.data.qvel[dof])
            if abs(target - velocity) > 1e-12:
                gain = min(gain, bound * (1 - 1e-6) / abs(target - velocity))
            self.model.actuator_gainprm[actuator, 0] = gain
            self.model.actuator_biasprm[actuator, 2] = -gain
            self.data.ctrl[actuator] = target
        else:
            raise ValueError(f'Unsupported MuJoCo control mode: {controlMode}')

    def applyExternalForce(self, objectUniqueId, linkIndex, forceObj, posObj, flags, **kwargs):
        body = self.body_ids[(objectUniqueId, linkIndex)]
        force, point = np.asarray(forceObj, dtype=float), np.asarray(posObj, dtype=float)
        centre = self.data.xipos[body]
        if flags == bullet.LINK_FRAME:
            matrix = self.data.ximat[body].reshape(3, 3)
            force = matrix @ force
            point = centre + matrix @ point
        elif flags != bullet.WORLD_FRAME:
            raise ValueError('External force frame must be LINK_FRAME or WORLD_FRAME')
        self.data.xfrc_applied[body, :3] += force
        self.data.xfrc_applied[body, 3:] += _cross3(point - centre, force)

    def applyExternalTorque(self, objectUniqueId, linkIndex, torqueObj, flags, **kwargs):
        body = self.body_ids[(objectUniqueId, linkIndex)]
        torque = np.asarray(torqueObj, dtype=float)
        if flags == bullet.LINK_FRAME:
            torque = self.data.ximat[body].reshape(3, 3) @ torque
        elif flags != bullet.WORLD_FRAME:
            raise ValueError('External torque frame must be LINK_FRAME or WORLD_FRAME')
        self.data.xfrc_applied[body, 3:] += torque

    def _index_contacts(self):
        if self._contact_keys is not None:
            return
        geoms = self.data.contact.geom[:self.data.ncon]
        keys = np.empty((len(geoms), 4), dtype=np.int32)
        for side in (0, 1):
            ids = geoms[:, side]
            keys[:, side] = self._geom_body[np.maximum(ids, 0)]
            keys[:, side + 2] = self._geom_link[np.maximum(ids, 0)]
            for row in np.flatnonzero(ids < 0):
                flex = int(self.data.contact.flex[row, side])
                key = self._flex_keys.get(flex, (-1, -1))
                keys[row, side], keys[row, side + 2] = key
        self._contact_keys = keys

    def has_contact(self, body_a, body_b):
        self._index_contacts()
        keys = self._contact_keys
        if np.any(((keys[:, 0] == body_a) & (keys[:, 1] == body_b)) |
                  ((keys[:, 1] == body_a) & (keys[:, 0] == body_b))):
            return True
        pair = tuple(sorted((body_a, body_b)))
        return any(self._sleep_contact_valid(native, pose)
                   for native, pose in self._sleep_contacts.get(pair, {}).items())

    def _sleep_contact_valid(self, bodies, pose=None):
        if not self.model.opt.enableflags & mujoco.mjtEnableBit.mjENBL_SLEEP:
            return False
        trees = self.model.body_treeid[list(bodies)]
        dynamic = trees >= 0
        if not np.any(dynamic) or np.any(self.data.tree_awake[trees[dynamic]]):
            return False
        if pose is None:
            return True
        return (np.array_equal(self.data.xpos[list(bodies)], pose[0])
                and np.array_equal(self.data.xquat[list(bodies)], pose[1]))

    def _remember_sleeping_contacts(self):
        retained = {}
        for pair, records in self._sleep_contacts.items():
            valid = {native: pose for native, pose in records.items()
                     if self._sleep_contact_valid(native, pose)}
            if valid:
                retained[pair] = valid
        self._sleep_contacts = retained
        if self.data.ntree_awake == self.model.ntree:
            return
        self._index_contacts()
        for body_a, body_b, link_a, link_b in np.unique(self._contact_keys, axis=0):
            native = (self.body_ids[(int(body_a), int(link_a))],
                      self.body_ids[(int(body_b), int(link_b))])
            if self._sleep_contact_valid(native):
                pair = tuple(sorted((int(body_a), int(body_b))))
                self._sleep_contacts.setdefault(pair, {})[native] = (
                    self.data.xpos[list(native)].copy(), self.data.xquat[list(native)].copy())

    def sleep_status(self):
        enabled = bool(self.model.opt.enableflags & mujoco.mjtEnableBit.mjENBL_SLEEP)
        awake = int(np.count_nonzero(self.data.tree_awake)) if enabled else int(self.model.ntree)
        return {'enabled': enabled, 'awake_trees': awake,
                'sleeping_trees': int(self.model.ntree) - awake,
                'contact_count': int(self.data.ncon)}

    def _contact_record(self, row, reverse=False):
        key = (row, reverse)
        if key in self._contact_records:
            return self._contact_records[key]
        contact = self.data.contact[row]
        body_a, body_b, link_a, link_b = map(int, self._contact_keys[row])
        frame = contact.frame.reshape(3, 3)
        force = np.empty(6)
        mujoco.mj_contactForce(self.model, self.data, row, force)
        # MuJoCo normal points geom0 -> geom1; Bullet exposes normal on B,
        # pointing B -> A. MuJoCo stores the midpoint of the two surfaces.
        normal = -frame[0]
        point_a = contact.pos - .5 * contact.dist * frame[0]
        point_b = contact.pos + .5 * contact.dist * frame[0]
        tangents = -frame[1:]
        if reverse:
            body_a, body_b, link_a, link_b = body_b, body_a, link_b, link_a
            point_a, point_b = point_b, point_a
            normal, tangents = -normal, -tangents
        result = (0, body_a, body_b, link_a, link_b, tuple(point_a), tuple(point_b),
                  tuple(normal), float(contact.dist), max(0., float(force[0])),
                  float(force[1]), tuple(tangents[0]), float(force[2]), tuple(tangents[1]))
        self._contact_records[key] = result
        return result

    def getContactPoints(self, bodyA=-1, bodyB=-1, linkIndexA=-2, linkIndexB=-2, **kwargs):
        self._index_contacts()
        keys = self._contact_keys
        forward = np.ones(len(keys), dtype=bool)
        reverse = np.ones(len(keys), dtype=bool)
        for value, column in ((bodyA, 0), (bodyB, 1), (linkIndexA, 2), (linkIndexB, 3)):
            if (column < 2 and value >= 0) or (column >= 2 and value != -2):
                forward &= keys[:, column] == value
                reverse &= keys[:, column ^ 1] == value
        return [self._contact_record(int(row), not forward[row])
                for row in np.flatnonzero(forward | reverse)]

    def rayTest(self, rayFromPosition, rayToPosition, **kwargs):
        return self.rayTestBatch([rayFromPosition], [rayToPosition], **kwargs)

    def rayTestBatch(self, rayFromPositions, rayToPositions, **kwargs):
        unsupported = set(kwargs) - {'physicsClientId', 'numThreads'}
        if unsupported:
            raise NotImplementedError(f'MuJoCo ray options are unsupported: {sorted(unsupported)}')
        starts, ends = list(rayFromPositions), list(rayToPositions)
        if len(starts) != len(ends):
            raise ValueError('Ray start and end lists must have equal length')
        results = []
        for start, end in zip(starts, ends):
            start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
            if start.shape != (3,) or end.shape != (3,) or not np.isfinite([start, end]).all():
                raise ValueError('Ray endpoints must contain three finite coordinates')
            length = float(np.linalg.norm(end - start))
            miss = (-1, -1, 1.0, (0., 0., 0.), (0., 0., 0.))
            if length == 0:
                results.append(miss)
                continue
            direction = (end - start) / length
            geom = np.full(1, -1, dtype=np.int32)
            normal = np.zeros(3)
            distance = mujoco.mj_ray(self.model, self.data, start, direction,
                                     None, True, -1, geom, normal)
            source = self.scene.geom_keys.get(int(geom[0]), (-1, -1))
            # Rigid flex triangles preserve concave field meshes. Native geom
            # rays do not include them, so query those surfaces explicitly.
            for flex, flex_source in self._flex_keys.items():
                flex_normal = np.zeros(3)
                value = mujoco.mj_rayFlex(self.model, self.data, 0, False, False, True,
                                         False, flex, start, direction, None, flex_normal)
                if value >= 0 and (distance < 0 or value < distance):
                    distance, normal, source = value, flex_normal, flex_source
            if distance < 0 or distance > length or source[0] < 0:
                results.append(miss)
            else:
                results.append((*source, float(distance / length), tuple(start + distance * direction), tuple(normal)))
        return results

    def _apply_contact_materials(self, rebuild=True):
        """Keep EERSim's configured Bullet material-combination convention.

        MuJoCo normally takes the larger coefficient. Bullet multiplies sliding
        friction and sums rolling/spinning times the opposing sliding friction.
        Native contacts are edited before constraint assembly, not after solving.
        """
        contacts = self.data.contact
        count = self.data.ncon
        if not count:
            return
        sides = []
        for side in (0, 1):
            geom = contacts.geom[:count, side]
            material = self._geom_material[np.maximum(geom, 0)].copy()
            flex_rows = np.flatnonzero(geom < 0)
            if len(flex_rows):
                material[flex_rows] = self._flex_material[contacts.flex[flex_rows, side]]
            sides.append(material)
        a, b = sides
        sliding = np.clip(a[:, 0] * b[:, 0], 0, 10)
        spinning = np.clip(a[:, 1] * b[:, 0] + b[:, 1] * a[:, 0], 0, 10)
        rolling = np.clip(a[:, 2] * b[:, 0] + b[:, 2] * a[:, 0], 0, 10)
        contacts.friction[:count] = np.column_stack((sliding, sliding, spinning, rolling, rolling))
        contacts.dim[:count] = np.where(rolling > 0, 6, np.where(spinning > 0, 4, np.where(sliding > 0, 3, 1)))
        restitution = np.clip(a[:, 3] * b[:, 3], 0, 1)
        damping = np.ones(count)
        elastic = restitution > 0
        log_restitution = np.log(restitution[elastic])
        damping[elastic] = np.maximum(.0001, -log_restitution / np.sqrt(math.pi ** 2 + log_restitution ** 2))
        # Keep contact stiffness constant while reducing damping. Holding the
        # positive-format time constant fixed would scale stiffness by 1/zeta^2
        # and inject energy for high-restitution impacts at coarse timesteps.
        contacts.solref[:count, 0] = max(.005, 2 * self.world.dt) / damping
        contacts.solref[:count, 1] = damping
        # step1 has built constraints using native max-mixing. Rebuild only
        # constraints after updating material values; collision/inertia and
        # actuation are not repeated. step2 retains implicitfast integration.
        if rebuild:
            mujoco.mj_makeConstraint(self.model, self.data)
            mujoco.mj_island(self.model, self.data)
            mujoco.mj_projectConstraint(self.model, self.data)
            mujoco.mj_referenceConstraint(self.model, self.data)

    def _can_cache_position_stage(self):
        # This is an audited public-API decomposition of MuJoCo 3.17's
        # mj_fwdPosition/mj_step1, not a replacement collision or physics solver:
        # https://github.com/google-deepmind/mujoco/blob/3.17.0/src/engine/engine_forward.c
        # The internal sleep-wake and discrete effective-metric stages cannot be
        # reproduced with public Python APIs, so those modes must use mj_step1.
        return (self._cached_position_stage
                and self.model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
                and not self.model.opt.enableflags
                and not self.model.nplugin
                and mujoco.get_mjcb_control() is None)

    def _step1_cached_static_flex(self):
        model, data = self.model, self.data
        mujoco.mj_checkPos(model, data)
        mujoco.mj_checkVel(model, data)
        data.flg_energypos = 0
        with self.world.performance.span('world.mujoco.kinematics'):
            mujoco.mj_kinematics(model, data)
            mujoco.mj_comPos(model, data)
            mujoco.mj_camlight(model, data)
            # mj_flex is invariant here: every vertex is welded to the world.
            mujoco.mj_tendon(model, data)
        with self.world.performance.span('world.mujoco.inertia'):
            mujoco.mj_makeM(model, data)
            mujoco.mj_factorM(model, data)
        with self.world.performance.span('world.mujoco.collision'):
            mujoco.mj_collision(model, data)
        with self.world.performance.span('world.mujoco.materials'):
            self._apply_contact_materials(rebuild=False)
        with self.world.performance.span('world.mujoco.constraints'):
            mujoco.mj_makeConstraint(model, data)
            mujoco.mj_island(model, data)
            mujoco.mj_projectConstraint(model, data)
        mujoco.mj_transmission(model, data)
        # The implicitfast integrator has no discrete effective metric. Its
        # internal mj_effBuild call is inactive; the native step2 remains intact.
        mujoco.mj_sensorPos(model, data)
        data.energy[:] = 0
        mujoco.mj_fwdVelocity(model, data)
        mujoco.mj_sensorVel(model, data)

    def _apply_body_drag(self):
        for body, rate in self._body_drag:
            tree = self.model.body_treeid[body]
            if tree >= 0 and not self.data.tree_awake[tree]:
                continue
            linear, angular = self._motion(body)
            rotation = self.data.ximat[body].reshape(3, 3)
            angular_momentum = rotation @ (self.model.body_inertia[body] * (rotation.T @ angular))
            # Passive drag must not masquerade as an external perturbation:
            # any nonzero xfrc_applied prevents native sleep. Add the same
            # generalized COM wrench after step1's passive-force computation.
            mujoco.mj_applyFT(self.model, self.data, -rate * self.model.body_mass[body] * linear,
                              -rate * angular_momentum, self.data.xipos[body], body,
                              self.data.qfrc_passive)

    def _check_warnings(self):
        counts = self.data.warning.number
        if np.any(counts):
            names = [f'{mujoco.mjtWarning(index).name} ({int(count)})'
                     for index, count in enumerate(counts) if count]
            raise RuntimeError('MuJoCo stopped after native warning(s): ' + ', '.join(names)
                               + '. Inspect overlapping collision geometry, joint inertia and timestep; '
                                 'numerical resets or truncated contacts cannot be treated as a valid run.')

    def stepSimulation(self, **kwargs):
        self._check_warnings()
        expected_time = float(self.data.time) + self.world.dt
        if (not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all()
                or np.any(np.abs(self.data.qpos) >= 1e10) or np.any(np.abs(self.data.qvel) >= 1e10)):
            raise RuntimeError('MuJoCo encountered unstable/nonfinite state before integration')
        with self.world.performance.span('world.mujoco.step1'):
            if self._can_cache_position_stage():
                self._step1_cached_static_flex()
            else:
                mujoco.mj_step1(self.model, self.data)
                with self.world.performance.span('world.mujoco.materials'):
                    self._apply_contact_materials()
        with self.world.performance.span('world.mujoco.body_drag'):
            self._apply_body_drag()
        awake_before = self.data.tree_awake.copy()
        with self.world.performance.span('world.mujoco.step2'):
            mujoco.mj_step2(self.model, self.data)
        if np.any(awake_before & ~self.data.tree_awake):
            # Native advanceStart recomputes velocity dynamics when a tree
            # first sleeps. implicitfast has already copied the acceleration
            # used to integrate, but that recomputation overwrites our passive
            # wrench in the force/acceleration feedback. Restore it before
            # observing contacts or warmstarting the next tick. Derived poses
            # still belong to this tick's solve; there is no extra integration.
            self._apply_body_drag()
            mujoco.mj_fwdAcceleration(self.model, self.data)
            mujoco.mj_fwdConstraint(self.model, self.data)
            self.data.flg_rnepost = 0
            mujoco.mj_sensorAcc(self.model, self.data)
            mujoco.mj_checkAcc(self.model, self.data)
            self.data.qacc_warmstart[:] = self.data.qacc
        self._check_warnings()
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all() or abs(self.data.time - expected_time) > 1e-8:
            raise RuntimeError('MuJoCo encountered unstable/nonfinite dynamics; reduce the timestep and inspect overlapping geometry or joint inertia')
        for key, actuator in self.actuator_ids.items():
            self._applied_effort[key] = float(self.data.actuator_force[actuator])
        self.data.xfrc_applied.fill(0)
        self.data.qfrc_applied.fill(0)
        # mj_step integrates qpos/qvel after its dynamics solve. Refresh only
        # derived poses/velocities, preserving solved contacts/forces and avoiding
        # a second collision/constraint solve just to render the completed tick.
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        mujoco.mj_comVel(self.model, self.data)
        if self.model.nflex and not self._static_flex:
            mujoco.mj_flex(self.model, self.data)
        self._invalidate_state_cache()
        self._remember_sleeping_contacts()
        self.world.physics_status['sleeping'] = self.sleep_status()
        self._shadow_dirty = True

    def getCameraImage(self, *args, **kwargs):
        self.sync_render_geometry()
        return bullet.getCameraImage(*args, **kwargs)

    def sync_render_geometry(self):
        if not self._shadow_dirty:
            return
        for (body, link), native in self.body_ids.items():
            if link == -1 and body in self.scene.free_joints:
                bullet.resetBasePositionAndOrientation(body, *self.getBasePositionAndOrientation(body),
                                                       physicsClientId=self.client)
        for key in self.joint_ids:
            position, velocity, _, _ = self.getJointState(*key)
            bullet.resetJointState(*key, position, velocity, physicsClientId=self.client)
        self._shadow_dirty = False


class MuJoCoWorld(World):
    def _release_native_workers(self):
        adapter = getattr(self, '_physics', None)
        if isinstance(adapter, MuJoCoAdapter) and adapter.data.threadpool:
            # Join now, even if a debugger/inspector still holds an old snapshot
            # of the adapter; ordinary mjData destruction is the final fallback.
            mujoco.mju_threadpool(adapter.data, 0)

    def reset(self, config=None):
        self._release_native_workers()
        self._physics = bullet
        super().reset(config)
        from .mujoco_scene import compile_scene
        with self.performance.span('world.mujoco_compile'):
            scene = compile_scene(self)
        self._physics = MuJoCoAdapter(self, scene)
        self.physics_status = {
            'backend': 'mujoco', 'engine': 'MuJoCo', 'version': mujoco.__version__,
            'device': 'CPU', 'hardware_accelerated': False,
            'solver': 'Newton', 'integrator': 'implicitfast',
            'solver_iterations': int(scene.model.opt.iterations),
            'cpu_threads': int(scene.cpu_threads),
            'cpu_threads_requested': int(self.config.physics_cpu_threads),
            'initialization_steps': 0,
            'material_combine': 'Bullet-compatible product sliding friction; cross-product sum spinning/rolling',
            'contact_model': 'MuJoCo compliant contacts; material-product restitution maps to an approximate damping ratio',
            'body_damping_model': 'Dissipative COM force/inertia torque: robot 0.02/s, props 0.04/s; PhysX rate semantics, not Bullet nonlinear damping',
            'static_flex_cache': self._physics._can_cache_position_stage(),
            'sleeping': self._physics.sleep_status(),
            'scene_query_identity': 'Native MuJoCo geom/flex identities mapped to imported body/link IDs',
        }
        self._latency.clear()
        self._delivered = None
        self._scoring_cache = None
        self.rng = np.random.default_rng(self.config.seed)
        self._observe()
        return self.snapshot()

    def _has_contact(self, body_a, body_b):
        if isinstance(self._physics, MuJoCoAdapter):
            return self._physics.has_contact(body_a, body_b)
        return super()._has_contact(body_a, body_b)

    def _deactivate_bounded_rest(self):
        # Engine contacts/constraints own the authoritative dynamics. Never
        # mutate a Bullet shadow to impose artificial rest on MuJoCo objects.
        self._rest_sleep_count = 0

    def close(self):
        self._release_native_workers()
        self._physics = bullet
        super().close()
