"""Fixed-step rigid-body world and hardware observations.

Bullet integrates gravity, joint inertia/limits and collision impulses. The sample
X-drive uses frictionless wheel contacts plus a passive, regularized Coulomb tire
model: slip velocity at each *actual* contact generates a capped contact force.
The force is applied to the wheel, so its reaction torque loads the motor. No
chassis pose/velocity is prescribed. This approximates free omni rollers without
hundreds of tiny roller contacts; it is a model to calibrate, not a digital twin.

World coordinates are right-handed: +X forward, +Y left, +Z up. The bundled
OTOS mapping uses +X forward, +Y right and CCW heading; EERKit negates that
heading on receipt. IMU heading follows PROS' clockwise convention.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import pybullet as p

from .performance import PerformanceRecorder

from .config import SimConfig
from .motor import cartridge_for, pwm_no_load_speed, pwm_torque, torque_fraction


def _clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _cross3(a, b) -> np.ndarray:
    # Fixed-size vectors avoid NumPy's general broadcasting/axis machinery in
    # the per-contact loop while retaining the same cross-product calculation.
    return np.array((a[1] * b[2] - a[2] * b[1],
                     a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]))


def _vec(value: Any, count: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{name} must contain {count} numbers")
    result = [float(x) for x in value]
    if not all(math.isfinite(x) for x in result):
        raise ValueError(f"{name} must contain finite numbers")
    return result


def _read_json(path: str | Path) -> dict[str, Any]:
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a JSON object")
    if data.get("schema_version", 1) != 1:
        raise ValueError(f"Unsupported schema_version in {path}")
    return data


class World:
    """One independently connected, deterministic Bullet DIRECT world.

    Calls must be serialized by the owner (typically the simulation thread).
    ``step`` always advances 1/physics_hz seconds, irrespective of render FPS.
    Commands contain physical positive-port values: the host adapter applies
    MotorGroup reversal; bindings convert physical shafts into joint axes.
    """

    def __init__(self, config: dict[str, Any] | SimConfig | None = None):
        self.performance = PerformanceRecorder()
        self._physics = p
        self.client = self._physics.connect(self._physics.DIRECT)
        if self.client < 0:
            raise RuntimeError("Could not create a PyBullet physics connection")
        try:
            with self.performance.span("world.setup"):
                self.reset(config)
        except Exception:
            self.close()
            raise

    def reset(self, config: dict[str, Any] | SimConfig | None = None) -> dict[str, Any]:
        if getattr(self, "_render_backend", None) is not None:
            self._render_backend.close()
        self._render_backend = None
        self.renderer_status = {"backend": "pending", "device": "Graphics initialization pending", "hardware_accelerated": False}
        self.config = SimConfig.from_dict(config if config is not None else getattr(self, "config", None))
        self.dt = 1.0 / self.config.physics_hz
        self.time = 0.0
        self.tick = 0
        self.rng = np.random.default_rng(self.config.seed)
        self._latency = deque()
        self._delivered: dict[str, Any] | None = None
        self._last_yaw = math.radians(self.config.start_pose[3])
        self._rotation = self._last_yaw
        self._commands: dict[str, dict[str, Any]] = {"motors": {}, "adi": {}, "joints": {}}
        self._motor_integrals: dict[int, float] = {}
        self._control_duties: dict[int, float] = {}
        self._next_control_time: dict[int, float] = {}
        self._motor_modes: dict[int, str] = {}
        self._servo_targets: dict[int, float] = {}
        self._servo_loads: dict[int, list[tuple[dict[str, Any], float]]] = {}
        self._hold_targets: dict[int, float] = {}
        self._last_torques: dict[int, float] = {}
        self._diagnostics: list[str] = []
        self._gear_mismatches: set[tuple[int, int]] = set()
        self._scoring_cache: dict[str, Any] | None = None
        c = self.client
        self._physics.resetSimulation(physicsClientId=c)
        self._physics.setGravity(0, 0, -self.config.gravity, physicsClientId=c)
        self._physics.setTimeStep(self.dt, physicsClientId=c)
        self._physics.setRealTimeSimulation(0, physicsClientId=c)
        iterations = int(getattr(self.config, "physics_solver_iterations", 100))
        self._physics.setPhysicsEngineParameter(
            fixedTimeStep=self.dt, numSolverIterations=iterations, numSubSteps=0,
            deterministicOverlappingPairs=1, enableConeFriction=1,
            # A 1 mm cache threshold in Bullet 3.25 repeatedly rebuilds rolling
            # contacts and injects energy in a friction-only sphere fixture.
            # 2 mm retains those contacts without changing collision geometry.
            contactBreakingThreshold=0.002, physicsClientId=c,
        )
        fallback_reason = getattr(self, "physics_status", {}).get("reason", "")
        self.physics_status = {"backend": "bullet", "engine": "PyBullet", "device": "CPU",
                               "hardware_accelerated": False, "solver_iterations": iterations}
        if self.config.physics_backend == "auto" and fallback_reason:
            self.physics_status["reason"] = fallback_reason
        self._export_items: list[tuple[int, dict[str, Any], Path]] = []
        self.field_objects: list[dict[str, Any]] = []
        self.base_inertial_frames: dict[int, tuple] = {}
        self._rest_bodies: dict[int, dict[str, Any]] = {}
        self._dynamic_field_bodies: set[int] = set()
        self._rest_sleep_count = 0
        self.game = _read_json(self.config.game_pack)
        self._load_field(Path(self.config.game_pack).resolve().parent)
        robot_path = Path(self.config.robot_urdf).expanduser().resolve()
        if not robot_path.is_file():
            raise ValueError(f"Robot URDF does not exist: {robot_path}")
        self.robot_settings = _read_json(self.config.robot_config) if self.config.robot_config else {}
        pose = self.config.start_pose
        self.robot = self._physics.loadURDF(
            str(robot_path), basePosition=pose[:3],
            baseOrientation=self._physics.getQuaternionFromEuler([0, 0, math.radians(pose[3])]),
            useFixedBase=bool(self.config.extra.get("fixed_base", self.robot_settings.get("fixed_base", False))),
            flags=self._physics.URDF_USE_INERTIA_FROM_FILE | self._physics.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
            physicsClientId=c,
        )
        self._joint_info: dict[str, tuple] = {}
        self._link_indices = {"base_link": -1}
        for idx in range(self._physics.getNumJoints(self.robot, physicsClientId=c)):
            info = self._physics.getJointInfo(self.robot, idx, physicsClientId=c)
            self._joint_info[info[1].decode()] = info
            self._link_indices[info[12].decode()] = idx
            if info[2] in (self._physics.JOINT_REVOLUTE, self._physics.JOINT_PRISMATIC):
                self._physics.setJointMotorControl2(self.robot, idx, self._physics.VELOCITY_CONTROL, force=0, physicsClientId=c)
        for idx in range(-1, self._physics.getNumJoints(self.robot, physicsClientId=c)):
            self._physics.changeDynamics(
                self.robot, idx, lateralFriction=self.config.friction,
                restitution=self.config.restitution, linearDamping=0.02,
                angularDamping=0.02, jointDamping=0.001,
                physicsClientId=c,
            )
        for name, friction in self.robot_settings.get("link_friction", {}).items():
            if name not in self._link_indices or not 0 <= float(friction) <= 5:
                raise ValueError(f"Invalid link_friction entry for '{name}'")
            self._physics.changeDynamics(self.robot, self._link_indices[name], lateralFriction=float(friction), physicsClientId=c)
        self.motor_bindings = self._load_motors()
        self._wheels = self.robot_settings.get("drive", {}).get("wheels", [])
        for wheel in self._wheels:
            info = self._require_joint(wheel["joint"])
            wheel["index"] = info[0]
            dynamics = self._physics.getDynamicsInfo(self.robot, info[0], physicsClientId=c)
            # Conservative local wheel inertia bounds the explicit tire impulse;
            # it prevents a 240 Hz contact force from reversing slip in one tick.
            wheel["inertia"] = max(1e-7, min(dynamics[2]))
            if self.robot_settings.get("drive", {}).get("type") == "omni":
                direction = np.array(_vec(wheel["direction"], 3, "wheel.direction"))
                if np.linalg.norm(direction) < 0.001:
                    raise ValueError("Wheel direction cannot be zero")
                wheel["direction"] = (direction / np.linalg.norm(direction)).tolist()
                self._physics.changeDynamics(self.robot, info[0], lateralFriction=0,
                                 rollingFriction=0, spinningFriction=0, jointDamping=0.0001, physicsClientId=c)
        self._pneumatics = self.robot_settings.get("pneumatics", [])
        for piston in self._pneumatics:
            piston["index"] = self._require_joint(piston["joint"])[0]
        for name, position in self.robot_settings.get("initial_joints", {}).items():
            self._physics.resetJointState(self.robot, self._require_joint(name)[0], float(position), physicsClientId=c)
        self._physics.performCollisionDetection(physicsClientId=c)
        self._observe()
        return self.snapshot()

    def _require_joint(self, name: str) -> tuple:
        if name not in self._joint_info:
            raise ValueError(f"Binding refers to missing URDF joint '{name}'")
        info = self._joint_info[name]
        if info[2] not in (self._physics.JOINT_REVOLUTE, self._physics.JOINT_PRISMATIC):
            raise ValueError(f"Joint '{name}' must be revolute, continuous or prismatic")
        return info

    def _load_motors(self) -> list[dict[str, Any]]:
        result, seen = [], set()
        source = self.config.motors if self.config.motors is not None else self.robot_settings.get("motors", [])
        for raw in source:
            motor = deepcopy(raw)
            signed = int(motor["port"])
            port = abs(signed)
            if port < 1 or port > 21 or port in seen:
                raise ValueError(f"Each motor must have a unique smart port from 1 to 21 (got {port})")
            seen.add(port)
            motor.update(self.config.motor_overrides.get(str(port), {}))
            info = self._require_joint(motor["joint"])
            motor["port"] = port
            motor["index"] = info[0]
            motor["reversed"] = bool(motor.get("reversed", signed < 0))
            motor["sign"] = -1 if motor["reversed"] else 1
            preset = cartridge_for(motor.get("cartridge", motor.get("rpm", 200)))
            motor["cartridge"] = preset.name
            motor["rpm"] = float(preset.rpm)
            motor["internal_ratio"] = preset.internal_ratio
            motor["ticks_per_rev"] = preset.ticks_per_rev
            motor["rated_power"] = preset.rated_power
            motor["stall_torque"] = float(motor.get("stall_torque", preset.stall_torque))
            motor["gear_ratio"] = float(motor.get("gear_ratio", 1))
            motor["max_power"] = float(motor.get("max_power", preset.peak_power))
            motor["efficiency"] = float(motor.get("efficiency", 1.0))
            motor["current_limit"] = float(motor.get("current_limit", preset.current_limit))
            motor["controller_period"] = preset.controller_period
            if not all(math.isfinite(motor[k]) and motor[k] > 0 for k in ("rpm", "stall_torque", "gear_ratio", "max_power", "efficiency")):
                raise ValueError(f"Motor {port}: speed, torque, ratio, efficiency and power must be positive")
            if motor["efficiency"] > 1 or not 0 <= motor["current_limit"] <= 2.5:
                raise ValueError(f"Motor {port}: external efficiency cannot exceed 1; current limit must be 0–2.5 A")
            motor["factor"] = motor["gear_ratio"]
            if info[2] == self._physics.JOINT_PRISMATIC:
                lead = float(motor.get("lead_m", 0.01))
                if not math.isfinite(lead) or lead <= 0:
                    raise ValueError("A prismatic motor lead_m must be positive metres/revolution")
                motor["factor"] *= 2 * math.pi / lead
            result.append(motor)
        return result

    def _shape(self, item: dict[str, Any], visual: bool = False) -> int:
        shape = item.get("shape", "box")
        geometry = {"box": self._physics.GEOM_BOX, "sphere": self._physics.GEOM_SPHERE,
                    "cylinder": self._physics.GEOM_CYLINDER, "capsule": self._physics.GEOM_CAPSULE}
        if shape not in geometry:
            raise ValueError(f"Unknown game object shape '{shape}'")
        kwargs: dict[str, Any] = {"shapeType": geometry[shape], "physicsClientId": self.client}
        if shape == "box":
            size = _vec(item.get("size", [0.1, 0.1, 0.1]), 3, "object.size")
            if min(size) <= 0:
                raise ValueError("Object dimensions must be positive")
            kwargs["halfExtents"] = [v * 0.5 for v in size]
        else:
            radius = float(item.get("radius", 0.05))
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError("Object radius must be positive")
            kwargs["radius"] = radius
            if shape != "sphere":
                length = float(item.get("height", 0.1))
                if not math.isfinite(length) or length <= 0:
                    raise ValueError("Object height must be positive")
                kwargs["length" if visual else "height"] = length
        if visual:
            kwargs["rgbaColor"] = _vec(item.get("color", [0.45, 0.5, 0.55, 1]), 4, "object.color")
            return self._physics.createVisualShape(**kwargs)
        return self._physics.createCollisionShape(**kwargs)

    def _load_field(self, directory: Path) -> None:
        field = self.game.get("field", {})
        size = _vec(field.get("size", [3.6576, 3.6576]), 2, "field.size")
        if min(size) <= 0:
            raise ValueError("Field dimensions must be positive")
        self.field_size = size
        floor = {"name": "floor", "shape": "box", "size": [*size, 0.08],
                 "position": [0, 0, -0.04], "color": [0.28, 0.31, 0.35, 1]}
        if field.get("floor", True):
            self._add_object(floor, directory)
        # A visual-only tile pattern shares the single planar collision shape.
        tile = float(field.get("tile_size", 0.6096))
        if not math.isfinite(tile) or tile <= 0 or max(size) / tile > 200:
            raise ValueError("field.tile_size must be positive and produce at most 200 tiles per side")
        counts = [int(math.ceil(length / tile - 1e-10)) for length in size]
        for ix in range(counts[0] if field.get("floor", True) else 0):
            for iy in range(counts[1]):
                bounds = [(max(-length / 2, -count*tile/2 + index*tile),
                           min(length / 2, -count*tile/2 + (index+1)*tile))
                          for length, count, index in zip(size, counts, (ix, iy))]
                shade = 0.33 + 0.025 * ((ix + iy) % 2)
                self._add_object({"shape": "box", "size": [max(.0001, b-a-.004) for a,b in bounds] + [0.001],
                                  "position": [(a+b)/2 for a,b in bounds] + [.0001],
                                  "color": [shade, shade + .025, shade + .045, 1], "collision": False}, directory)
        if field.get("walls", True):
            h = float(field.get("wall_height", 0.30))
            thick = 0.0254
            for axis in (0, 1):
                for sign in (-1, 1):
                    dims = [size[0] + 2 * thick, size[1] + 2 * thick, h]
                    dims[axis] = thick
                    pos = [0., 0., h / 2]
                    pos[axis] = sign * (size[axis] / 2 + thick / 2)
                    self._add_object({"name": f"wall_{axis}_{sign}", "size": dims,
                                      "position": pos, "color": field.get("wall_color", [.15, .18, .23, 1])}, directory)
        for item in self.game.get("objects", []) + self.config.extra.get("field_objects", []):
            self._add_object(item, directory)

    def _add_object(self, item: dict[str, Any], directory: Path) -> int:
        position = _vec(item.get("position", [0, 0, 0]), 3, "object.position")
        rotation = _vec(item.get("rotation", [0, 0, 0]), 3, "object.rotation (radians)")
        orientation = self._physics.getQuaternionFromEuler(rotation)
        mass = float(item.get("mass", 0))
        if not math.isfinite(mass) or mass < 0:
            raise ValueError("Object mass must be finite and nonnegative")
        if "urdf" in item:
            path = Path(item["urdf"])
            if not path.is_absolute():
                path = directory / path
            if not path.is_file():
                raise ValueError(f"Game object URDF does not exist: {path}")
            # Free props enable natural sleeping/wakeup. Reduced coordinates
            # let this Bullet version set the leaf margins of STL hulls; its
            # maximal-coordinate margin API only changes the outer compound.
            rigid = ET.parse(path).getroot().find("joint") is None
            body = self._physics.loadURDF(str(path), position, orientation, useFixedBase=bool(item.get("fixed", mass == 0)),
                              useMaximalCoordinates=False, globalScaling=float(item.get("scale", 1)),
                              flags=self._physics.URDF_USE_INERTIA_FROM_FILE | self._physics.URDF_ENABLE_CACHED_GRAPHICS_SHAPES |
                                    (self._physics.URDF_ENABLE_SLEEPING if rigid else 0),
                              physicsClientId=self.client)
            # For URDF objects omit mass to use fixed=True by default; set
            # fixed=False to retain all masses/inertias authored in the URDF.
            if "mass" in item and not item.get("fixed", mass == 0):
                self._physics.changeDynamics(body, -1, mass=mass, physicsClientId=self.client)
        else:
            body = self._physics.createMultiBody(baseMass=mass, basePosition=position, baseOrientation=orientation,
                                     useMaximalCoordinates=mass > 0,
                                     baseCollisionShapeIndex=self._shape(item) if item.get("collision", True) else -1,
                                     baseVisualShapeIndex=self._shape(item, True), physicsClientId=self.client)
        # This Bullet build leaves getDynamicsInfo()[3:5] uninitialized for
        # maximal-coordinate bodies. Capture the actual local inertial transform
        # at import time, including any scaling/principal-axis rotation, so GPU
        # rendering can safely transform link-local visuals from the COM pose.
        self.base_inertial_frames[body] = self._physics.multiplyTransforms(
            *self._physics.invertTransform(position, orientation),
            *self._physics.getBasePositionAndOrientation(body, physicsClientId=self.client))
        for index in range(-1, self._physics.getNumJoints(body, physicsClientId=self.client)):
            kind = item.get("kind", item.get("scoring", {}).get("type"))
            plastic = kind in ("pin", "cup", "goal", "loader", "toggle")
            margin = item.get("collision_margin", .0001 if kind in ("pin", "cup", "goal") else None)
            contact = {} if margin is None else {"collisionMargin": max(0., float(margin))}
            friction = self.config.plastic_friction if plastic else self.config.friction
            self._physics.changeDynamics(body, index, lateralFriction=float(item.get("friction", friction)),
                             # Persistent friction anchors inject energy into
                             # freely rolling shapes in this Bullet build.
                             frictionAnchor=int(bool(item.get("friction_anchor", False))),
                             restitution=float(item.get("restitution", self.config.restitution)),
                             rollingFriction=float(item.get("rolling_friction", 0.001)),
                             physicsClientId=self.client, **contact)
            if self._physics.getDynamicsInfo(body, index, physicsClientId=self.client)[0] > 0:
                self._dynamic_field_bodies.add(body)
                # Static field inventories should become cheap once settled;
                # Bullet automatically wakes them again on physical contact.
                self._physics.changeDynamics(body, index, activationState=self._physics.ACTIVATION_STATE_ENABLE_SLEEPING,
                                 sleepThreshold=float(item.get("sleep_speed", .01)) ** 2,
                                 physicsClientId=self.client)
            if index >= 0 and self._physics.getJointInfo(body, index, physicsClientId=self.client)[2] in (self._physics.JOINT_REVOLUTE, self._physics.JOINT_PRISMATIC):
                self._physics.setJointMotorControl2(body, index, self._physics.VELOCITY_CONTROL, force=0, physicsClientId=self.client)
        score = deepcopy(item.get("scoring", {}))
        if not score and item.get("kind") == "pin":
            score = {"type": "pin", "length": .165, "radius": .0178}
        self.field_objects.append({"id": body, "name": item.get("name", ""), "mass": mass,
                                   "scoring": score})
        if body in self._dynamic_field_bodies and self._physics.getNumJoints(body, physicsClientId=self.client) == 0:
            position, quaternion = self._physics.getBasePositionAndOrientation(body, physicsClientId=self.client)
            self._rest_bodies[body] = {
                "since": self.tick, "minimum": list(position), "maximum": list(position),
                "quaternion": quaternion, "qminimum": list(quaternion), "qmaximum": list(quaternion), "still": False,
            }
        self._export_items.append((body, deepcopy(item), Path(directory)))
        return body

    def _deactivate_bounded_rest(self) -> None:
        """Deactivate supported numerical jitter using Bullet's normal sleep.

        Dense hollow-piece contacts can oscillate faster than Bullet's velocity
        threshold despite almost no displacement. Require a full two seconds
        within a 0.25 mm COM box diagonal and 0.2 degree pairwise orientation
        range. Moving pieces and anything touching the robot or an articulated
        field mechanism remain active. No transform, mass or collision changes
        are made. Only the already bounded residual velocity is cleared before
        native sleep; ordinary contact wakes these bodies again.
        """
        if not self.config.extra.get("numerical_rest", True) or not self._rest_bodies:
            return
        candidates: set[int] = set()
        stationary: set[int] = set()
        angular_chord_squared = (2 * math.sin(math.radians(.2) / 4)) ** 2
        for body, state in self._rest_bodies.items():
            velocity = self._physics.getBaseVelocity(body, physicsClientId=self.client)
            speed_squared = sum(v*v for vector in velocity for v in vector)
            if speed_squared == 0:
                state["still"] = True
                stationary.add(body)
                continue
            position, quaternion = self._physics.getBasePositionAndOrientation(body, physicsClientId=self.client)
            if sum(a*b for a, b in zip(state["quaternion"], quaternion)) < 0:
                quaternion = tuple(-q for q in quaternion)
            minimum = [min(a, b) for a, b in zip(state["minimum"], position)]
            maximum = [max(a, b) for a, b in zip(state["maximum"], position)]
            qminimum = [min(a, b) for a, b in zip(state["qminimum"], quaternion)]
            qmaximum = [max(a, b) for a, b in zip(state["qmaximum"], quaternion)]
            span_squared = sum((b-a)**2 for a, b in zip(minimum, maximum))
            rotation_span_squared = sum((b-a)**2 for a, b in zip(qminimum, qmaximum))
            if state["still"] or span_squared > .00025**2 or rotation_span_squared > angular_chord_squared:
                state.update(since=self.tick, minimum=list(position), maximum=list(position),
                             quaternion=quaternion, qminimum=list(quaternion), qmaximum=list(quaternion), still=False)
                continue
            state.update(minimum=minimum, maximum=maximum, qminimum=qminimum, qmaximum=qmaximum)
            # Separate impact ceilings admit the high-frequency angular jitter
            # this spatial test addresses, without reusing Bullet's stricter
            # mixed-unit speed criterion and reproducing its failure to sleep.
            if ((self.tick-state["since"])*self.dt >= 2 and
                    sum(v*v for v in velocity[0]) <= .01**2 and
                    sum(v*v for v in velocity[1]) <= .1**2):
                candidates.add(body)
        if not candidates:
            return
        neighbours: dict[int, set[int]] = {}
        pending = set(candidates)
        while pending:
            body = pending.pop()
            contacts = [c for c in self._physics.getContactPoints(body, physicsClientId=self.client)
                        if c[8] < .001]
            # Unsupported bodies must continue to fall, however small their
            # instantaneous speed may be (including near the top of a throw).
            if not any(c[9] > .001 for c in contacts):
                neighbours[body] = {self.robot}
                continue
            adjacent = set()
            for contact in contacts:
                other, link = ((contact[2], contact[4]) if contact[1] == body
                               else (contact[1], contact[3]))
                if (other == self.robot or other in self._dynamic_field_bodies or
                        self._physics.getDynamicsInfo(other, link, physicsClientId=self.client)[0] > 0):
                    adjacent.add(other)
            neighbours[body] = adjacent
            # Exact-zero velocity alone does not establish a resting contact
            # group: that neighbour might also touch the robot or a mechanism.
            pending |= (adjacent & stationary) - neighbours.keys()
        # Sleep complete qualifying contact groups together. Sleeping one cup
        # underneath an actively moving pin would leave their island awake.
        permitted = set(neighbours)
        while True:
            rejected = {body for body in permitted if neighbours[body] - permitted}
            if not rejected:
                break
            permitted -= rejected
        for body in candidates & permitted:
            # goToSleep() alone leaves residual multibody velocity intact in
            # Bullet 3.25, so its next activation check can immediately wake it.
            # Native island deactivation normally performs this zeroing later;
            # do it here only after the strict supported-rest checks above.
            self._physics.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0], physicsClientId=self.client)
            self._physics.changeDynamics(body, -1, activationState=self._physics.ACTIVATION_STATE_SLEEP,
                             physicsClientId=self.client)
            self._rest_bodies[body]["still"] = True
            self._rest_sleep_count += 1

    @property
    def joints(self) -> list[dict[str, Any]]:
        result = []
        for name, info in self._joint_info.items():
            if info[2] not in (self._physics.JOINT_REVOLUTE, self._physics.JOINT_PRISMATIC):
                continue
            state = self._physics.getJointState(self.robot, info[0], physicsClientId=self.client)
            result.append({"name": name, "index": info[0],
                           "type": "prismatic" if info[2] == self._physics.JOINT_PRISMATIC else ("continuous" if info[8] > info[9] else "revolute"),
                           "position": state[0], "velocity": state[1], "lower": info[8], "upper": info[9]})
        return result

    def _motor_torque(self, motor: dict[str, Any], position: float, velocity: float, command: dict[str, Any]) -> float:
        port = motor["port"]
        mode = command.get("mode", "voltage")
        value = float(command.get("value", 0))
        if not math.isfinite(value):
            raise ValueError(f"Non-finite command for motor {port}")
        nominal = motor["rpm"] * 2 * math.pi / 60
        stall = motor["stall_torque"]
        brake = command.get("brake", command.get("brake_mode", "coast"))
        stopped = mode in ("voltage", "velocity") and value == 0
        if not stopped:
            self._hold_targets.pop(port, None)
        if self._motor_modes.get(port) != mode:
            self._motor_modes[port] = mode
            self._motor_integrals.pop(port, None)
            self._control_duties.pop(port, None)
            self._next_control_time.pop(port, None)
        if stopped and brake not in ("hold", 2):
            self._hold_targets.pop(port, None)
            self._motor_integrals.pop(port, None)
            self._control_duties.pop(port, None)
            return 0.0 if brake in ("coast", 0) else pwm_torque(motor, 0, velocity)
        if stopped:
            target = self._hold_targets.setdefault(port, position)
            target_velocity = _clip(12 * (target - position), -nominal, nominal)
        elif mode == "position":
            target_velocity = _clip(float(motor.get("position_kp", 10)) * (math.radians(value) - position), -nominal, nominal)
            limit_rpm = abs(float(command.get("velocity", command.get("max_velocity", motor["rpm"]))))
            target_velocity = _clip(target_velocity, -limit_rpm * math.pi / 30, limit_rpm * math.pi / 30)
        elif mode == "velocity":
            target_velocity = _clip(value * math.pi / 30, -nominal, nominal)
        elif mode == "voltage":
            duty = _clip(value / 12000., -1, 1)
            torque = pwm_torque(motor, duty, velocity)
            # The published curve still has substantial torque at nominal RPM.
            # An approximate governor enforces the separate firmware speed cap.
            if abs(velocity) > nominal and torque * velocity > 0:
                torque = -math.copysign(min(stall, (abs(velocity) - nominal) * stall / nominal * 8), velocity)
            return torque
        elif mode == "torque":
            limit = min(stall * motor["current_limit"] / 2.5, motor["max_power"] / max(abs(velocity), .001))
            if value * velocity > 0:
                limit = min(limit, stall * torque_fraction(velocity / nominal))
            return _clip(value, -limit, limit)
        else:
            raise ValueError(f"Unknown motor control mode '{mode}'")
        if port not in self._control_duties or self.time + 1e-12 >= self._next_control_time.get(port, 0):
            error = (target_velocity - velocity) / nominal
            period = motor["controller_period"]
            integral = _clip(self._motor_integrals.get(port, 0) + error * period, -.4, .4)
            self._motor_integrals[port] = integral
            # Feedforward approximates the no-load PWM, with 10 ms PI updates.
            # The gains are tunable simulation parameters, not proprietary V5 gains.
            duty = _clip(target_velocity / nominal + float(motor.get("velocity_kp", 2.5)) * error + 1.5 * integral, -1, 1)
            self._control_duties[port] = duty
            next_time = self._next_control_time.get(port, self.time)
            self._next_control_time[port] = next_time + period * max(1, math.floor((self.time - next_time) / period) + 1)
        torque = pwm_torque(motor, self._control_duties[port], velocity)
        if abs(velocity) > nominal and torque * velocity > 0:
            torque = -math.copysign(min(stall, (abs(velocity) - nominal) * stall / nominal * 8), velocity)
        return torque

    def _apply_motors(self) -> None:
        with self.performance.span('world.control.motors'):
            torques: dict[int, float] = {}
            linear_limits: dict[int, float] = {}
            servos: dict[int, list[tuple[dict[str, Any], float, float]]] = {}
            self._servo_loads = {}
            for motor in self.motor_bindings:
                position, velocity, _, _ = self._physics.getJointState(self.robot, motor["index"], physicsClientId=self.client)
                factor = motor["factor"] * motor["sign"]
                command = self._commands["motors"].get(str(motor["port"]), {})
                servo = self._servo_command(motor, position * factor, velocity * factor, command)
                if servo is not None:
                    target, bound = servo
                    force = bound * abs(factor) * motor["efficiency"]
                    servos.setdefault(motor["index"], []).append((motor, target / factor, force))
                    continue
                torque = self._motor_torque(motor, position * factor, velocity * factor, command)
                bound = min(motor["stall_torque"] * motor["current_limit"] / 2.5,
                            motor["max_power"] / max(abs(velocity * factor), .001))
                torque = _clip(torque, -bound, bound)
                self._last_torques[motor["port"]] = torque
                torques[motor["index"]] = torques.get(motor["index"], 0) + torque * factor * motor["efficiency"]
                if self._joint_info[motor["joint"]][2] == self._physics.JOINT_PRISMATIC:
                    limit = motor["rpm"] * math.pi / 30 / abs(factor)
                    linear_limits[motor["index"]] = min(linear_limits.get(motor["index"], limit), limit)
            for index, torque in torques.items():
                if index in linear_limits:
                    # Screw reductions can produce a very stiff actuator. Bullet's
                    # implicit finite-force motor enforces the V5 speed bound without
                    # an explicit force overshooting many revolutions in one tick.
                    self._physics.setJointMotorControl2(self.robot, index, self._physics.VELOCITY_CONTROL,
                                           targetVelocity=math.copysign(linear_limits[index], torque),
                                           force=abs(torque), physicsClientId=self.client)
                else:
                    self._physics.setJointMotorControl2(self.robot, index, self._physics.VELOCITY_CONTROL, force=0, physicsClientId=self.client)
                    self._physics.setJointMotorControl2(self.robot, index, self._physics.TORQUE_CONTROL, force=torque, physicsClientId=self.client)
            for index, group in servos.items():
                total = sum(force for _, _, force in group)
                if total == 0 and index in torques:
                    for motor, _, _ in group:
                        self._last_torques[motor["port"]] = 0
                    continue
                target = sum(target * force for _, target, force in group) / total if total > 0 else 0
                self._physics.setJointMotorControl2(self.robot, index, self._physics.VELOCITY_CONTROL, targetVelocity=target,
                                       force=total, velocityGain=1, physicsClientId=self.client)
                self._servo_loads[index] = [(motor, force / total if total else 0) for motor, _, force in group]
        with self.performance.span('world.control.pneumatics'):
            for piston in self._pneumatics:
                value = self._commands["adi"].get(str(piston["port"]).upper(), False)
                target = float(piston.get("extended", 0.05) if value else piston.get("retracted", 0))
                self._physics.setJointMotorControl2(self.robot, piston["index"], self._physics.POSITION_CONTROL, targetPosition=target,
                                       force=float(piston.get("force", 40)), maxVelocity=float(piston.get("max_velocity", 1)),
                                       positionGain=0.15, velocityGain=1, physicsClientId=self.client)
        with self.performance.span('world.control.manual_joints'):
            for name, command in self._commands["joints"].items():
                if isinstance(command, (int, float)):
                    command = {"mode": "position", "value": command}
                info = self._require_joint(name)
                mode = command.get("mode", "position")
                value = float(command.get("value", 0))
                if not math.isfinite(value):
                    raise ValueError("Joint command must be finite")
                force = min(abs(float(command.get("force", info[10] or 10))), info[10] or 1000)
                if mode == "position":
                    if info[8] <= info[9]:
                        value = _clip(value, info[8], info[9])
                    self._physics.setJointMotorControl2(self.robot, info[0], self._physics.POSITION_CONTROL, targetPosition=value, force=force, physicsClientId=self.client)
                elif mode == "velocity":
                    self._physics.setJointMotorControl2(self.robot, info[0], self._physics.VELOCITY_CONTROL, targetVelocity=value, force=force, physicsClientId=self.client)
                elif mode == "torque":
                    self._physics.setJointMotorControl2(self.robot, info[0], self._physics.VELOCITY_CONTROL, force=0, physicsClientId=self.client)
                    self._physics.setJointMotorControl2(self.robot, info[0], self._physics.TORQUE_CONTROL, force=_clip(value, -force, force), physicsClientId=self.client)
                else:
                    raise ValueError(f"Unknown joint control mode '{mode}'")

    def _servo_command(self, motor: dict[str, Any], position: float, velocity: float,
                       command: dict[str, Any]) -> tuple[float, float] | None:
        """Implicit finite-force V5 velocity/position/brake controller.

        An explicit high-gain torque PI can inject energy into a light wheel at
        240 Hz. Bullet solves this bounded velocity actuator together with the
        contacts and inertia, preserving the published torque/current envelope.
        No body or joint position/velocity is reset to implement motion.
        """
        mode, value = command.get("mode", "voltage"), float(command.get("value", 0))
        if not math.isfinite(value):
            raise ValueError("Motor command must be finite")
        stopped = mode in ("voltage", "velocity") and value == 0
        if mode == "voltage" and not stopped:
            self._hold_targets.pop(motor["port"], None)
            duty = _clip(value / 12000., -1, 1)
            return pwm_no_load_speed(motor, duty), abs(pwm_torque(motor, duty, velocity))
        if mode not in ("velocity", "position") and not stopped:
            return None
        port = motor["port"]
        nominal = motor["rpm"] * math.pi / 30
        brake = command.get("brake", command.get("brake_mode", "coast"))
        if not stopped:
            self._hold_targets.pop(port, None)
        if stopped and brake in ("coast", 0):
            self._hold_targets.pop(port, None)
            self._servo_targets.pop(port, None)
            return (0, 0)
        if stopped and brake in ("brake", 1):
            self._hold_targets.pop(port, None)
            return (0, abs(pwm_torque(motor, 0, velocity)))
        if self._motor_modes.get(port) != mode:
            self._servo_targets.pop(port, None)
            self._motor_modes[port] = mode
        if port not in self._servo_targets or self.time + 1e-12 >= self._next_control_time.get(port, 0):
            if stopped:
                target_position = self._hold_targets.setdefault(port, position)
                target = 12 * (target_position - position)
            elif mode == "position":
                target = float(motor.get("position_kp", 10)) * (math.radians(value) - position)
            else:
                target = value * math.pi / 30
            limit = nominal
            if mode == "position":
                profile_rpm = abs(float(command.get("velocity", command.get("max_velocity", motor["rpm"]))))
                limit = min(nominal, profile_rpm * math.pi / 30)
            self._servo_targets[port] = _clip(target, -limit, limit)
            period = motor["controller_period"]
            next_time = self._next_control_time.get(port, self.time)
            self._next_control_time[port] = next_time + period * max(1, math.floor((self.time - next_time) / period) + 1)
        target = self._servo_targets[port]
        bound = motor["stall_torque"] * motor["current_limit"] / 2.5
        if (target - velocity) * velocity >= 0:
            bound = min(bound, motor["stall_torque"] * torque_fraction(velocity / nominal))
        bound = min(bound, motor["max_power"] / max(abs(velocity), .001))
        return target, bound

    def _point_velocity(self, body: int, link: int, point: np.ndarray) -> np.ndarray:
        key = (body, link)
        if key not in self._point_motion_cache:
            if link < 0:
                pos, _ = self._physics.getBasePositionAndOrientation(body, physicsClientId=self.client)
                linear, angular = self._physics.getBaseVelocity(body, physicsClientId=self.client)
            else:
                # Bullet's integration updates collider poses before cached link
                # frames. Refresh the COM used by tire forces independently of
                # whether the renderer has requested forward kinematics.
                state = self._physics.getLinkState(body, link, computeLinkVelocity=1,
                                                     computeForwardKinematics=1, physicsClientId=self.client)
                pos, linear, angular = state[0], state[6], state[7]
            self._point_motion_cache[key] = (np.asarray(pos), np.asarray(linear), angular)
        pos, linear, angular = self._point_motion_cache[key]
        return linear + _cross3(angular, point - pos)

    def _apply_omni_tires(self) -> None:
        if self.robot_settings.get("drive", {}).get("type") != "omni":
            return
        # Applying forces does not change velocities until Bullet steps, so
        # every contact on a link can reuse this tick's rigid motion.
        self._point_motion_cache = {}
        contact_masses = {}
        _, quaternion = self._physics.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        rotation = np.array(self._physics.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
        for wheel in self._wheels:
            world_direction = rotation @ np.array(wheel["direction"])
            contacts = self._physics.getContactPoints(bodyA=self.robot, linkIndexA=wheel["index"], physicsClientId=self.client)
            # Settings may be edited between ticks. Reuse their scalar values
            # only within this wheel's contact loop, and read them lazily so an
            # airborne wheel retains the same validation/force behavior.
            tire_settings = None
            if self.performance.enabled:
                self.performance.increment("world.tire_contact_reports", len(contacts))
            for contact in contacts:
                normal = np.array(contact[7])
                normal_load = max(0., contact[9])
                if normal_load <= 0 or normal[2] < 0.35 or contact[2] == self.robot:
                    continue
                point = np.array(contact[5])
                direction = world_direction.copy()
                direction -= normal * np.dot(direction, normal)
                length = np.linalg.norm(direction)
                if length < 0.001:
                    continue
                direction /= length
                lateral = _cross3(normal, direction)
                relative = self._point_velocity(self.robot, wheel["index"], point) - self._point_velocity(contact[2], contact[4], point)
                # tanh is a bounded Coulomb regularization; its product with slip
                # always dissipates energy. Normal loads come from Bullet contacts.
                if tire_settings is None:
                    tire_settings = (float(wheel.get("friction", self.config.friction)),
                                     float(wheel.get("slip_speed", 0.15)),
                                     float(wheel.get("radius", 0.0254)),
                                     float(wheel.get("roller_friction", 0.025)))
                mu, slip_scale, radius, roller_friction = tire_settings
                slip = float(np.dot(relative, direction))
                driven_force = -mu * normal_load * math.tanh(slip / max(.01, slip_scale))
                stopping_force = abs(slip) * wheel["inertia"] / (self.dt * radius * radius)
                driven_force = _clip(driven_force, -stopping_force, stopping_force)
                roller_force = -roller_friction * normal_load * math.tanh(float(np.dot(relative, lateral)) / .1)
                force = direction * driven_force + lateral * roller_force
                self.performance.increment("world.tire_contacts_applied")
                self._physics.applyExternalForce(self.robot, wheel["index"], force.tolist(), point.tolist(), self._physics.WORLD_FRAME, physicsClientId=self.client)
                # Equal and opposite reaction preserves momentum for movable fields.
                other = (contact[2], contact[4])
                if other not in contact_masses:
                    contact_masses[other] = self._physics.getDynamicsInfo(*other, physicsClientId=self.client)[0]
                if contact_masses[other] > 0:
                    self._physics.applyExternalForce(contact[2], contact[4], (-force).tolist(), point.tolist(), self._physics.WORLD_FRAME, physicsClientId=self.client)

    def step(self, commands: dict[str, Any] | None = None) -> dict[str, Any]:
        if commands is not None:
            bindings = {motor["port"]: motor for motor in self.motor_bindings}
            for key, command in commands.get("motors", {}).items():
                port = int(key)
                if port not in bindings and float(command.get("value", 0)) != 0:
                    raise ValueError(f"Motor port {port} was commanded but has no URDF joint binding; configure it in Hardware")
                configured_rpm = command.get("gearset")
                if port in bindings and configured_rpm is not None:
                    physical = bindings[port]
                    mismatch = (port, int(configured_rpm))
                    if int(configured_rpm) != physical["rpm"] and mismatch not in self._gear_mismatches:
                        self._gear_mismatches.add(mismatch)
                        self._diagnostics.append(
                            f"Motor {port}: code configures {configured_rpm} rpm; physical {physical['cartridge']} cartridge is {physical['rpm']:g} rpm. "
                            "Software gearing does not change the installed cartridge; match the Hardware binding to the real robot."
                        )
            for family in ("motors", "adi", "joints"):
                for key, value in commands.get(family, {}).items():
                    key = str(key)
                    if family == "adi":
                        key = chr(64 + int(key)) if key.isdigit() and 1 <= int(key) <= 8 else key.upper()
                    self._commands[family][key] = deepcopy(value)
        self._apply_motors()
        with self.performance.span("world.control.tires"):
            self._apply_omni_tires()
        with self.performance.span("world.integrate"):
            self._physics.stepSimulation(physicsClientId=self.client)
        with self.performance.span('world.control.feedback'):
            for index, group in self._servo_loads.items():
                applied = self._physics.getJointState(self.robot, index, physicsClientId=self.client)[3]
                for motor, share in group:
                    self._last_torques[motor["port"]] = applied * share / (motor["factor"] * motor["sign"] * motor["efficiency"])
        self.tick += 1
        self.time = self.tick * self.dt
        with self.performance.span("world.rest"):
            self._deactivate_bounded_rest()
        with self.performance.span("world.sensors"):
            self._observe()
        if self.performance.enabled:
            self.performance.increment("world.steps")
            self.performance.increment("world.motor_updates", len(self.motor_bindings))
            self.performance.increment("world.pneumatic_updates", len(self._pneumatics))
            self.performance.gauge("world.motor_bindings", len(self.motor_bindings))
            self.performance.gauge("world.field_objects", len(self.field_objects))
        with self.performance.span("world.snapshot"):
            return self.snapshot()

    def disable(self) -> None:
        """Cut motor power immediately without advancing simulation time."""
        self._commands["motors"] = {
            str(motor["port"]): {"mode": "voltage", "value": 0, "brake": "coast"}
            for motor in self.motor_bindings
        }
        self._commands["joints"].clear()
        self._motor_integrals.clear()
        self._control_duties.clear()
        self._next_control_time.clear()
        self._motor_modes.clear()
        self._servo_targets.clear()
        self._servo_loads.clear()
        self._hold_targets.clear()
        for info in self._joint_info.values():
            if info[2] in (self._physics.JOINT_REVOLUTE, self._physics.JOINT_PRISMATIC):
                self._physics.setJointMotorControl2(self.robot, info[0], self._physics.VELOCITY_CONTROL, force=0, physicsClientId=self.client)
        self._last_torques = {motor["port"]: 0 for motor in self.motor_bindings}

    def _distances(self) -> dict[str, float]:
        sensors = self.robot_settings.get("distance_sensors", [])
        if not sensors:
            return {}
        position, quaternion = self._physics.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        rot = np.array(self._physics.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
        result = {}
        for sensor in sensors:
            offset = np.array(_vec(sensor.get("position", [.22, 0, .07]), 3, "distance.position"))
            direction = np.array(_vec(sensor.get("direction", [1, 0, 0]), 3, "distance.direction"))
            direction_length = np.linalg.norm(direction)
            if direction_length < .001:
                raise ValueError("Distance sensor direction cannot be zero")
            ray_direction = rot @ (direction / direction_length)
            origin = np.asarray(position) + rot @ offset
            limit = float(sensor.get("max_range", 2.0))
            if not math.isfinite(limit) or limit <= 0:
                raise ValueError("Distance sensor max_range must be positive metres")
            hit = self._physics.rayTest(origin.tolist(), (origin + limit * ray_direction).tolist(), physicsClientId=self.client)[0]
            distance = 9999.0 if hit[0] < 0 else max(0, hit[2] * limit * 1000)
            result[str(sensor["port"])] = distance
        return result

    def _observe(self) -> None:
        position, quaternion = self._physics.getBasePositionAndOrientation(self.robot, physicsClientId=self.client)
        yaw = self._physics.getEulerFromQuaternion(quaternion)[2]
        delta = math.atan2(math.sin(yaw - self._last_yaw), math.cos(yaw - self._last_yaw))
        self._rotation += delta
        self._last_yaw = yaw
        noise = self.config.noise
        settings = self.robot_settings.get("sensors", {})
        position_noise = self.rng.normal(0, noise["position_std_m"], 2)
        yaw_noise = self.rng.normal(0, noise["heading_std_deg"])
        noisy_yaw = math.degrees(self._rotation) + yaw_noise
        imu_sign = float(settings.get("imu_heading_sign", -1))
        otos = settings.get("otos", {})
        frame: dict[str, Any] = {
            "sample_time": self.time,
            "imu": {"heading": (noisy_yaw * imu_sign) % 360, "rotation": noisy_yaw * imu_sign},
            "otos": [(position[0] + position_noise[0]) / .0254 * float(otos.get("x_sign", 1)),
                     (position[1] + position_noise[1]) / .0254 * float(otos.get("y_sign", -1)),
                     noisy_yaw * float(otos.get("heading_sign", 1))],
            "motors": {}, "distance": self._distances(),
        }
        for motor in self.motor_bindings:
            pos, velocity, _, _ = self._physics.getJointState(self.robot, motor["index"], physicsClientId=self.client)
            frame["motors"][str(motor["port"])] = {
                "position": pos * motor["factor"] * motor["sign"] + math.radians(self.rng.normal(0, noise["encoder_std_deg"])),
                "velocity": velocity * motor["factor"] * motor["sign"],
                "torque": self._last_torques.get(motor["port"], 0),
                "current_estimate": min(motor["current_limit"], abs(self._last_torques.get(motor["port"], 0)) / motor["stall_torque"] * 2.5),
            }
        for port, value in frame["distance"].items():
            if value < 9999:
                frame["distance"][port] = max(0, value + self.rng.normal(0, noise["distance_std_mm"]))
        if self._delivered is None:
            self._delivered = deepcopy(frame)
        self._latency.append(frame)
        cutoff = self.time - noise["latency_ms"] / 1000
        # Latency is sampled at physics ticks; dropout means sample-and-hold.
        while self._latency and self._latency[0]["sample_time"] <= cutoff + 1e-12:
            ready = self._latency.popleft()
            if self.rng.random() >= noise["dropout"]:
                self._delivered = ready
        self._snapshot = deepcopy(self._delivered)
        linear, angular = self._physics.getBaseVelocity(self.robot, physicsClientId=self.client)
        if self._scoring_cache is None or self.tick % max(1, self.config.physics_hz // 30) == 0:
            self._scoring_cache = self._scoring()
        self._snapshot.update({"time": self.time, "tick": self.tick,
                               "pose": [*position, yaw], "velocity": list(linear),
                               "angular_velocity": list(angular),
                               "joints": self.joints,
                               "diagnostics": list(self._diagnostics),
                               "scoring": deepcopy(self._scoring_cache),
                               "sensor_age_ms": (self.time - self._delivered["sample_time"]) * 1000})

    def _has_contact(self, body_a: int, body_b: int) -> bool:
        return bool(self._physics.getContactPoints(body_a, body_b, physicsClientId=self.client))

    def _scoring(self) -> dict[str, Any]:
        """Prototype pin placement detection using insertion and actual contacts.

        This deliberately does not calculate official game points or adjudicate
        alliance orientation, stack multipliers, toggles, control or violations.
        """
        goals = [item for item in self.field_objects if item["scoring"].get("type") == "goal"]
        pins = [item for item in self.field_objects if item["scoring"].get("type") == "pin"]
        # The world cannot move during this synchronous observation. Reuse a
        # goal's pose and numeric metadata across pins, then discard the cache:
        # custom movable goals, live edits and resets remain visible next call.
        goal_geometry = {}
        placed = []
        for pin in pins:
            xyz, quat = self._physics.getBasePositionAndOrientation(pin["id"], physicsClientId=self.client)
            axis = np.array(self._physics.getMatrixFromQuaternion(quat)).reshape(3, 3)[:, 2]
            if abs(axis[2]) < .94:
                continue
            bottom = np.array(xyz) - np.sign(axis[2]) * axis * float(pin["scoring"].get("length", .165)) / 2
            if self._has_contact(pin["id"], self.robot):
                continue
            for goal_index, goal in enumerate(goals):
                spec = goal["scoring"]
                if goal_index not in goal_geometry:
                    centre, _ = self._physics.getBasePositionAndOrientation(goal["id"], physicsClientId=self.client)
                    goal_geometry[goal_index] = (
                        np.array(centre[:2]), float(spec["height"]) + float(spec.get("base_z", 0)),
                        float(spec.get("socket_radius", .027)))
                centre_xy, top, socket_radius = goal_geometry[goal_index]
                clearance = socket_radius - float(pin["scoring"].get("radius", .020)) + .003
                if np.linalg.norm(bottom[:2] - centre_xy) > clearance:
                    continue
                if not top - float(spec.get("socket_depth", .055)) - .008 <= bottom[2] <= top + .002:
                    continue
                if self._has_contact(pin["id"], goal["id"]):
                    placed.append({"pin": pin["name"], "goal": goal["name"], "alliance": spec.get("alliance", "neutral")})
                    break
        return {"label": "Prototype placed pins · contact detection, not official points",
                "placed_pins": len(placed), "placements": placed}

    def snapshot(self) -> dict[str, Any]:
        """Cached sensor sample: rendering/read frequency cannot affect noise."""
        return {**deepcopy(self._snapshot), "physics": deepcopy(self.physics_status)}

    def render(self, width: int = 960, height: int = 720, yaw: float = -45,
               pitch: float = -55, distance: float = 5, target: list[float] | None = None) -> np.ndarray:
        width, height = int(width), int(height)
        if not 16 <= width <= 4096 or not 16 <= height <= 4096:
            raise ValueError("Image dimensions must be between 16 and 4096")
        target = target or [0, 0, 0]
        if self._render_backend is None:
            from .render_backend import RenderBackend
            self._render_backend = RenderBackend(self.client, self.config.extra.get("renderer", "auto"),
                                                 base_inertial_frames=self.base_inertial_frames,
                                                 physics_api=self._physics)
        pixels = self._render_backend.render(width, height, yaw, pitch, distance, target)
        self.renderer_status = dict(self._render_backend.status)
        if pixels is not None:
            return pixels
        view = self._physics.computeViewMatrixFromYawPitchRoll(target, max(.1, distance), yaw, pitch, 0, 2)
        projection = self._physics.computeProjectionMatrixFOV(52, width / height, .02, 30)
        camera = self._physics.getCameraImage(width, height, view, projection,
                                 renderer=self._physics.ER_TINY_RENDERER, lightDirection=[-2, -3, 6],
                                 shadow=1, flags=self._physics.ER_NO_SEGMENTATION_MASK, physicsClientId=self.client)
        pixels = np.asarray(camera[2], dtype=np.uint8).reshape(height, width, 4).copy()
        depth = np.asarray(camera[3]).reshape(height, width)
        pixels[depth >= .99999] = [9, 20, 32, 255]
        return pixels

    def close(self) -> None:
        if getattr(self, "_render_backend", None) is not None:
            self._render_backend.close()
            self._render_backend = None
        if getattr(self, "client", -1) >= 0:
            if self._physics.isConnected(self.client):
                self._physics.disconnect(physicsClientId=self.client)
            self.client = -1

    def __enter__(self) -> "World":
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()
