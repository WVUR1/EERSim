"""Compile the loaded EERSim scene into native MuJoCo bodies and contacts.

The importer is only queried for metadata; it never advances Bullet. Link
frames, explicit mass/inertia, initial joint positions, and individual convex
collision leaves are retained. Static concave triangle meshes become rigid
flexes (all vertices belong to one body), preserving goal openings. MuJoCo
requires a positive radius on 2D flexes: their explicit shell is one micrometre.

Contact solvers differ: MuJoCo uses compliant constraints, not Bullet impulses.
The runtime maps combined restitution to a damping ratio through the damped
oscillator approximation below; contact compliance and timestep still affect
measured rebound. This is not an exact cross-engine restitution equivalence.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import struct
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np


@dataclass
class Scene:
    model: Any
    data: Any
    body_ids: dict[tuple[int, int], int]
    joint_ids: dict[tuple[int, int], int]
    geom_keys: dict[int, tuple[int, int]]
    flex_keys: dict[int, tuple[int, int]]
    free_joints: dict[int, int]
    actuator_ids: dict[tuple[int, int], int]
    dynamics: dict[tuple[int, int], tuple]
    xml: str
    cpu_threads: int = 0



def physics_worker_count(requested: int) -> int:
    """Bound native workers to available CPUs; the calling thread also works.

    MuJoCo 3.17 has no native upper bound and constructs one OS thread per
    worker. Respect affinity on Linux and avoid oversubscribing a profile saved
    on a larger machine. Zero explicitly selects inline execution.
    """
    if hasattr(os, "sched_getaffinity"):
        available = len(os.sched_getaffinity(0))
    else:
        available = getattr(os, "process_cpu_count", os.cpu_count)() or 1
    return min(requested, max(0, available - 1))


def restitution_damping_ratio(restitution: float) -> float:
    """Approximate restitution with the underdamped oscillator rebound ratio.

    ``e = exp(-pi*zeta/sqrt(1-zeta**2))`` assumes a linear impact oscillator.
    MuJoCo's impedance and contact activation make actual restitution dependent
    on timestep and scene. Zero restitution is critically damped; a small
    positive lower bound is required by MuJoCo's positive solref format.
    """
    restitution = float(restitution)
    if not math.isfinite(restitution) or not 0 <= restitution <= 1:
        raise ValueError("Restitution must be finite and between zero and one")
    if restitution == 0:
        return 1.
    logarithm = -math.log(restitution)
    return max(1e-4, logarithm / math.sqrt(math.pi ** 2 + logarithm ** 2))


def _numbers(values) -> str:
    values = [float(value) for value in values]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("MuJoCo geometry and physical properties must be finite")
    return " ".join(format(value, ".17g") for value in values)


def _vector(element, attribute, default, scale=1.) -> list[float]:
    text = element.get(attribute, default) if element is not None else default
    values = [float(value) * scale for value in text.split()]
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"Invalid URDF {attribute}: {text!r}")
    return values


def _quat(values) -> str:
    return _numbers([values[3], *values[:3]])


def _mesh_triangles(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read source triangles without approximating or convexifying the mesh."""
    if path.suffix.lower() == ".obj":
        vertices, faces = [], []
        for line in path.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if not parts:
                continue
            if parts[0] == "v":
                vertices.append([float(value) for value in parts[1:4]])
            elif parts[0] == "f":
                ids = [int(value.split("/")[0]) for value in parts[1:]]
                ids = [value - 1 if value > 0 else len(vertices) + value for value in ids]
                faces.extend((ids[0], ids[index], ids[index+1]) for index in range(1, len(ids)-1))
        points = np.asarray(vertices, dtype=float)
        triangles = np.asarray(faces, dtype=int)
    elif path.suffix.lower() == ".stl":
        data = path.read_bytes()
        if len(data) < 84 or len(data) != 84 + 50 * struct.unpack_from("<I", data, 80)[0]:
            raise ValueError(f"MuJoCo concave STL import requires binary STL: {path}")
        points = np.asarray([struct.unpack_from("<12fH", data, offset)[3:12]
                             for offset in range(84, len(data), 50)], dtype=float).reshape(-1, 3)
        points, indices = np.unique(points, axis=0, return_inverse=True)
        triangles = indices.reshape(-1, 3)
    else:
        raise ValueError(f"MuJoCo collision meshes must use OBJ or binary STL: {path}")
    if (points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points))
            or triangles.ndim != 2 or triangles.shape[1] != 3 or not len(triangles)
            or triangles.min() < 0 or triangles.max() >= len(points)):
        raise ValueError(f"Invalid collision mesh: {path}")
    # OBJ seams commonly duplicate coordinates for visual normals/UVs. Flex
    # collision has neither, so weld only bit-identical positions. Keep first
    # occurrence order and every triangle: this changes storage, not surfaces.
    _, first, inverse = np.unique(points, axis=0, return_index=True, return_inverse=True)
    if len(first) != len(points):
        order = np.argsort(first)
        remap = np.empty(len(order), dtype=int)
        remap[order] = np.arange(len(order))
        points, triangles = points[first[order]], remap[inverse[triangles]]
    # MuJoCo rejects repeated vertices/zero area elements; no physical surface
    # is removed by discarding triangles whose area is exactly zero.
    area2 = np.linalg.norm(np.cross(points[triangles[:, 1]] - points[triangles[:, 0]],
                                   points[triangles[:, 2]] - points[triangles[:, 0]]), axis=1)
    triangles = triangles[area2 > 0]
    if not len(triangles):
        raise ValueError(f"Collision mesh has no nondegenerate triangles: {path}")
    return points, triangles


class _Compiler:
    def __init__(self, world):
        self.world, self.p, self.client = world, world._physics, world.client
        self.root = ET.Element("mujoco", model="EERSim")
        ET.SubElement(self.root, "compiler", angle="radian", autolimits="true",
                      inertiafromgeom="false", fusestatic="false", balanceinertia="false")
        ET.SubElement(self.root, "option", timestep=_numbers([world.dt]),
                      gravity=_numbers([0, 0, -world.config.gravity]),
                      integrator="implicitfast", solver="Newton", cone="elliptic",
                      iterations=str(world.config.physics_solver_iterations), tolerance="1e-8")
        default = ET.SubElement(self.root, "default")
        ET.SubElement(default, "geom", solref="0.005 1", solimp="0.95 0.99 0.001",
                      margin="0", gap="0", density="0")
        self.asset = ET.SubElement(self.root, "asset")
        self.worldbody = ET.SubElement(self.root, "worldbody")
        self.contact = ET.SubElement(self.root, "contact")
        self.deformable = ET.SubElement(self.root, "deformable")
        self.actuator = ET.SubElement(self.root, "actuator")
        self.bodies, self.joints, self.geoms, self.flexes = {}, {}, {}, {}
        self.free, self.actuators, self.dynamics, self.initial = {}, {}, {}, {}
        self.meshes, self.assets, self.triangles = {}, {}, {}
        self.moving_boxes = []

    def dynamics_for(self, body, link):
        key = body, link
        if key not in self.dynamics:
            values = list(self.p.getDynamicsInfo(body, link, physicsClientId=self.client))
            if link == -1 and body in self.world.base_inertial_frames:
                values[3], values[4] = self.world.base_inertial_frames[body]
            self.dynamics[key] = tuple(values)
        return self.dynamics[key]

    def base_pose(self, body):
        dynamics = self.dynamics_for(body, -1)
        pose = self.p.getBasePositionAndOrientation(body, physicsClientId=self.client)
        return self.p.multiplyTransforms(*pose, *self.p.invertTransform(dynamics[3], dynamics[4]))

    def body(self, parent, body, link, position, orientation):
        name = f"b{body}_l{link}"
        self.bodies[body, link] = name
        element = ET.SubElement(parent, "body", name=name, pos=_numbers(position), quat=_quat(orientation))
        dynamics = self.dynamics_for(body, link)
        # Fixed bases still retain their imported COM frame. Giving them zero
        # mass/inertia preserves that frame without adding dynamic inertia.
        ET.SubElement(element, "inertial", mass=_numbers([dynamics[0]]), pos=_numbers(dynamics[3]),
                      quat=_quat(dynamics[4]), diaginertia=_numbers(dynamics[2]))
        return element

    def free_joint(self, element, body):
        if self.dynamics_for(body, -1)[0] > 0:
            name = f"free{body}"
            ET.SubElement(element, "freejoint", name=name)
            self.free[body] = name

    def mesh(self, path, scale):
        key = str(path), tuple(scale)
        if key not in self.meshes:
            name = f"mesh{len(self.meshes)}"
            filename = name + path.suffix.lower()
            if path.suffix.lower() not in (".obj", ".stl"):
                raise ValueError(f"MuJoCo collision meshes must use OBJ or binary STL: {path}")
            self.assets[filename] = path.read_bytes()
            ET.SubElement(self.asset, "mesh", name=name, file=filename, scale=_numbers(scale), inertia="convex")
            self.meshes[key] = name
        return self.meshes[key]

    def collision(self, element, body, link, collision, directory, scale, concave=False):
        origin = collision.find("origin")
        xyz = _vector(origin, "xyz", "0 0 0", scale)
        rpy = _vector(origin, "rpy", "0 0 0")
        quaternion = self.p.getQuaternionFromEuler(rpy)
        geometry = collision.find("geometry")
        if geometry is None or len(geometry) != 1:
            raise ValueError("Every URDF collision needs exactly one geometry primitive")
        shape = geometry[0]
        dynamics = self.dynamics_for(body, link)
        # Native MuJoCo material mixing uses max; the runtime applies the
        # imported material-product rule to each generated contact before solve.
        friction = _numbers([dynamics[1], dynamics[7], dynamics[6]])
        dimensions = "6" if dynamics[6] > 0 else ("4" if dynamics[7] > 0 else "3")
        attrs = {"pos": _numbers(xyz), "quat": _quat(quaternion), "friction": friction,
                 "condim": dimensions}
        if shape.tag == "mesh":
            filename = shape.get("filename", "")
            if "://" in filename:
                raise ValueError("URDF mesh URIs must be resolved to local relative/absolute paths before MuJoCo import")
            path = (directory / filename).resolve()
            mesh_scale = _vector(shape, "scale", "1 1 1", scale)
            if min(mesh_scale) <= 0:
                raise ValueError(f"MuJoCo mesh scale must be positive: {path}")
            if concave or collision.get("concave") == "yes":
                # A rigid flex preserves every concave source triangle. This
                # does not create soft bodies, artificial joints or a hull.
                if self.dynamics_for(body, -1)[0] > 0 or link >= 0:
                    raise ValueError("MuJoCo concave triangle colliders currently require a fixed root link; use convex collision leaves on moving links")
                if path not in self.triangles:
                    self.triangles[path] = _mesh_triangles(path)
                vertices, faces = self.triangles[path]
                rotation = np.asarray(self.p.getMatrixFromQuaternion(quaternion)).reshape(3, 3)
                points = (vertices * mesh_scale) @ rotation.T + xyz
                name = f"flex{len(self.flexes)}"
                flex = ET.SubElement(self.deformable, "flex", name=name, dim="2", radius="0.000001",
                                     body=self.bodies[body, link], vertex=_numbers(points.reshape(-1)),
                                     element=" ".join(str(int(index)) for index in faces.reshape(-1)))
                ET.SubElement(flex, "contact", selfcollide="none", friction=friction,
                              condim=dimensions, solref="0.005 1", solimp="0.95 0.99 0.001")
                self.flexes[name] = body, link
                return
            attrs.update(type="mesh", mesh=self.mesh(path, mesh_scale))
        elif shape.tag == "box":
            attrs.update(type="box", size=_numbers(np.asarray(_vector(shape, "size", "0 0 0", scale)) / 2))
        elif shape.tag in ("sphere", "cylinder", "capsule"):
            sizes = [float(shape.get("radius")) * scale]
            if shape.tag != "sphere":
                sizes.append(float(shape.get("length")) * scale / 2)
            attrs.update(type=shape.tag, size=_numbers(sizes))
        else:
            raise ValueError(f"Unsupported MuJoCo URDF geometry: {shape.tag}")
        name = f"geom{len(self.geoms)}"
        geom = ET.SubElement(element, "geom", name=name, **attrs)
        if shape.tag == "box" and (self.dynamics_for(body, -1)[0] > 0 or link >= 0):
            self.moving_boxes.append(geom)
        self.geoms[name] = body, link

    def urdf(self, body, path, scale=1.):
        root = ET.parse(path).getroot()
        links = {element.get("name"): element for element in root.findall("link")}
        joints = root.findall("joint")
        children, incoming = {}, {}
        for joint in joints:
            kind, name = joint.get("type"), joint.get("name")
            if kind not in ("fixed", "continuous", "revolute", "prismatic"):
                raise ValueError(f"MuJoCo does not implement URDF joint '{name}' type '{kind}'")
            for feature in ("mimic", "safety_controller", "calibration"):
                if joint.find(feature) is not None:
                    raise ValueError(f"MuJoCo does not implement URDF {feature} on joint '{name}'")
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            children.setdefault(parent, []).append(joint)
            incoming[child] = joint
        roots = set(links) - set(incoming)
        if len(roots) != 1 or len(incoming) != len(joints):
            raise ValueError(f"URDF must describe one acyclic link tree: {path}")
        root_name = roots.pop()
        infos = [self.p.getJointInfo(body, index, physicsClientId=self.client)
                 for index in range(self.p.getNumJoints(body, physicsClientId=self.client))]
        indices = {root_name: -1, **{info[12].decode(): info[0] for info in infos}}
        info_by_name = {info[1].decode(): info for info in infos}
        visited = set()

        def visit(name, parent_element, joint=None):
            if name in visited:
                raise ValueError(f"URDF link tree contains a cycle at {name}")
            visited.add(name)
            link = indices[name]
            if joint is None:
                pos, quat = self.base_pose(body)
            else:
                origin = joint.find("origin")
                pos = _vector(origin, "xyz", "0 0 0", scale)
                quat = self.p.getQuaternionFromEuler(_vector(origin, "rpy", "0 0 0"))
            element = self.body(parent_element, body, link, pos, quat)
            if joint is None:
                self.free_joint(element, body)
            elif joint.get("type") != "fixed":
                info = info_by_name[joint.get("name")]
                joint_name = f"j{body}_{link}"
                axis = _vector(joint.find("axis"), "xyz", "1 0 0")
                if np.linalg.norm(axis) <= 1e-12:
                    raise ValueError(f"URDF joint '{joint.get('name')}' has a zero axis")
                damping = float(info[6])
                if body == self.world.robot:
                    # getJointInfo retains source URDF damping after Bullet's
                    # changeDynamics call. Mirror the effective values applied
                    # by World.reset, including its omni-wheel override.
                    damping = .001
                    if (self.world.robot_settings.get("drive", {}).get("type") == "omni"
                            and any(wheel["index"] == link for wheel in self.world._wheels)):
                        damping = .0001
                attrs = {"name": joint_name, "type": "slide" if joint.get("type") == "prismatic" else "hinge",
                         "axis": _numbers(axis), "damping": _numbers([damping]), "frictionloss": _numbers([info[7]])}
                limit = joint.find("limit")
                if joint.get("type") != "continuous" and limit is not None:
                    length_scale = scale if joint.get("type") == "prismatic" else 1.
                    attrs.update(limited="true", range=_numbers([float(limit.get("lower")) * length_scale,
                                                                 float(limit.get("upper")) * length_scale]))
                else:
                    attrs["limited"] = "false"
                ET.SubElement(element, "joint", **attrs)
                self.joints[body, link] = joint_name
                self.initial[body, link] = self.p.getJointState(body, link, physicsClientId=self.client)[:2]
                if body == self.world.robot:
                    actuator_name = f"a{body}_{link}"
                    ET.SubElement(self.actuator, "general", name=actuator_name, joint=joint_name,
                                  dyntype="none", gaintype="fixed", gainprm="1", biastype="affine",
                                  biasprm="0 0 0", ctrllimited="false", forcelimited="true", forcerange="-1 1")
                    self.actuators[body, link] = actuator_name
            for collision in links[name].findall("collision"):
                self.collision(element, body, link, collision, path.parent, scale, links[name].get("concave") == "yes")
            for child_joint in children.get(name, []):
                visit(child_joint.find("child").get("link"), element, child_joint)

        visit(root_name, self.worldbody)
        if visited != set(links):
            raise ValueError(f"URDF contains disconnected links: {path}")
        # Existing World imports disable all intra-URDF collision. Preserve
        # this for non-adjacent links as well as MuJoCo's parent exclusions.
        names = [self.bodies[body, index] for index in indices.values()]
        for index, first in enumerate(names):
            for second in names[index+1:]:
                ET.SubElement(self.contact, "exclude", body1=first, body2=second)

    def primitive(self, body, item):
        element = self.body(self.worldbody, body, -1, *self.base_pose(body))
        self.free_joint(element, body)
        if not item.get("collision", True):
            return
        collision = ET.Element("collision")
        geometry = ET.SubElement(collision, "geometry")
        shape = item.get("shape", "box")
        if shape == "box":
            ET.SubElement(geometry, "box", size=_numbers(item.get("size", [.1, .1, .1])))
        else:
            attrs = {"radius": str(item.get("radius", .05))}
            if shape != "sphere":
                attrs["length"] = str(item.get("height", .1))
            ET.SubElement(geometry, shape, **attrs)
        self.collision(element, body, -1, collision, Path.cwd(), 1.)

    def _prepare_box_flex_collisions(self):
        """Use exact cuboid hulls for the generic convex/flex collider.

        MuJoCo 3.17's specialized box/triangle test can miss a box face crossing
        a large triangle when neither triangle vertices nor box corners lie
        inside the thin shell. A mesh with the same eight corners dispatches
        to GJK instead. No dimensions, mass, inertia or contact margin change.
        Static boxes need no conversion: our rigid flexes are static too.
        """
        if not self.flexes:
            return
        meshes = {}
        faces = (0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6,
                 0, 1, 5, 0, 5, 4, 2, 6, 7, 2, 7, 3,
                 0, 4, 6, 0, 6, 2, 1, 3, 7, 1, 7, 5)
        for geom in self.moving_boxes:
            size = tuple(float(value) for value in geom.get("size").split())
            if size not in meshes:
                name = f"cuboid{len(meshes)}"
                vertices = [size[axis] * sign for x in (-1, 1) for y in (-1, 1)
                            for z in (-1, 1) for axis, sign in enumerate((x, y, z))]
                ET.SubElement(self.asset, "mesh", name=name, vertex=_numbers(vertices),
                              face=" ".join(map(str, faces)), inertia="convex")
                meshes[size] = name
            geom.set("type", "mesh")
            geom.set("mesh", meshes[size])
            del geom.attrib["size"]

    def finish(self):
        import mujoco
        self._prepare_box_flex_collisions()
        xml = ET.tostring(self.root, encoding="unicode")
        model = mujoco.MjModel.from_xml_string(xml, self.assets)
        data = mujoco.MjData(model)
        # mjData owns the native worker pool; mj_deleteData joins and destroys
        # it. Rebuilding/resetting the scene therefore cannot orphan workers.
        cpu_threads = physics_worker_count(self.world.config.physics_cpu_threads)
        if cpu_threads:
            mujoco.mju_threadpool(data, cpu_threads)
        ids = lambda mapping, kind: {key: mujoco.mj_name2id(model, kind, name) for key, name in mapping.items()}
        body_ids = ids(self.bodies, mujoco.mjtObj.mjOBJ_BODY)
        joint_ids = ids(self.joints, mujoco.mjtObj.mjOBJ_JOINT)
        free_joints = ids(self.free, mujoco.mjtObj.mjOBJ_JOINT)
        actuator_ids = ids(self.actuators, mujoco.mjtObj.mjOBJ_ACTUATOR)
        geom_keys = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name): key for name, key in self.geoms.items()}
        flex_keys = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, name): key for name, key in self.flexes.items()}
        for key, (position, velocity) in self.initial.items():
            joint = joint_ids[key]
            data.qpos[model.jnt_qposadr[joint]] = position
            data.qvel[model.jnt_dofadr[joint]] = velocity
        # Free body velocities are generally zero during import; retaining
        # them makes compilation valid after a paused state as well.
        for body, joint in free_joints.items():
            linear, angular = self.p.getBaseVelocity(body, physicsClientId=self.client)
            dof = model.jnt_dofadr[joint]
            base_pose = self.base_pose(body)
            rotation = np.asarray(self.p.getMatrixFromQuaternion(base_pose[1])).reshape(3, 3)
            # Bullet reports velocity at the COM; MuJoCo's free translation
            # coordinate belongs to the link origin.
            offset = rotation @ np.asarray(self.dynamics_for(body, -1)[3])
            data.qvel[dof:dof+3] = np.asarray(linear) - np.cross(angular, offset)
            # MuJoCo's free rotational velocity is in the body frame.
            data.qvel[dof+3:dof+6] = rotation.T @ angular
        mujoco.mj_forward(model, data)
        return Scene(model, data, body_ids, joint_ids, geom_keys, flex_keys,
                     free_joints, actuator_ids, self.dynamics, xml, cpu_threads)


def compile_scene(world) -> Scene:
    """Compile a loaded World without advancing either physics engine."""
    compiler = _Compiler(world)
    for body, item, directory in world._export_items:
        if "urdf" in item:
            compiler.urdf(body, (Path(directory) / item["urdf"]).resolve(), float(item.get("scale", 1.)))
        else:
            compiler.primitive(body, item)
    compiler.urdf(world.robot, Path(world.config.robot_urdf).resolve())
    return compiler.finish()


compile_world = compile_scene
