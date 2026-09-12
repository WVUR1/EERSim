"""Export a loaded EERSim scene to the pinned ovphysx 0.5.11 runtime.

Collision leaves are partitioned without changing their union. Robot links form
a reduced-coordinate articulation; field mechanisms retain regular joints.
This module only authors the scene: the backend owns time, control and sensors.
"""
from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull


def number(x):
    x = float(x)
    if not math.isfinite(x):
        raise ValueError("Nonfinite geometry or physical property")
    return format(x, ".17g")


def vector(values):
    return "(" + ", ".join(number(v) for v in values) + ")"


def quat(q):
    return vector([q[3], *q[:3]])


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def hull_info(points):
    hull = ConvexHull(np.unique(points, axis=0))
    vertices = hull.points[hull.vertices]
    hull = ConvexHull(vertices)
    planes = {}
    for simplex, equation in zip(hull.simplices, hull.equations):
        planes.setdefault(tuple(np.round(equation, 8)), set()).update(map(int, simplex))
    return hull, {"vertices": len(hull.vertices), "faces": len(planes),
                  "max_face_vertices": max(map(len, planes.values()))}


def eligible(info):
    return info["vertices"] <= 64 and info["faces"] <= 64 and info["max_face_vertices"] <= 32


def bisect_hull(hull):
    """Intersect a convex polyhedron with the two halves of an axial plane.

    Every new point is on an existing hull edge. The two closed halfspaces
    partition the original hull, sharing only their zero-volume boundary.
    """
    points = hull.points
    axis = int(np.argmax(np.ptp(points, axis=0)))
    cut = float((points[:, axis].min() + points[:, axis].max()) / 2)
    edges = {tuple(sorted((int(face[i]), int(face[(i + 1) % 3]))))
             for face in hull.simplices for i in range(3)}
    intersections = []
    for a, b in edges:
        da, db = points[a, axis] - cut, points[b, axis] - cut
        if da * db < 0:
            point = points[a] + da / (da - db) * (points[b] - points[a])
            point[axis] = cut
            intersections.append(point)
    if not intersections:
        raise ValueError("Could not partition oversized convex hull")
    return [np.concatenate([points[points[:, axis] <= cut], intersections]),
            np.concatenate([points[points[:, axis] >= cut], intersections])]


def split_exact(points, policy):
    original, original_info = hull_info(points)
    queue = [original.points]
    result, infos = [], []
    while queue:
        candidate, info = hull_info(queue.pop())
        if eligible(info):
            result.append(candidate)
            infos.append(info)
        elif policy != "split-exact":
            raise ValueError(f"Original convex leaf exceeds GPU cooking limits: {info}; "
                             "use --convex-policy split-exact for explicit exact subdivision")
        else:
            queue.extend(bisect_hull(candidate))
            if len(queue) + len(result) > 512:
                raise ValueError("Convex subdivision exceeded bounded 512-piece limit")
    volume_error = abs(sum(h.volume for h in result) - original.volume)
    rng = np.random.default_rng(0)
    directions = np.concatenate([np.eye(3), -np.eye(3), rng.normal(size=(512, 3))])
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    support = original.points @ directions.T
    split_points = np.concatenate([h.points for h in result])
    support_error = float(np.max(np.abs(support.max(axis=0) - (split_points @ directions.T).max(axis=0))))
    outside_error = float(np.max(split_points @ original.equations[:, :3].T + original.equations[:, 3]))
    if volume_error > max(1e-16, original.volume * 1e-9) or support_error > 1e-9 or outside_error > 1e-9:
        raise ValueError(f"Exact subdivision validation failed: volume={volume_error}, "
                         f"support={support_error}, outside={outside_error}")
    return result, {"original": original_info, "output": infos,
                    "volume_m3": float(original.volume), "volume_error_m3": float(volume_error),
                    "support_error_m": support_error, "outside_error_m": max(0., outside_error),
                    "support_directions": len(directions)}


def read_mesh(path):
    if path.suffix.lower() == ".obj":
        vertices, faces = [], []
        for line in path.read_text(encoding="utf-8", errors="strict").splitlines():
            fields = line.split()
            if not fields:
                continue
            if fields[0] == "v":
                vertices.append([float(v) for v in fields[1:4]])
            elif fields[0] == "f":
                indices = [int(f.split("/")[0]) for f in fields[1:]]
                indices = [i - 1 if i > 0 else len(vertices) + i for i in indices]
                faces.extend((indices[0], indices[i], indices[i + 1]) for i in range(1, len(indices) - 1))
        return np.asarray(vertices), np.asarray(faces, dtype=int)
    if path.suffix.lower() == ".stl":
        data = path.read_bytes()
        if len(data) < 84 or len(data) != 84 + 50 * struct.unpack_from("<I", data, 80)[0]:
            raise ValueError(f"Only binary STL is supported by this bounded exporter: {path}")
        points = [struct.unpack_from("<12fH", data, offset)[3:12] for offset in range(84, len(data), 50)]
        cloud = np.asarray(points).reshape(-1, 3)
        vertices, indices = np.unique(cloud, axis=0, return_inverse=True)
        return vertices, indices.reshape(-1, 3)
    raise ValueError(f"Unsupported collision mesh format: {path}")


def validate_joint_features(path):
    """Fail before export instead of silently discarding joint behavior."""
    for joint in ET.parse(path).getroot().findall('joint'):
        name, kind = joint.get('name', '<unnamed>'), joint.get('type')
        if kind not in ('fixed', 'continuous', 'revolute', 'prismatic'):
            raise ValueError(f"PhysX does not support URDF joint '{name}' of type '{kind}'; use fixed, continuous, revolute or prismatic joints")
        for feature in ('mimic', 'safety_controller', 'calibration'):
            if joint.find(feature) is not None:
                raise ValueError(f"PhysX does not implement URDF {feature} behavior on joint '{name}'; model and validate that mechanism explicitly before using this backend")



class Exporter:
    def __init__(self, world, output, policy, convex_cook_scale=100., *, backend='gpu',
                 iterations=100, articulation=True):
        if backend not in ('cpu', 'gpu'):
            raise ValueError("PhysX export backend must be 'cpu' or 'gpu'")
        if isinstance(iterations, bool) or int(iterations) != iterations or not 1 <= iterations <= 255:
            raise ValueError('PhysX position iterations must be an integer from 1 to 255')
        self.iterations = int(iterations)
        self.articulation = bool(articulation)
        self.p, self.world, self.output, self.policy = world._physics, world, Path(output), policy
        # Observe only contacts consumed by tires/sensors and placement checks.
        # All other bodies retain their physical collisions in the solver.
        self.contact_bodies = {world.robot} | {item['id'] for item in world.field_objects
                                              if item['scoring'].get('type') == 'pin'}
        self._dynamics_overrides = {}
        self._articulation_root = None
        self.convex_cook_scale = float(convex_cook_scale)
        if not math.isfinite(self.convex_cook_scale) or self.convex_cook_scale < 1:
            raise ValueError('convex_cook_scale must be finite and at least1')
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / "geometry").mkdir(exist_ok=True)
        self.lines = ['#usda 1.0', '(defaultPrim = "World"', ' metersPerUnit = 1',
                      ' kilogramsPerUnit = 1', ' upAxis = "Z")', 'def Xform "World"', '{']
        self.manifest = {"schema_version": 1, "purpose": "EERSim PhysX runtime scene",
            "ovphysx_version": "0.5.11", "physics_hz": world.config.physics_hz,
            "backend": backend, "scene_path": "/World/physicsScene",
            "convex_cook_scale": self.convex_cook_scale,
            "gravity_m_s2": world.config.gravity, "solver": {"PhysX": "TGS", "position_iterations": self.iterations,
                "velocity_iterations": 1, "Bullet": "sequential impulse, maximum100iterations"},
            "sources": {}, "meshes": {}, "actors": [], "joints": [], "counts": {},
            "differences": [
                "Control, contact tire forces and observations are supplied by the runtime adapter, not this exporter.",
                "Robot uses a reduced-coordinate articulation; field mechanisms use regular PhysX joints.",
                "Bullet rolling/spinning friction has no direct USD material mapping and is recorded but not applied.",
                "PhysX and Bullet use different contact/friction/solver/sleep algorithms; no trajectory equivalence claim.",
                "Native PhysX sleeping only; EERSim's numerical-rest helper is not implemented.",
                "Convex margins map to per-shape restOffset; this is a declared contact-model approximation.",
                "Exact convex subdivision preserves the union but introduces additional internal seams.",
                "PhysX cooking runtime statistics must still verify GPU eligibility and absence of warnings."]}
        self.mesh_cache = {}
        self.counts = Counter()
        self.lines += ['def PhysicsScene "physicsScene" (prepend apiSchemas = ["PhysxSceneAPI"]) {',
            ' vector3f physics:gravityDirection = (0, 0, -1)',
            f' float physics:gravityMagnitude = {number(world.config.gravity)}',
            f' bool physxScene:enableGPUDynamics = {str(backend == "gpu").lower()}',
            f' uniform token physxScene:broadphaseType = {json.dumps("GPU" if backend == "gpu" else "MBP")}',
            ' uniform token physxScene:solverType = "TGS"',
            f' uniform uint physxScene:minPositionIterationCount = {self.iterations}',
            f' uniform uint physxScene:maxPositionIterationCount = {self.iterations}',
            ' uniform uint physxScene:minVelocityIterationCount = 1',
            ' uniform uint physxScene:maxVelocityIterationCount = 1', '}']

    def dynamics(self, body, index):
        override = self._dynamics_overrides.get((body, index))
        if override is not None:
            return override
        values = list(self.p.getDynamicsInfo(body, index, physicsClientId=self.world.client))
        if index < 0 and body in self.world.base_inertial_frames:
            # getDynamicsInfo leaves these two fields uninitialized for this
            # Bullet build's maximal-coordinate primitives. World captures the
            # actual transform at load time, before anything moves.
            values[3], values[4] = self.world.base_inertial_frames[body]
        return tuple(values)

    def source(self, path):
        path = Path(path).resolve()
        self.manifest["sources"][str(path)] = sha(path)

    def pose(self, position, orientation, scale=None):
        self.lines += [f' double3 xformOp:translate = {vector(position)}',
                       f' quatd xformOp:orient = {quat(orientation)}']
        ops = ['xformOp:translate', 'xformOp:orient']
        if scale is not None:
            self.lines.append(f' double3 xformOp:scale = {vector(scale)}')
            ops.append('xformOp:scale')
        self.lines.append(' uniform token[] xformOpOrder = ' + json.dumps(ops))

    def material(self, name, dynamics):
        self.lines += [f'def Material "{name}" (prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]) {{',
            f' float physics:staticFriction = {number(dynamics[1])}',
            f' float physics:dynamicFriction = {number(dynamics[1])}',
            f' float physics:restitution = {number(dynamics[5])}',
            ' uniform token physxMaterial:frictionCombineMode = "multiply"',
            ' uniform token physxMaterial:restitutionCombineMode = "multiply"', '}']

    def mesh(self, path, scale, concave):
        cook_scale = 1. if concave else self.convex_cook_scale
        key = (str(path.resolve()), tuple(scale), concave, cook_scale)
        if key in self.mesh_cache:
            return self.mesh_cache[key]
        self.source(path)
        vertices, faces = read_mesh(path)
        vertices = vertices * np.asarray(scale)
        digest = hashlib.sha256((sha(path) + repr(scale) + str(concave) + repr(cook_scale)).encode()).hexdigest()[:20]
        if concave:
            pieces = [(vertices, faces)]
            audit = {"triangle_mesh": True, "vertices": len(vertices), "triangles": len(faces)}
        else:
            hulls, audit = split_exact(vertices, self.policy)
            pieces = []
            for h in hulls:
                faces = h.simplices.copy()
                for i, face in enumerate(faces):
                    a, b, c = h.points[face]
                    if np.dot(np.cross(b - a, c - a), h.equations[i, :3]) < 0:
                        faces[i, 1], faces[i, 2] = faces[i, 2], faces[i, 1]
                pieces.append((h.points, faces))
        output_files = []
        audit['convex_cook_scale'] = cook_scale
        audit['compensating_collider_scale'] = 1. / cook_scale
        for i, (points, faces) in enumerate(pieces):
            # The convex cooker consumes raw points, retaining positive transform
            # magnitude separately in PxMeshScale. Precondition only that raw
            # coordinate space; actor transforms, inertia and contact offsets stay
            # in metres. This reduces the cooker's absolute plane-tolerance floor.
            cook_points = points * cook_scale
            quantization_error = float(np.max(np.abs(cook_points.astype(np.float32).astype(float) / cook_scale - points)))
            if quantization_error > 1e-7:
                raise ValueError(f"USD float32 point quantization exceeds 0.1 micrometre: {path}")
            audit['max_float32_coordinate_error_m'] = max(audit.get('max_float32_coordinate_error_m', 0.), quantization_error)
            filename = f"{digest}_{i}.usda"
            schemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxCollisionAPI"]
            if not concave:
                schemas.append("PhysxConvexHullCollisionAPI")
            text = ['#usda 1.0', '(defaultPrim = "Geometry")',
                    'def Mesh "Geometry" (prepend apiSchemas = ' + json.dumps(schemas) + ') {',
                    ' uniform token subdivisionScheme = "none"',
                    ' point3f[] points = [' + ', '.join(vector(v) for v in cook_points) + ']',
                    ' int[] faceVertexCounts = [' + ', '.join('3' for _ in faces) + ']',
                    ' int[] faceVertexIndices = [' + ', '.join(str(int(v)) for v in faces.flatten()) + ']',
                    f' uniform token physics:approximation = "{"none" if concave else "convexHull"}"']
            if not concave:
                text += [' int physxConvexHullCollision:hullVertexLimit = 64',
                         ' float physxConvexHullCollision:minThickness = 0']
            text.append('}')
            destination = self.output / 'geometry' / filename
            destination.write_text('\n'.join(text) + '\n', encoding='utf-8')
            output_files.append('geometry/' + filename)
        audit.update(source_sha256=sha(path), output_files=output_files,
                     output_sha256={f: sha(self.output / f) for f in output_files})
        self.manifest['meshes'][repr(key)] = audit
        self.mesh_cache[key] = output_files
        return output_files

    def actor(self, body, index, collisions, scale, base_path, label, robot=False, filtered_targets=None):
        p, c = self.p, self.world.client
        d = self.dynamics(body, index)
        name = f'{"rigid" if d[0] > 0 else "static"}_{body}_{"base" if index < 0 else index}'
        material = f'material_{body}_{"base" if index < 0 else index}'
        self.material(material, d)
        if index < 0:
            com_pose = p.getBasePositionAndOrientation(body, physicsClientId=c)
            loaded_dynamics = p.getDynamicsInfo(body, index, physicsClientId=c)
            inertial = self.world.base_inertial_frames.get(body, (loaded_dynamics[3], loaded_dynamics[4]))
            position, orientation = p.multiplyTransforms(*com_pose, *p.invertTransform(*inertial))
        else:
            state = p.getLinkState(body, index, physicsClientId=c)
            position, orientation = state[4:6]
        schemas = ['PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'PhysxRigidBodyAPI'] if d[0] > 0 else []
        if d[0] > 0 and body in self.contact_bodies:
            schemas.append('PhysxContactReportAPI')
        is_articulation_root = self._articulation_root == '/World/' + name
        if is_articulation_root:
            schemas += ['PhysicsArticulationRootAPI', 'PhysxArticulationAPI']
        if filtered_targets:
            schemas.append('PhysicsFilteredPairsAPI')
        metadata = f' (prepend apiSchemas = {json.dumps(schemas)})' if schemas else ''
        self.lines.append(f'def Xform "{name}"{metadata} {{')
        self.pose(position, orientation)
        if filtered_targets:
            self.lines.append(' rel physics:filteredPairs = [' + ', '.join('<' + x + '>' for x in filtered_targets) + ']')
            self.counts['filtered_body_pairs'] += len(filtered_targets)
        if d[0] > 0:
            self.lines += [f' float physics:mass = {number(d[0])}',
                f' point3f physics:centerOfMass = {vector(d[3])}',
                f' float3 physics:diagonalInertia = {vector(d[2])}',
                f' quatf physics:principalAxes = {quat(d[4])}',
                f' int physxRigidBody:solverPositionIterationCount = {self.iterations}',
                ' int physxRigidBody:solverVelocityIterationCount = 1',
                ' float physxContactReport:threshold = 0',
                f' float physxRigidBody:linearDamping = {number(.02 if robot else .04)}',
                f' float physxRigidBody:angularDamping = {number(.02 if robot else .04)}']
        if is_articulation_root:
            self.lines += [' bool physxArticulation:enabledSelfCollisions = false',
                           f' int physxArticulation:solverPositionIterationCount = {self.iterations}',
                           ' int physxArticulation:solverVelocityIterationCount = 1']
        shapes = 0
        for ci, collision in enumerate(collisions):
            origin = collision.find('origin')
            xyz = [float(x) * scale for x in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(x) for x in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            geometry = collision.find('geometry')
            primitive = next(iter(geometry))
            part_orientation = p.getQuaternionFromEuler(rpy)
            rest = 0.
            refs = [None]
            primitive_scale = None
            if primitive.tag == 'mesh':
                source = (base_path / primitive.get('filename')).resolve()
                mesh_scale = [float(x) * scale for x in primitive.get('scale', '1 1 1').split()]
                concave = collision.get('concave') == 'yes'
                if concave and d[0] > 0:
                    raise ValueError(f'Dynamic triangle collider is unsupported: {label}')
                refs = self.mesh(source, mesh_scale, concave)
                rest = float(d[11]) if not concave else 0.
                if not concave and self.convex_cook_scale != 1:
                    primitive_scale = [1. / self.convex_cook_scale] * 3
            elif primitive.tag == 'box':
                primitive_scale = [float(x) * scale for x in primitive.get('size').split()]
            elif primitive.tag not in ('sphere', 'cylinder', 'capsule'):
                raise ValueError(f'Unsupported primitive: {primitive.tag}')
            for pi, reference in enumerate(refs):
                kind = {'box': 'Cube', 'sphere': 'Sphere', 'cylinder': 'Cylinder', 'capsule': 'Capsule', 'mesh': 'Mesh'}[primitive.tag]
                metadata = ['prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]']
                if reference:
                    metadata.append(f'prepend references = @{reference}@</Geometry>')
                self.lines.append(f'def {kind} "collider_{ci}_{pi}" (' + '\n'.join(metadata) + ') {')
                self.pose(xyz, part_orientation, primitive_scale)
                if primitive.tag == 'box':
                    self.lines.append(' double size = 1')
                elif primitive.tag in ('sphere', 'cylinder', 'capsule'):
                    self.lines.append(f' double radius = {number(float(primitive.get("radius")) * scale)}')
                    if primitive.tag != 'sphere':
                        self.lines += [f' double height = {number(float(primitive.get("length")) * scale)}', ' uniform token axis = "Z"']
                self.lines += [f' rel material:binding:physics = </World/{material}>',
                    f' float physxCollision:restOffset = {number(rest)}',
                    f' float physxCollision:contactOffset = {number(rest + .002)}', '}']
                self.counts['collision_shapes'] += 1
                self.counts['triangle_meshes' if primitive.tag == 'mesh' and collision.get('concave') == 'yes' else ('convex_meshes' if primitive.tag == 'mesh' else primitive.tag)] += 1
                shapes += 1
        self.lines.append('}')
        self.counts['dynamic_actors' if d[0] > 0 else 'static_link_parents'] += 1
        self.manifest['actors'].append({'path': '/World/' + name, 'label': label, 'body_id': body,
            'link_index': index, 'mass_kg': d[0], 'inertia_kg_m2': d[2], 'local_com': d[3],
            'principal_axes': d[4], 'position': position, 'orientation': orientation,
            'robot': bool(robot), 'articulation_link': bool(robot and self.articulation),
            'friction': d[1], 'restitution': d[5], 'unmapped_rolling_friction': d[6],
            'unmapped_spinning_friction': d[7], 'shapes': shapes})
        return '/World/' + name

    def urdf(self, body, path, scale=1., label='', robot=False):
        p, c = self.p, self.world.client
        self.source(path)
        tree = ET.parse(path).getroot()
        links = {x.get('name'): x for x in tree.findall('link')}
        info = [p.getJointInfo(body, i, physicsClientId=c) for i in range(p.getNumJoints(body, physicsClientId=c))]
        root_name = next(n for n in links if n not in {x.find('child').get('link') for x in tree.findall('joint')})
        indices = {root_name: -1, **{x[12].decode(): x[0] for x in info}}
        joint_info = {entry[1].decode(): entry for entry in info}
        fixed_base = bool(robot and self.articulation and self.dynamics(body, -1)[0] == 0)
        if fixed_base:
            # Bullet removes the base mass/inertia for useFixedBase. PhysX marks
            # a fixed-base articulation with a world joint instead, so restore
            # the authored positive base mass tensor before adding that joint.
            inertial = links[root_name].find('inertial')
            if inertial is None or inertial.find('mass') is None or inertial.find('inertia') is None:
                raise ValueError('A fixed-base PhysX robot needs an explicit base mass and inertia in its URDF')
            mass = float(inertial.find('mass').get('value'))
            if not math.isfinite(mass) or mass <= 0:
                raise ValueError('The fixed-base robot must have a positive authored base mass')
            inertia = inertial.find('inertia')
            tensor = np.array([[float(inertia.get(key, '0')) for key in row]
                               for row in [('ixx', 'ixy', 'ixz'), ('ixy', 'iyy', 'iyz'), ('ixz', 'iyz', 'izz')]])
            origin = inertial.find('origin')
            rpy = [float(x) for x in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            rotation = np.asarray(p.getMatrixFromQuaternion(p.getQuaternionFromEuler(rpy))).reshape(3, 3)
            tensor = rotation @ tensor @ rotation.T
            diagonal, axes = np.linalg.eigh(tensor)
            if not np.all(np.isfinite(diagonal)) or np.any(diagonal <= 0):
                raise ValueError('The fixed-base robot inertia must be positive definite')
            if np.linalg.det(axes) < 0:
                axes[:, 0] *= -1
            # Matrix-to-quaternion through a stable trace branch; axes are an
            # orthonormal rotation from principal frame into the link frame.
            from scipy.spatial.transform import Rotation
            principal = Rotation.from_matrix(axes).as_quat().tolist()
            centre = [float(x) * scale for x in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            dynamics = list(self.dynamics(body, -1))
            dynamics[0], dynamics[2], dynamics[3], dynamics[4] = mass, (diagonal * scale ** 2).tolist(), centre, principal
            self._dynamics_overrides[(body, -1)] = tuple(dynamics)
        paths = {name: '/World/' + f'{"rigid" if self.dynamics(body, index)[0] > 0 else "static"}_{body}_{"base" if index < 0 else index}'
                 for name, index in indices.items()}
        if robot and self.articulation:
            if any(self.dynamics(body, index)[0] <= 0 for index in indices.values()):
                raise ValueError('PhysX robot articulation requires positive mass on every link; combine massless fixed links in the source URDF')
            self._articulation_root = None if fixed_base else paths[root_name]
            self.manifest['robot'] = {'body_id': body, 'base_path': paths[root_name],
                'articulation_root': f'/World/robot_fixed_base_{body}' if fixed_base else paths[root_name],
                'fixed_base': fixed_base, 'source_urdf': str(path.resolve()),
                'links': {name: {'index': index, 'path': paths[name]} for name, index in indices.items()}}
        filter_targets = {path: [] for path in paths.values()}
        ordered_paths = list(paths.values())
        for i, first in enumerate(ordered_paths):
            for second in ordered_paths[i + 1:]:
                # PhysX's filter loader requires pair.first to resolve to a
                # native body/shape. A plain static parent Xform does not;
                # only pair.second supports descent to collider children.
                # Author static/dynamic exclusions on the dynamic body.
                source, target = (second, first) if first.startswith('/World/static_') and second.startswith('/World/rigid_') else (first, second)
                filter_targets[source].append(target)
        for ordinal, (name, index) in enumerate(indices.items()):
            collisions = links[name].findall('collision')
            if links[name].get('concave') == 'yes':
                for element in collisions:
                    if element.find('geometry/mesh') is not None:
                        element.set('concave', 'yes')
            paths[name] = self.actor(body, index, collisions, scale, path.parent, label + '/' + name, robot,
                                     filtered_targets=filter_targets[paths[name]])
        # World's URDF imports do not set URDF_USE_SELF_COLLISION. The
        # EXCLUDE_PARENT flag on its robot load does not enable self collision:
        # Bullet's URDF2Bullet sets hasSelfCollision from USE_SELF_COLLISION only.
        # Preserve that complete intra-body exclusion when links become separate
        # regular PhysX actors, including overlapping nonadjacent mast/carriage.
        self.manifest.setdefault('collision_filters', []).append({'body_id': body,
            'policy': 'all intra-URDF link pairs disabled, matching Bullet load flags',
            'actors': list(paths.values()), 'authored_targets': filter_targets})
        if fixed_base:
            root_path = self.manifest['robot']['articulation_root']
            self.lines += [f'def PhysicsFixedJoint "{root_path.rsplit("/", 1)[-1]}" (prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]) {{',
                f' rel physics:body0 = <{paths[root_name]}>',
                ' bool physxArticulation:enabledSelfCollisions = false',
                f' int physxArticulation:solverPositionIterationCount = {self.iterations}',
                ' int physxArticulation:solverVelocityIterationCount = 1', '}']
        for ji, joint in enumerate(tree.findall('joint')):
            jt = joint.get('type')
            kind = {'fixed': 'PhysicsFixedJoint', 'continuous': 'PhysicsRevoluteJoint',
                    'revolute': 'PhysicsRevoluteJoint', 'prismatic': 'PhysicsPrismaticJoint'}.get(jt)
            if kind is None:
                raise ValueError(f'Unsupported URDF joint {jt}')
            origin = joint.find('origin')
            xyz = [float(x) * scale for x in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
            rpy = [float(x) for x in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
            axis = joint.find('axis')
            axis = np.asarray([float(x) for x in (axis.get('xyz', '1 0 0') if axis is not None else '1 0 0').split()])
            axis /= np.linalg.norm(axis)
            # USD joint's X axis is rotated to the arbitrary URDF joint axis.
            if axis[0] < -1 + 1e-10:
                align = [0., 1., 0., 0.]
            else:
                align = np.asarray([0., -axis[2], axis[1], 1 + axis[0]])
                align /= np.linalg.norm(align)
            _, q0 = p.multiplyTransforms([0, 0, 0], p.getQuaternionFromEuler(rpy), [0, 0, 0], align)
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            entry = joint_info[joint.get('name')]
            current = p.getJointState(body, entry[0], physicsClientId=c)
            driven = bool(robot and self.articulation and jt != 'fixed')
            drive_axis = 'linear' if jt == 'prismatic' else 'angular'
            schemas = [f'PhysicsDriveAPI:{drive_axis}', f'PhysicsJointStateAPI:{drive_axis}'] if driven else []
            metadata = f' (prepend apiSchemas = {json.dumps(schemas)})' if schemas else ''
            joint_path = f'/World/joint_{body}_{ji}'
            self.lines += [f'def {kind} "joint_{body}_{ji}"{metadata} {{',
                f' rel physics:body0 = <{paths[parent]}>', f' rel physics:body1 = <{paths[child]}>',
                f' point3f physics:localPos0 = {vector(xyz)}', f' quatf physics:localRot0 = {quat(q0)}',
                ' point3f physics:localPos1 = (0, 0, 0)', f' quatf physics:localRot1 = {quat(align)}',
                ' bool physics:collisionEnabled = false']
            if jt != 'fixed':
                self.lines.append(' uniform token physics:axis = "X"')
                limit = joint.find('limit')
                if jt != 'continuous' and limit is not None:
                    factor = 180 / math.pi if jt == 'revolute' else scale
                    self.lines += [f' float physics:lowerLimit = {number(float(limit.get("lower")) * factor)}',
                                   f' float physics:upperLimit = {number(float(limit.get("upper")) * factor)}']
            if driven:
                factor = 180 / math.pi if jt != 'prismatic' else 1.
                self.lines += [f' uniform token drive:{drive_axis}:physics:type = "force"',
                    f' float drive:{drive_axis}:physics:stiffness = 0',
                    f' float drive:{drive_axis}:physics:damping = 0',
                    f' float drive:{drive_axis}:physics:maxForce = 0',
                    f' float drive:{drive_axis}:physics:targetPosition = {number(current[0] * factor)}',
                    f' float drive:{drive_axis}:physics:targetVelocity = 0',
                    f' float state:{drive_axis}:physics:position = {number(current[0] * factor)}',
                    f' float state:{drive_axis}:physics:velocity = {number(current[1] * factor)}']
            self.lines.append('}')
            self.counts['joints'] += 1
            self.manifest['joints'].append({'path': joint_path, 'name': joint.get('name'), 'type': jt,
                'body_id': body, 'joint_index': entry[0], 'parent_link_index': entry[16], 'child_link_index': entry[0],
                'parent': paths[parent], 'child': paths[child], 'robot': bool(robot),
                'articulation': bool(robot and self.articulation), 'drive_axis': drive_axis if driven else None,
                'axis': axis.tolist(), 'origin_xyz': xyz, 'origin_rpy': rpy,
                'local_position0': xyz, 'local_rotation0': list(q0),
                'local_position1': [0., 0., 0.], 'local_rotation1': list(align),
                'lower': entry[8], 'upper': entry[9], 'max_force': entry[10], 'max_velocity': entry[11],
                'damping': entry[6], 'friction': entry[7], 'initial_position': current[0], 'initial_velocity': current[1]})

    def primitive_body(self, item, body):
        link = ET.Element('collision')
        geometry = ET.SubElement(link, 'geometry')
        kind = item.get('shape', 'box')
        if kind == 'box':
            ET.SubElement(geometry, 'box', size=' '.join(map(str, item.get('size', [.1, .1, .1]))))
        elif kind in ('sphere', 'cylinder', 'capsule'):
            attributes = {'radius': str(item.get('radius', .05))}
            if kind != 'sphere':
                attributes['length'] = str(item.get('height', .1))
            ET.SubElement(geometry, kind, **attributes)
        else:
            raise ValueError(f'Unsupported generated field primitive {kind}')
        self.actor(body, -1, [link] if item.get('collision', True) else [], 1., self.output, item.get('name', ''))

    def finish(self):
        self.lines.append('}')
        scene = self.output / 'field.usda'
        scene.write_text('\n'.join(self.lines) + '\n', encoding='utf-8')
        self.manifest['contact_sensor_paths'] = [actor['path'] for actor in self.manifest['actors']
            if actor['mass_kg'] > 0 and actor['body_id'] in self.contact_bodies]
        self.manifest['counts'] = dict(self.counts)
        self.manifest['scene_sha256'] = sha(scene)
        self.manifest['convex_policy'] = self.policy
        self.manifest['exporter_sha256'] = sha(__file__) if Path(__file__).is_file() else None
        from . import __version__
        self.manifest['eersim_version'] = __version__
        self.manifest['scene_file'] = 'field.usda'
        (self.output / 'field.manifest.json').write_text(json.dumps(self.manifest, indent=2) + '\n', encoding='utf-8')
        return self.manifest


def export_world(world, output: Path, *, backend='gpu', iterations=100,
                 articulation=True, convex_policy='split-exact', convex_cook_scale=100.) -> dict:
    """Author a loaded, unstepped World and return its body/joint manifest.

    No simulation or GPU initialization occurs here. ``field.usda`` and its
    relative ``geometry/`` references are written below ``output`` together with
    ``field.manifest.json``. The source World supplies actual import-time poses,
    inertia and dynamics; it must still use the loader's Bullet API while this
    function runs. Robot joint state uses current radian/metre values, converted
    to degrees only for USD angular attributes. Runtime tensor state uses SI.

    The caller owns cache validity and output lifetime. Unsupported geometry or
    an invalid hull partition fails explicitly; nothing is silently omitted.
    """
    if convex_policy not in ('reject', 'split-exact'):
        raise ValueError("convex_policy must be 'reject' or 'split-exact'")
    if not hasattr(world, '_export_items'):
        raise TypeError('World must capture _export_items while loading its field')
    sources = {Path(world.config.robot_urdf).resolve()}
    sources.update((Path(directory) / item['urdf']).resolve()
                   for _, item, directory in world._export_items if 'urdf' in item)
    for path in sorted(sources):
        validate_joint_features(path)
    exporter = Exporter(world, Path(output).expanduser().resolve(), convex_policy,
                        convex_cook_scale, backend=backend, iterations=iterations,
                        articulation=articulation)
    exporter.source(world.config.game_pack)
    if world.config.robot_config:
        exporter.source(world.config.robot_config)
    for body, item, directory in world._export_items:
        if 'urdf' in item:
            exporter.urdf(body, (Path(directory) / item['urdf']).resolve(),
                          float(item.get('scale', 1)), item.get('name', ''))
        else:
            exporter.primitive_body(item, body)
    exporter.urdf(world.robot, Path(world.config.robot_urdf).resolve(), label='robot', robot=True)
    return exporter.finish()
