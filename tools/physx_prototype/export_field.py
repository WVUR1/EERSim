"""Offline physics-only export of a loaded EERSim world to ovphysx 0.5.11 USDA.

No stepping, GUI, remote execution, motor control, or published-source edits.
Requires EERSim's Python dependencies and scipy==1.16.3 for exact hull partitions.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
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


class Exporter:
    def __init__(self, world, output, policy, convex_cook_scale=1.):
        import pybullet as p
        self.p, self.world, self.output, self.policy = p, world, output, policy
        self.convex_cook_scale = float(convex_cook_scale)
        if not math.isfinite(self.convex_cook_scale) or self.convex_cook_scale < 1:
            raise ValueError('convex_cook_scale must be finite and at least1')
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / "geometry").mkdir(exist_ok=True)
        self.lines = ['#usda 1.0', '(defaultPrim = "World"', ' metersPerUnit = 1',
                      ' kilogramsPerUnit = 1', ' upAxis = "Z")', 'def Xform "World"', '{']
        self.manifest = {"schema_version": 1, "benchmark": "passive full-field contact throughput",
            "ovphysx_version": "0.5.11", "physics_hz": world.config.physics_hz,
            "convex_cook_scale": self.convex_cook_scale,
            "gravity_m_s2": world.config.gravity, "solver": {"PhysX": "TGS", "position_iterations": 100,
                "velocity_iterations": 1, "Bullet": "sequential impulse, maximum100iterations"},
            "sources": {}, "meshes": {}, "actors": [], "joints": [], "counts": {},
            "differences": [
                "No PROS program, voltage/torque motor model or external omni-tire forces; all joints passive.",
                "Regular PhysX joints replace Bullet reduced-coordinate articulations for this bounded import.",
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
            ' bool physxScene:enableGPUDynamics = true',
            ' uniform token physxScene:broadphaseType = "GPU"',
            ' uniform token physxScene:solverType = "TGS"',
            ' uniform uint physxScene:minPositionIterationCount = 100',
            ' uniform uint physxScene:maxPositionIterationCount = 100',
            ' uniform uint physxScene:minVelocityIterationCount = 1',
            ' uniform uint physxScene:maxVelocityIterationCount = 1', '}']

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
        d = p.getDynamicsInfo(body, index, physicsClientId=c)
        name = f'{"rigid" if d[0] > 0 else "static"}_{body}_{"base" if index < 0 else index}'
        material = f'material_{body}_{"base" if index < 0 else index}'
        self.material(material, d)
        if index < 0:
            com_pose = p.getBasePositionAndOrientation(body, physicsClientId=c)
            inertial = self.world.base_inertial_frames.get(body, (d[3], d[4]))
            position, orientation = p.multiplyTransforms(*com_pose, *p.invertTransform(*inertial))
        else:
            state = p.getLinkState(body, index, physicsClientId=c)
            position, orientation = state[4:6]
        schemas = ['PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'PhysxRigidBodyAPI'] if d[0] > 0 else []
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
                ' int physxRigidBody:solverPositionIterationCount = 100',
                ' int physxRigidBody:solverVelocityIterationCount = 1',
                f' float physxRigidBody:linearDamping = {number(.02 if robot else .04)}',
                f' float physxRigidBody:angularDamping = {number(.02 if robot else .04)}']
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
        paths = {name: '/World/' + f'{"rigid" if p.getDynamicsInfo(body, index, physicsClientId=c)[0] > 0 else "static"}_{body}_{"base" if index < 0 else index}'
                 for name, index in indices.items()}
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
            self.lines += [f'def {kind} "joint_{body}_{ji}" {{',
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
            self.lines.append('}')
            self.counts['joints'] += 1
            self.manifest['joints'].append({'name': joint.get('name'), 'type': jt, 'parent': paths[parent], 'child': paths[child]})

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
        self.manifest['counts'] = dict(self.counts)
        self.manifest['scene_sha256'] = sha(scene)
        self.manifest['convex_policy'] = self.policy
        self.manifest['exporter_sha256'] = sha(__file__)
        (self.output / 'field.manifest.json').write_text(json.dumps(self.manifest, indent=2) + '\n', encoding='utf-8')
        print(json.dumps({'scene': str(scene), 'sha256': sha(scene), 'counts': dict(self.counts)}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--convex-policy', choices=['reject', 'split-exact'], default='reject')
    parser.add_argument('--convex-cook-scale', type=float, default=1.,
                        help='Explicit raw-convex cooking preconditioner; compensating collider scale preserves world geometry')
    args = parser.parse_args()
    sys.path.insert(0, str(args.repository.resolve()))
    from eersim.demos import claw_demo_config
    from eersim.physics import World
    # Capture actual generated floor/rail primitives as World imports them.
    class CapturedWorld(World):
        def _add_object(self, item, directory):
            body = super()._add_object(item, directory)
            self.export_items.append((body, item.copy(), Path(directory)))
            return body
    config = claw_demo_config()
    world = CapturedWorld.__new__(CapturedWorld)
    world.export_items = []
    CapturedWorld.__init__(world, config)
    try:
        exporter = Exporter(world, args.output.resolve(), args.convex_policy, args.convex_cook_scale)
        exporter.source(config['game_pack'])
        exporter.source(config['robot_config'])
        for body, item, directory in world.export_items:
            if 'urdf' in item:
                exporter.urdf(body, (directory / item['urdf']).resolve(), float(item.get('scale', 1)), item.get('name', ''))
            else:
                exporter.primitive_body(item, body)
        exporter.urdf(world.robot, Path(config['robot_urdf']), label='robot', robot=True)
        exporter.finish()
    finally:
        world.close()


if __name__ == '__main__':
    main()
