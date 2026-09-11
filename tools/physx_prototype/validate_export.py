"""Read-only USD composition and physical-payload checks; no simulation."""
if not __debug__:
    raise RuntimeError("Run validation without Python optimization; -O and -OO disable required assertion checks")

import argparse
import json
from pathlib import Path

from pxr import Usd, UsdGeom, UsdPhysics
from pxr import Gf
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--equivalent-to', type=Path,
                        help='Assert the same world-space geometry and physical settings as this prior export')
    parser.add_argument('--allow-filter-authorship-changes', action='store_true',
                        help='Permit equivalent filtered pairs to move onto their resolvable dynamic actor')
    args = parser.parse_args()
    manifest = json.loads((args.directory / 'field.manifest.json').read_text())
    stage = Usd.Stage.Open(str(args.directory / 'field.usda'))
    assert stage
    prims = list(stage.Traverse())
    rigid = [x for x in prims if x.HasAPI(UsdPhysics.RigidBodyAPI)]
    collisions = [x for x in prims if x.HasAPI(UsdPhysics.CollisionAPI)]
    assert len(rigid) == manifest['counts']['dynamic_actors']
    assert len(collisions) == manifest['counts']['collision_shapes']
    meshes = 0
    for prim in collisions:
        if prim.IsA(UsdGeom.Mesh):
            meshes += 1
            assert prim.HasAPI(UsdPhysics.MeshCollisionAPI), str(prim.GetPath())
            approx = prim.GetAttribute('physics:approximation').Get()
            assert approx in ('none', 'convexHull'), (str(prim.GetPath()), approx)
            if approx == 'convexHull':
                assert prim.GetAttribute('physxConvexHullCollision:hullVertexLimit').Get() == 64
                assert prim.GetAttribute('physxConvexHullCollision:minThickness').Get() == 0
        targets = prim.GetRelationship('material:binding:physics').GetTargets()
        assert len(targets) == 1
        material = stage.GetPrimAtPath(targets[0])
        for setting in ('frictionCombineMode', 'restitutionCombineMode'):
            assert material.GetAttribute('physxMaterial:' + setting).Get() == 'multiply'
    for record in manifest['actors']:
        if record['mass_kg'] <= 0:
            continue
        prim = stage.GetPrimAtPath(record['path'])
        mass = prim.GetAttribute('physics:mass').Get()
        assert abs(mass - record['mass_kg']) < max(1e-7, mass * 1e-7)
        assert prim.GetAttribute('physxRigidBody:solverPositionIterationCount').Get() == 100
        assert prim.GetAttribute('physxRigidBody:solverVelocityIterationCount').Get() == 1
    filters = 0
    for group in manifest.get('collision_filters', []):
        for i, path in enumerate(group['actors']):
            expected = group.get('authored_targets', {}).get(path, group['actors'][i + 1:])
            if expected:
                prim = stage.GetPrimAtPath(path)
                assert prim.HasAPI(UsdPhysics.FilteredPairsAPI)
                actual = list(map(str, prim.GetRelationship('physics:filteredPairs').GetTargets()))
                assert actual == expected, (path, actual, expected)
                filters += len(actual)
                assert prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.CollisionAPI), \
                    'Filter author must resolve to a native body or collider: ' + path
        expected_pairs = {tuple(sorted((a, b))) for i, a in enumerate(group['actors']) for b in group['actors'][i + 1:]}
        authored_pairs = {tuple(sorted((a, b))) for a, targets in group.get('authored_targets',
            {a: group['actors'][i + 1:] for i, a in enumerate(group['actors'])}).items() for b in targets}
        assert authored_pairs == expected_pairs
    cache = UsdGeom.XformCache()
    joint_errors = []
    for joint in [x for x in prims if x.IsA(UsdPhysics.Joint)]:
        frames = []
        for i in (0, 1):
            path = joint.GetRelationship('physics:body' + str(i)).GetTargets()[0]
            matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))
            position = Gf.Vec3d(joint.GetAttribute('physics:localPos' + str(i)).Get())
            rotation = Gf.Rotation(Gf.Quatd(joint.GetAttribute('physics:localRot' + str(i)).Get()))
            frames.append((matrix.Transform(position), [matrix.TransformDir(rotation.TransformDir(Gf.Vec3d(axis)))
                for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]))
        delta = frames[1][0] - frames[0][0]
        axis = frames[0][1][0]
        if joint.IsA(UsdPhysics.PrismaticJoint):
            displacement = Gf.Dot(delta, axis)
            position_error = (delta - axis * displacement).GetLength()
            lower, upper = [joint.GetAttribute('physics:' + x + 'Limit').Get() for x in ('lower', 'upper')]
            assert lower - 1e-6 <= displacement <= upper + 1e-6
        else:
            position_error = delta.GetLength()
        axis_error = (frames[0][1][0] - frames[1][1][0]).GetLength()
        assert position_error < 1e-6 and axis_error < 1e-6, (str(joint.GetPath()), position_error, axis_error)
        if not joint.IsA(UsdPhysics.RevoluteJoint):
            assert max((a-b).GetLength() for a,b in zip(frames[0][1], frames[1][1])) < 1e-6
        joint_errors.append(position_error)
    result = {'passed': True, 'usd_core_version': Usd.GetVersion(), 'prims': len(prims),
              'dynamic_actors': len(rigid), 'colliders': len(collisions), 'mesh_colliders': meshes,
              'filtered_body_pairs': filters, 'joints': len(joint_errors),
              'max_joint_forbidden_translation_m': max(joint_errors, default=0.)}
    if args.equivalent_to:
        previous_manifest = json.loads((args.equivalent_to / 'field.manifest.json').read_text())
        reoriented_pairs = []
        for key in ('actors', 'joints', 'counts', 'collision_filters', 'solver', 'gravity_m_s2', 'physics_hz'):
            if key == 'collision_filters' and args.allow_filter_authorship_changes:
                for new, old in zip(manifest[key], previous_manifest[key]):
                    assert new['body_id'] == old['body_id'] and new['actors'] == old['actors'] and new['policy'] == old['policy']
                    old_targets = old.get('authored_targets', {a: old['actors'][i + 1:] for i, a in enumerate(old['actors'])})
                    old_pairs = {(a, b) for a, targets in old_targets.items() for b in targets}
                    new_pairs = {(a, b) for a, targets in new['authored_targets'].items() for b in targets}
                    for first, second in old_pairs - new_pairs:
                        assert first.startswith('/World/static_') and second.startswith('/World/rigid_')
                        assert (second, first) in new_pairs - old_pairs
                        reoriented_pairs.append([first, second])
                    assert len(old_pairs-new_pairs) == len(new_pairs-old_pairs)
                assert len(manifest[key]) == len(previous_manifest[key])
                continue
            assert manifest[key] == previous_manifest[key], 'Changed physical metadata: ' + key
        previous = Usd.Stage.Open(str(args.equivalent_to / 'field.usda'))
        previous_cache = UsdGeom.XformCache()
        maximum_error = 0.
        mesh_count = 0
        for prim in prims:
            old = previous.GetPrimAtPath(prim.GetPath())
            assert old and old.GetTypeName() == prim.GetTypeName(), str(prim.GetPath())
            # Physics APIs include scalar coefficients, body inertia, contact
            # offsets, limits and filters. Coordinate preconditioning changes none.
            for attr in prim.GetAttributes():
                if str(attr.GetName()).startswith(('physics:', 'physx')):
                    assert attr.Get() == old.GetAttribute(attr.GetName()).Get(), (str(prim.GetPath()), str(attr.GetName()))
            if prim.IsA(UsdGeom.Mesh):
                points = np.asarray(prim.GetAttribute('points').Get(), dtype=float)
                old_points = np.asarray(old.GetAttribute('points').Get(), dtype=float)
                assert points.shape == old_points.shape
                new_matrix = np.asarray(cache.GetLocalToWorldTransform(prim))
                old_matrix = np.asarray(previous_cache.GetLocalToWorldTransform(old))
                new_world = np.column_stack((points, np.ones(len(points)))) @ new_matrix
                old_world = np.column_stack((old_points, np.ones(len(points)))) @ old_matrix
                maximum_error = max(maximum_error, float(np.max(np.abs(new_world-old_world))))
                for name in ('faceVertexCounts', 'faceVertexIndices'):
                    assert prim.GetAttribute(name).Get() == old.GetAttribute(name).Get()
                mesh_count += 1
            else:
                for attr in prim.GetAttributes():
                    assert attr.Get() == old.GetAttribute(attr.GetName()).Get(), (str(prim.GetPath()), str(attr.GetName()))
        assert maximum_error <= 1e-7, maximum_error
        result['world_equivalence'] = {'baseline': str(args.equivalent_to),
            'mesh_instances_checked': mesh_count, 'maximum_coordinate_error_m': maximum_error,
            'actors_joints_materials_contact_offsets_unchanged': True,
            'filter_authorship_change_allowed': args.allow_filter_authorship_changes,
            'reoriented_filter_pairs': reoriented_pairs}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
