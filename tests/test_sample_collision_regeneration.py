"""The existing asset-regeneration entry point must retain corrected contacts."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np

from collider_metadata_assertions import assert_goal_profile_equivalent

REPO=Path(__file__).resolve().parents[1]


def test_sample_generator_reproduces_corrected_socket_and_pin_collision_paths(tmp_path):
    checkout=tmp_path/'checkout with spaces'
    objects=checkout/'assets/objects';objects.mkdir(parents=True)
    visuals=checkout/'assets/meshes/override';visuals.mkdir(parents=True)
    tools=checkout/'tools';tools.mkdir()
    shutil.copyfile(REPO/'tools/build_goal_pin_colliders.py',tools/'build_goal_pin_colliders.py')
    for name in ('goal_socket_collision.obj','goal_profile.json'):
        shutil.copyfile(REPO/'assets/objects'/name,objects/name)
    shutil.copyfile(REPO/'assets/meshes/override/goal_socket.obj',visuals/'goal_socket.obj')
    immutable_before={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in visuals.iterdir()}
    spec=importlib.util.spec_from_file_location('sample_generator',REPO/'assets/generate_samples.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.ROOT=checkout/'assets'
    # Exercise the actual legacy entry point, not a mocked delegate. It creates
    # source root profiles, all five color variants, and all goal wrappers.
    module.make_pin_and_goals()
    expected_metadata=json.loads((REPO/'assets/objects/pin_root_bands.json').read_text())
    # These OBJ profiles are regenerated from scratch here. Their byte hashes
    # must match provenance on Windows and Linux, not just their parsed points.
    for metadata in expected_metadata['meshes'].values():
        assert hashlib.sha256((objects/metadata['source']).read_bytes()).hexdigest()==metadata['source_sha256']
    expected_files=['goal_socket_collision.obj','goal_profile.json','pin_root_bands.json']+list(expected_metadata['meshes'])
    for name in expected_files:
        actual=(objects/name).read_bytes()
        expected=(REPO/'assets/objects'/name).read_bytes()
        if name=='goal_profile.json':
            assert_goal_profile_equivalent(actual,expected)
        else:
            assert actual==expected,name
    for path in objects.iterdir():
        if path.suffix in ('.json','.urdf'):
            assert b'\r' not in path.read_bytes(),path.name
    for name in expected_metadata['urdfs']:
        expected=ET.parse(REPO/'assets/objects'/name).getroot().find('link')
        actual=ET.parse(objects/name).getroot().find('link')
        assert [ET.tostring(c)for c in actual.findall('collision')]==[ET.tostring(c)for c in expected.findall('collision')]
        assert len(actual.findall('collision'))==13
        assert float(actual.find('./inertial/mass').get('value'))==.075
        assert np.allclose([float(v)for v in actual.find('./inertial/inertia').attrib.values()],
                           [float(v)for v in expected.find('./inertial/inertia').attrib.values()],rtol=0,atol=1e-18)
        assert [v.find('./geometry/mesh').get('filename')for v in actual.findall('visual')]==[v.find('./geometry/mesh').get('filename')for v in expected.findall('visual')]
    first={p.name:p.read_bytes()for p in objects.iterdir()}
    module.make_pin_and_goals()
    assert first=={p.name:p.read_bytes()for p in objects.iterdir()}
    assert immutable_before=={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in visuals.iterdir()}
