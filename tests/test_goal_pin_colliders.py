"""Offline CAD/collision regressions. No physics, GUI, or GPU initialization."""
from __future__ import annotations
from collections import Counter
import importlib.util
import json
import os
from pathlib import Path
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial import ConvexHull

from collider_metadata_assertions import assert_goal_profile_equivalent

REPO=Path(__file__).resolve().parents[1]
# Only used while reviewing a partial staged tree; normal repository tests need
# no environment override and read all assets from their own checkout.
SOURCE_ASSETS=Path(os.environ.get('EERSIM_COLLIDER_SOURCE_ASSETS',REPO/'assets'))
OUTPUT=REPO/'assets/objects'
spec=importlib.util.spec_from_file_location('goal_pin_generator',REPO/'tools/build_goal_pin_colliders.py')
generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(generator)


def asset(relative):
    candidate=REPO/'assets'/relative
    return candidate if candidate.exists() else SOURCE_ASSETS/relative


def stl_points(path):
    data=path.read_bytes();count=struct.unpack_from('<I',data,80)[0]
    assert len(data)==84+50*count
    records=np.frombuffer(data,offset=84,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
    return np.unique(records['v'].reshape(-1,3).astype(np.float64),axis=0)


def rotation(roll,yaw):
    cr,sr,cy,sy=np.cos(roll),np.sin(roll),np.cos(yaw),np.sin(yaw)
    return np.array([[cy,-sy*cr,sy*sr],[sy,cy*cr,-cy*sr],[0,sr,cr]])


def clips_surface(points,triangles):
    """Strict triangle/convex halfspace intersection, including edge-only cuts."""
    hull=ConvexHull(points);planes=np.unique(np.round(hull.equations,12),axis=0)
    selected=np.all(triangles.max(axis=1)>=points.min(axis=0),axis=1)&np.all(triangles.min(axis=1)<=points.max(axis=0),axis=1)
    for triangle in triangles[selected]:
        if np.any(np.min(triangle@planes[:,:3].T+planes[:,3],axis=0)>1e-12):continue
        polygon=list(triangle)
        for plane in planes:
            if not polygon:break
            output=[];a=polygon[-1];da=a@plane[:3]+plane[3]
            for b in polygon:
                db=b@plane[:3]+plane[3]
                if (da<=1e-12)!=(db<=1e-12):output.append(a+(b-a)*da/(da-db))
                if db<=1e-12:output.append(b)
                a,da=b,db
            polygon=output
        if len(polygon)<3:continue
        polygon=np.asarray(polygon);center=polygon.mean(axis=0)
        area=sum(np.linalg.norm(np.cross(polygon[i]-polygon[0],polygon[i+1]-polygon[0])) for i in range(1,len(polygon)-1))/2
        if area>1e-15 and np.max(center@planes[:,:3].T+planes[:,3]) < -1e-9:return True
    return False


def test_goal_shell_is_closed_consistently_oriented_and_retains_outer_wall():
    vertices,faces=generator.read_obj(OUTPUT/'goal_socket_collision.obj')
    summary=generator.closed_mesh_summary(vertices,faces)
    assert summary['triangles']==5312
    profile=json.loads((OUTPUT/'goal_profile.json').read_text())
    outer_count=32*len(profile['outer_sections'])
    import hashlib
    assert hashlib.sha256(vertices[:outer_count].astype('<f8').tobytes()).hexdigest()==profile['original_outer_vertices_sha256']
    shoulder=profile['horizontal_shoulder_z_m']
    rings=[s for s in profile['inner_sections']if s['z']==shoulder]
    assert len(rings)==2
    assert np.min(np.array(rings[1]['inner'])-rings[0]['inner'])>.003
    triangles=vertices[faces]
    ledge=(np.ptp(triangles[:,:,2],axis=1)==0)&(triangles[:,0,2]==shoulder)
    assert ledge.sum()==128


@pytest.mark.parametrize('roll',[0.,np.pi])
def test_original_goal_poses_have_no_pin_mesh_intersection_but_whole_root_does(roll):
    # CAD-authored neutral socket offset. Flipped colors share the same shape.
    vertices,faces=generator.read_obj(OUTPUT/'goal_socket_collision.obj')
    transform=rotation(roll,3*np.pi/4);triangles=vertices[faces]
    tree=ET.parse(OUTPUT/'pin_red_blue.urdf').getroot()
    collisions=tree.findall('./link/collision')
    assert len(collisions)==13
    for collision in collisions:
        name=collision.find('./geometry/mesh').get('filename')
        points=stl_points(asset('objects/'+name))@transform.T+[0,0,.0845]
        assert not clips_surface(points,triangles),name
    # Negative control detects the original convex bridge despite corrected goal.
    old_name='pin_root_-1.stl' if roll==0 else 'pin_root_1.stl'
    old=stl_points(asset('objects/'+old_name))@transform.T+[0,0,.0845]
    assert clips_surface(old,triangles)


def test_all_five_official_preplacements_keep_the_same_seating_frame():
    game=json.loads(asset('games/override-2026-27.json').read_text())
    pins=[o for o in game['objects']if o['name'].startswith('pin_goal_')]
    assert len(pins)==5
    for pin in pins:
        goal=min((o for o in game['objects']if o['name'].startswith('goal_')),
                 key=lambda o:np.linalg.norm(np.array(o['position'][:2])-pin['position'][:2]))
        path=(asset('games/override-2026-27.json').parent/goal['urdf']).resolve()
        socket=next(c for c in ET.parse(path).getroot().findall('./link/collision')if c.get('concave')=='yes')
        offset=np.fromstring(socket.find('origin').get('xyz'),sep=' ')
        assert np.allclose(np.array(pin['position'])-goal['position']-offset,[0,0,.0845],atol=1e-12,rtol=0)
        assert pin['rotation'][1]==0
        assert abs(abs(pin['rotation'][0])-np.pi)<1e-12 or pin['rotation'][0]==0
        assert pin['rotation'][2]==pytest.approx(3*np.pi/4)


def test_root_bands_retain_source_profiles_and_all_pin_inertial_visual_data():
    metadata=json.loads((OUTPUT/'pin_root_bands.json').read_text())
    assert len(metadata['meshes'])==8
    for name,item in metadata['meshes'].items():
        source,_=generator.read_obj(asset('objects/'+item['source']))
        lo,hi=item['z_min_max_m'];expected=source[(source[:,2]==lo)|(source[:,2]==hi)]
        expected=np.unique(expected.astype('<f4').astype(np.float64),axis=0)
        actual=stl_points(OUTPUT/name)
        assert all(np.any(np.all(expected==point,axis=1))for point in actual)
        assert ConvexHull(actual).volume==pytest.approx(ConvexHull(expected).volume,rel=1e-12)
    for name in metadata['urdfs']:
        link=ET.parse(OUTPUT/name).getroot().find('link')
        assert link.find('./inertial/mass').get('value')=='0.075'
        assert link.find('./inertial/origin').get('xyz')=='0 0 0'
        inertia=link.find('./inertial/inertia').attrib
        assert inertia=={'ixx':'0.000183825','iyy':'0.000183825','izz':'0.000183825','ixy':'0','ixz':'0','iyz':'0'}
        assert [v.find('./geometry/mesh').get('filename')for v in link.findall('visual')]==['../meshes/override/pin_upper.obj','../meshes/override/pin_lower.obj']
        names=[c.find('./geometry/mesh').get('filename')for c in link.findall('collision')]
        assert len(names)==13 and all((asset('objects/'+n)).exists()for n in names)
        assert 'pin_root_-1.stl' not in names and 'pin_root_1.stl' not in names


def test_lip_tracks_cad_without_replacing_horizontal_shoulder_by_a_ramp():
    cv,cf=generator.read_obj(OUTPUT/'goal_socket_collision.obj')
    sv,sf=generator.read_obj(asset('meshes/override/goal_socket.obj'))
    angles=(np.arange(192)+.371)*2*np.pi/192
    for height in (.075000001,.075847001,.076250001,.076499,.076501,.076691001,.077150001):
        cad=generator.section_radii(sv[sf],height,angles)
        collider=generator.section_radii(cv[cf],height,angles)
        assert np.max(cad-collider)<.00015
        assert np.max(collider-cad)<.00003
    assert np.min(generator.section_radii(cv[cf],.0765001,angles)-generator.section_radii(cv[cf],.0764999,angles))>.003


@pytest.mark.parametrize('input_text_newline', ['\n', '\r\n'])
def test_regeneration_is_deterministic_and_does_not_modify_inputs(tmp_path, input_text_newline):
    source=tmp_path/'source';objects=source/'assets/objects';objects.mkdir(parents=True)
    visual=source/'assets/meshes/override';visual.mkdir(parents=True)
    shutil.copyfile(asset('meshes/override/goal_socket.obj'),visual/'goal_socket.obj')
    names=['goal_socket_collision.obj','goal_profile.json','pin_root_-1.obj','pin_root_1.obj']
    names+=json.loads((OUTPUT/'pin_root_bands.json').read_text())['urdfs']
    for name in names:
        original=asset('objects/'+name)
        if original.suffix in ('.json','.urdf'):
            # Git normalizes these text formats to LF, while binary-tagged OBJ
            # retains CRLF. Also cover regeneration of a prior Windows output.
            (objects/name).write_bytes(original.read_text(encoding='utf-8').replace('\n',input_text_newline).encode('utf-8'))
        else:
            shutil.copyfile(original,objects/name)
    before={p.relative_to(source):p.read_bytes()for p in source.rglob('*')if p.is_file()}
    out=tmp_path/'generated'
    generator.build_goal(source,out);generator.build_root_bands(source,out)
    assert before=={p.relative_to(source):p.read_bytes()for p in source.rglob('*')if p.is_file()}
    for path in (out/'objects').iterdir():
        expected=(OUTPUT/path.name).read_bytes()
        if path.name=='goal_profile.json':
            assert_goal_profile_equivalent(path.read_bytes(),expected)
        else:
            assert path.read_bytes()==expected,path.name
        if path.suffix in ('.json','.urdf'):
            assert b'\r' not in path.read_bytes(),path.name


@pytest.mark.parametrize('newline', ['\n', '\r\n'])
def test_obj_writer_preserves_binary_tagged_line_endings(tmp_path, newline):
    source=tmp_path/'source.obj'
    source.write_bytes((newline.join(['# original','v 0 0 0'])+newline).encode())
    output=tmp_path/'generated.obj'
    generator.write_obj(output,np.array([[1.,2.,3.]]),np.empty((0,3),dtype=int),
                        'fixture',newline=generator.obj_newline(source))
    assert output.read_bytes()==('# fixture; metres'+newline+'v 1.000000000000 2.000000000000 3.000000000000'+newline).encode()
