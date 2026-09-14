"""Rebuild the goal contact lip from the official CAD mesh; offline geometry only.

This does not change the visual mesh, starts, mass, inertia or physics settings.
The outer goal surface is copied exactly from the existing collider. The inner
surface preserves the CAD's horizontal shoulder and refines its curved lip.
Run with --repository SOURCE --output-assets NEW_ASSET_DIRECTORY. Inputs are
never modified. NumPy and SciPy are offline generation dependencies only.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import re
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull


def read_obj(path):
    vertices, faces = [], []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if fields and fields[0] == "v":
            vertices.append([float(x) for x in fields[1:4]])
        elif fields and fields[0] == "f":
            indices = [int(x.split("/")[0])-1 for x in fields[1:]]
            if any(i < 0 for i in indices):
                raise ValueError("These source meshes must use positive OBJ indices")
            faces.extend((indices[0], indices[j], indices[j+1]) for j in range(1,len(indices)-1))
    return np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64)


def obj_newline(path):
    """Preserve binary-tagged OBJ provenance bytes across operating systems."""
    data = Path(path).read_bytes()
    return "\r\n" if b"\r\n" in data else "\n"


def write_obj(path, vertices, faces, description, *, newline="\r\n"):
    text = ["# " + description + "; metres"]
    text.extend("v " + " ".join(f"{v:.12f}" for v in row) for row in vertices)
    text.extend("f " + " ".join(str(int(i)+1) for i in row) for row in faces)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_bytes((newline.join(text)+newline).encode("utf-8"))


def section_radii(triangles, z, angles):
    """Minimum positive ray/mesh-section intersections (the central opening)."""
    selected = triangles[(triangles[:,:,2].min(axis=1) < z) &
                         (triangles[:,:,2].max(axis=1) > z)]
    a = selected
    b = np.roll(selected, -1, axis=1)
    crosses = (a[:,:,2]-z)*(b[:,:,2]-z) < 0
    selected = crosses.sum(axis=1) == 2
    a, b, crosses = a[selected], b[selected], crosses[selected]
    aa, bb = a[crosses], b[crosses]
    points = aa+(bb-aa)*((z-aa[:,2])/(bb[:,2]-aa[:,2]))[:,None]
    segments = points[:,:2].reshape(-1,2,2)
    if not len(segments):
        raise ValueError(f"CAD has no section at {z}")
    origin = segments[:,0]
    edge = segments[:,1]-origin
    directions = np.column_stack((np.cos(angles),np.sin(angles)))
    den = directions[:,0,None]*edge[None,:,1]-directions[:,1,None]*edge[None,:,0]
    valid = abs(den) > 1e-14
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = (origin[:,0]*edge[:,1]-origin[:,1]*edge[:,0])[None,:]/den
        u = (origin[None,:,0]*directions[:,1,None]-origin[None,:,1]*directions[:,0,None])/den
    hit = valid & (distance>0) & (u>=-1e-10) & (u<=1+1e-10)
    values = np.min(np.where(hit,distance,np.inf),axis=1)
    if not np.isfinite(values).all():
        raise ValueError("A radial ray missed the central CAD section")
    return values


def closed_mesh_summary(vertices, faces):
    edges = Counter(tuple(sorted((int(a),int(b)))) for f in faces
                    for a,b in zip(f,np.roll(f,-1)))
    triangles = vertices[faces]
    volume = float(np.einsum("ij,ij->i",triangles[:,0],
                            np.cross(triangles[:,1],triangles[:,2])).sum()/6)
    if not all(v == 2 for v in edges.values()) or volume <= 0:
        raise ValueError("Generated contact shell is not closed with positive signed volume")
    directed = Counter((int(a),int(b)) for f in faces for a,b in zip(f,np.roll(f,-1)))
    if any(directed.get((b,a),0) != count for (a,b),count in directed.items()):
        raise ValueError("Generated faces have inconsistent winding")
    return {"vertices":len(vertices),"triangles":len(faces),
            "closed_two_faces_per_edge":True,"signed_volume_m3":volume}


def build_goal(repository, output_assets, tolerance=0.000025):
    assets = repository/"assets"
    source = assets/"meshes/override/goal_socket.obj"
    source_v, source_f = read_obj(source)
    source_t = source_v[source_f]
    old_v, old_f = read_obj(assets/"objects/goal_socket_collision.obj")
    old_profile = json.loads((assets/"objects/goal_profile.json").read_text())
    source_sha=hashlib.sha256(source.read_bytes()).hexdigest()
    if old_profile.get('source_sha256') != source_sha:
        raise ValueError('CAD source hash differs from the recorded goal profile provenance')
    original_sections = old_profile.get('outer_sections',old_profile.get('sections'))
    if not original_sections:
        raise ValueError("The source profile must record the original outer sections")
    original_heights = [float(s["z"]) for s in original_sections]
    if 'inner_sections' in old_profile:
        # Regeneration of the corrected format: original outer vertices are
        # retained first, before all independently sampled inner vertices.
        outer_indices=np.arange(len(original_heights)*32)
    else:
        if old_v.shape != (len(original_heights)*64,3):
            raise ValueError("Expected the authored 32-inner/32-outer ring layout")
        outer_indices = np.array([64*j+i for j in range(len(original_heights)) for i in range(32,64)])
    remap = {int(old):i for i,old in enumerate(outer_indices)}
    outer_faces = [tuple(remap[int(i)] for i in face) for face in old_f
                   if all(int(i) in remap for i in face)]
    outer_v = old_v[outer_indices]
    angles = np.arange(64)*2*np.pi/64
    cache = {}
    epsilon = 1e-10
    def radii(z, side=0):
        sample_z = z + (side or 1)*epsilon
        if z == original_heights[0]: sample_z = z+epsilon
        if z == original_heights[-1]: sample_z = z-epsilon
        key = (z,side)
        if key not in cache:
            # Avoid exact triangle-ring planes, except explicit one-sided limits.
            cache[key] = section_radii(source_t,sample_z,angles)
        return cache[key]
    # The source CAD has a horizontal annular shoulder at 76.5mm. Discover its
    # actual plane from central, horizontal faces; never replace it by a ramp.
    is_horizontal = np.ptp(source_t[:,:,2],axis=1) < 1e-11
    centroid = source_t.mean(axis=1)
    annulus = is_horizontal & (centroid[:,2]>.076) & (centroid[:,2]<.077) & (np.linalg.norm(centroid[:,:2],axis=1)<.041)
    shoulder_planes = np.unique(np.round(centroid[annulus,2],10))
    if len(shoulder_planes)!=1:
        raise ValueError("Expected exactly one central horizontal goal shoulder")
    shoulder = float(shoulder_planes[0])
    # Only change the lip's axial fidelity. Retain all original axial anchors.
    anchors = sorted(set(original_heights+[.074,shoulder]))
    rings = []
    max_checked_error = 0.
    def interval(lo, rlo, hi, rhi, depth=0):
        nonlocal max_checked_error
        if hi-lo < 1e-9 or depth>20:
            raise ValueError("CAD section refinement did not converge")
        probes = [(q,lo+(hi-lo)*q) for q in (.25,.5,.75)]
        error = max(float(np.max(abs(radii(z)-(rlo*(1-q)+rhi*q)))) for q,z in probes)
        # The lower socket is unchanged in axial sampling; only the lip is refined.
        if lo >= .074 and error > tolerance:
            mid=(lo+hi)/2; rmid=radii(mid)
            interval(lo,rlo,mid,rmid,depth+1)
            interval(mid,rmid,hi,rhi,depth+1)
        else:
            if lo>=.074: max_checked_error=max(max_checked_error,error)
            rings.append((hi,rhi))
    rings.append((anchors[0],radii(anchors[0])))
    for lo,hi in zip(anchors,anchors[1:]):
        if lo == shoulder:
            # A second, larger inner ring at the same height is the flat ledge.
            rings.append((lo,radii(lo,+1)))
        rlo=radii(lo,+1 if lo==shoulder else 0)
        rhi=radii(hi,-1 if hi==shoulder else 0)
        interval(lo,rlo,hi,rhi)
    inner_v=np.asarray([[radius*np.cos(angle),radius*np.sin(angle),z]
                        for z,rs in rings for radius,angle in zip(rs,angles)])
    vertices=np.vstack((outer_v,inner_v)); offset=len(outer_v)
    faces=list(outer_faces)
    for j in range(len(rings)-1):
        for i in range(64):
            lo=offset+j*64+i; ln=offset+j*64+(i+1)%64
            hi=lo+64; hn=ln+64
            faces.extend(((lo,hi,hn),(lo,hn,ln)))
    for top in (False,True):
        outer_start=(len(original_heights)-1)*32 if top else 0
        inner_start=offset+(len(rings)-1)*64 if top else offset
        for i in range(32):
            a=outer_start+i; b=outer_start+(i+1)%32
            x=inner_start+2*i; y=inner_start+(2*i+1)%64; z=inner_start+(2*i+2)%64
            tris=((a,b,z),(a,z,y),(a,y,x))
            faces.extend(tris if top else [tuple(reversed(t)) for t in tris])
    faces=np.asarray(faces,dtype=np.int64)
    summary=closed_mesh_summary(vertices,faces)
    destination=output_assets/"objects/goal_socket_collision.obj"
    write_obj(destination,vertices,faces,"CAD triangle-section goal lip; original outer wall",
              newline=obj_newline(assets/"objects/goal_socket_collision.obj"))
    profile={"source_sha256":source_sha,
        "source":"../meshes/override/goal_socket.obj","units":"metres",
        "method":"64 radial CAD mesh section rays for the inner opening; exact horizontal shoulder preserved; adaptive quarter/midpoint lip checks. Original 32-ray outer wall is unchanged. This is a triangle-mesh approximation, not an exact BRep.",
        "inner_angular_samples":64,"outer_angular_samples":32,
        "lip_checked_axial_error_limit_m":tolerance,
        "lip_max_checked_axial_error_m":max_checked_error,
        "horizontal_shoulder_z_m":shoulder,
        "inner_sections":[{"z":z,"inner":rs.tolist()} for z,rs in rings],
        "outer_sections":[{"z":s["z"],"outer":s["outer"]} for s in original_sections],
        "original_outer_vertices_sha256":hashlib.sha256(outer_v.astype("<f8").tobytes()).hexdigest(),
        "mesh":summary}
    (destination.parent/"goal_profile.json").write_text(json.dumps(profile,indent=2)+"\n", encoding="utf-8", newline="\n")
    return profile


def write_convex_stl(path, points):
    points = np.unique(points,axis=0)
    hull=ConvexHull(points)
    faces=hull.simplices.copy()
    triangles=points[faces]
    reverse=np.einsum('ij,ij->i',np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),hull.equations[:,:3])<0
    faces[reverse]=faces[reverse,::-1]
    used=np.unique(faces)
    remap=np.full(len(points),-1,dtype=np.int64);remap[used]=np.arange(len(used))
    points=points[used];faces=remap[faces]
    summary=closed_mesh_summary(points,faces)
    path.parent.mkdir(parents=True,exist_ok=True)
    triangles=points[faces]
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    normals/=np.linalg.norm(normals,axis=1)[:,None]
    data=bytearray(b'EERSim CAD-profile convex axial band; metres'.ljust(80,b' '))
    data.extend(struct.pack('<I',len(faces)))
    for normal,triangle in zip(normals,triangles):
        data.extend(struct.pack('<12fH',*normal,*triangle.ravel(),0))
    path.write_bytes(data)
    return summary


def build_root_bands(repository,output_assets):
    """Partition the existing nonconvex root profile BEFORE convexification.

    Adjacent ring pairs retain source flats and axial curvature; a single hull
    across all five rings otherwise fills the root/neck fillet. These hulls
    may still be split by the PhysX importer to satisfy GPU cooking limits.
    """
    source=repository/'assets/objects';output=output_assets/'objects'
    replacements={};result={}
    for sign in (-1,1):
        points,_=read_obj(source/f'pin_root_{sign}.obj')
        heights=np.unique(points[:,2]);names=[]
        if len(heights)!=5:
            raise ValueError('Expected five authored pin-root rings')
        for index,(lo,hi) in enumerate(zip(heights,heights[1:])):
            band=points[(points[:,2]==lo)|(points[:,2]==hi)]
            name=f'pin_root_{sign}_band_{index}.stl'
            summary=write_convex_stl(output/name,band)
            result[name]={'source':f'pin_root_{sign}.obj','source_sha256':hashlib.sha256((source/f'pin_root_{sign}.obj').read_bytes()).hexdigest(),
                          'z_min_max_m':[float(lo),float(hi)],**summary}
            names.append(name)
        replacements[f'pin_root_{sign}.stl']=names
    modified=[]
    for urdf in sorted(source.glob('pin_*.urdf')):
        text=urdf.read_text(encoding='utf-8')
        # Preserve authored visual/inertial data and unrelated collision leaves;
        # use the repository's canonical LF text format on every platform.
        for old,names in replacements.items():
            filename_pattern=re.escape(old[:-4])+r'(?:_band_[0-9]+)?\.stl'
            pattern=r'    <collision>\s*<geometry>\s*<mesh filename="('+filename_pattern+r')" />\s*</geometry>\s*</collision>'
            matches=list(re.finditer(pattern,text))
            if len(matches) not in (1,len(names)):
                raise ValueError(f'{urdf.name}: expected an original root block or its complete band replacement')
            if len(matches)>1 and any(text[a.end():b.start()].strip() for a,b in zip(matches,matches[1:])):
                raise ValueError('Root collision band blocks must be contiguous')
            block=matches[0].group(0)
            existing_name=matches[0].group(1)
            text=text[:matches[0].start()]+'\n'.join(block.replace(existing_name,name) for name in names)+text[matches[-1].end():]
        (output/urdf.name).write_text(text,encoding='utf-8',newline='\n');modified.append(urdf.name)
    if not modified: raise ValueError('No pin URDFs found')
    (output/'pin_root_bands.json').write_text(json.dumps({'method':'Four adjacent authored axial-ring convex bands per root; preserves flats and avoids whole-profile convex bridging. Source points unchanged except binary STL float32 quantization.','meshes':result,'urdfs':modified},indent=2)+'\n',encoding='utf-8',newline='\n')
    return {'meshes':result,'urdfs':modified}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository",type=Path,required=True)
    parser.add_argument("--output-assets",type=Path,required=True)
    parser.add_argument("--root-bands",action="store_true",help="Also replace whole pin-root hulls by adjacent axial bands")
    args=parser.parse_args(); repository=args.repository.resolve(); output=args.output_assets.resolve()
    if output == (repository/"assets").resolve() or (repository/"assets").resolve() in output.parents:
        parser.error("Use a separate output directory; source assets are read-only inputs")
    result=build_goal(repository,output)
    bands=build_root_bands(repository,output) if args.root_bands else None
    print(json.dumps({"goal_mesh":result["mesh"],"inner_rings":len(result["inner_sections"]),
                     "checked_axial_error_m":result["lip_max_checked_axial_error_m"],
                     "root_band_meshes":len(bands['meshes']) if bands else 0}))

if __name__ == "__main__": main()
