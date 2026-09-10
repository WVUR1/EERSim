"""Convert official VEX STEP geometry into measured, attributed SI visual meshes.

Requires cadquery-ocp (OpenCASCADE). CAD sources and intermediate BREP files stay
in the explicitly supplied cache directory; only derived OBJ/MTL and provenance
JSON belong in simulator assets.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time
import zipfile

SOURCE_URL = "https://link.vex.com/docs/26-27/v5rc/field-cad"
ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def label_name(label):
    from OCP.TDataStd import TDataStd_Name
    attr = TDataStd_Name()
    if label.IsAttribute(TDataStd_Name.GetID_s()) and label.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def label_key(label):
    from OCP.TDF import TDF_Tool
    from OCP.TCollection import TCollection_AsciiString
    value = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, value)
    return value.ToCString()


def bbox(shape, precise=True):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    box = Bnd_Box()
    if precise:
        BRepBndLib.AddOptimal_s(shape, box, False, False)
    else:
        BRepBndLib.Add_s(shape, box, False)
    # Corner access works on OCP 7 and 8; OCP 8.0.1 does not register
    # Bnd_Box::Limits returned by Get() with the Python binding.
    low, high = box.CornerMin(), box.CornerMax()
    return [[low.X(), low.Y(), low.Z()], [high.X(), high.Y(), high.Z()]]


def matrix(location):
    trsf = location.Transformation()
    return [[trsf.Value(row, col) for col in range(1, 5)] for row in range(1, 4)] + [[0, 0, 0, 1]]


def load_step(step, cache=None):
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.Interface import Interface_Static
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDocStd import TDocStd_Application
    from OCP.BinXCAFDrivers import BinXCAFDrivers
    binary = cache / "assembly.xbf" if cache is not None else None
    checksum = digest(step)
    application = TDocStd_Application()
    BinXCAFDrivers.DefineFormat_s(application)
    if binary is not None and binary.is_file() and binary.with_suffix(".sha256").is_file() and binary.with_suffix(".sha256").read_text() == checksum:
        print("Reading cached binary CAD document...", flush=True)
        doc = application.Retrieve(TCollection_ExtendedString(str(binary.resolve().parent)),
                                   TCollection_ExtendedString(binary.name))
        return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()), XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)
    Interface_Static.SetCVal_s("xstep.cascade.unit", "MM")
    print(f"Reading official STEP: {step.name}", flush=True)
    if reader.ReadFile(str(step)) != IFSelect_RetDone:
        raise ValueError(f"OpenCASCADE could not read {step}")
    doc = TDocStd_Document(TCollection_ExtendedString("BinXCAF"))
    print("Transferring CAD assembly and named parts...", flush=True)
    if not reader.Transfer(doc):
        raise ValueError("OpenCASCADE could not transfer the STEP assembly")
    print("CAD assembly transfer complete.", flush=True)
    if binary is not None:
        try:
            status = application.SaveAs(doc, TCollection_ExtendedString(str(binary.resolve())))
            if int(status) == 0 and binary.is_file():
                binary.with_suffix(".sha256").write_text(checksum)
                print("Saved binary document cache.", flush=True)
            else:
                print(f"Binary document cache unavailable ({status}); continuing with live CAD.", flush=True)
        except Exception as exc:
            print(f"Binary document cache unavailable ({exc}); continuing with live CAD.", flush=True)
    return doc, XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()), XCAFDoc_DocumentTool.ColorTool_s(doc.Main())


def color_of(shape, color_tool):
    from OCP.Quantity import Quantity_ColorRGBA
    from OCP.XCAFDoc import XCAFDoc_ColorSurf, XCAFDoc_ColorGen
    color = Quantity_ColorRGBA()
    if color_tool.GetColor(shape, XCAFDoc_ColorSurf, color) or color_tool.GetColor(shape, XCAFDoc_ColorGen, color):
        rgb = color.GetRGB()
        return [rgb.Red(), rgb.Green(), rgb.Blue(), color.Alpha()]
    return None


def catalogue(step, cache):
    from OCP.collections import Sequence_TDF_Label
    from OCP.TDF import TDF_Label
    from OCP.TopLoc import TopLoc_Location
    from OCP.BRepTools import BRepTools
    doc, shapes, colors = load_step(step, cache)
    labels = Sequence_TDF_Label()
    shapes.GetShapes(labels)
    definitions, shape_map = [], {}
    cache.mkdir(parents=True, exist_ok=True)
    for index in range(1, labels.Length() + 1):
        label = labels.Value(index)
        if shapes.IsReference_s(label):
            continue
        shape = shapes.GetShape_s(label)
        if shape.IsNull():
            continue
        key, name = label_key(label), label_name(label)
        bounds = bbox(shape, precise=False)
        size = [high - low for low, high in zip(*bounds)]
        entry = {"id": key, "name": name, "assembly": shapes.IsAssembly_s(label),
                 "bounds_mm": bounds, "size_mm": size, "color_rgba": color_of(shape, colors)}
        # Cache all reusable game pieces and compact field subassemblies.
        # Never duplicate the complete 132 MB source field as a BREP.
        if name.startswith(("276-9250", "276-7596")) and max(size) < 1800:
            brep = cache / f"{key.replace(':', '_')}.brep"
            if not BRepTools.Write_s(shape, str(brep)):
                raise ValueError(f"Could not cache CAD shape {name}")
            entry["brep"] = brep.name
        definitions.append(entry)
        shape_map[key] = shape
        print(f"PART {key} {name} {'ASSEMBLY' if entry['assembly'] else 'part'} {size}", flush=True)
    instances = []
    roots = Sequence_TDF_Label()
    shapes.GetFreeShapes(roots)

    def visit(label, location, path):
        key = label_key(label)
        name = label_name(label)
        current = path + [name or key]
        instances.append({"definition": key, "name": name, "path": current,
                          "transform_mm": matrix(location)})
        components = Sequence_TDF_Label()
        shapes.GetComponents_s(label, components)
        for index in range(1, components.Length() + 1):
            component = components.Value(index)
            reference = TDF_Label()
            target = component
            if shapes.IsReference_s(component):
                shapes.GetReferredShape_s(component, reference)
                target = reference
            visit(target, location.Multiplied(shapes.GetLocation_s(component)), current)
    for index in range(1, roots.Length() + 1):
        visit(roots.Value(index), TopLoc_Location(), [])
    result = {"source_step": step.name, "source_step_sha256": digest(step), "source_units": "millimetres",
              "catalog_bounds_method": "OpenCASCADE conservative BRep bounds; exported selected assets use exact surface extrema",
              "definitions": definitions, "instances": instances}
    (cache / "catalog.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(f"Catalogued {len(definitions)} definitions and {len(instances)} placements.", flush=True)
    return result, shape_map, colors, doc


def read_brep(path):
    from OCP.TopoDS import TopoDS_Shape
    from OCP.BRep import BRep_Builder
    from OCP.BRepTools import BRepTools
    shape = TopoDS_Shape()
    if not BRepTools.Read_s(shape, str(path), BRep_Builder()):
        raise ValueError(f"Cannot read cached CAD shape {path}")
    return shape


def axial_surfaces(shape):
    """Report actual analytical cylindrical/conical surfaces, in CAD millimetres."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Cone
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    surfaces = {}
    while explorer.More():
        face = (getattr(TopoDS, "Face_s", None) or TopoDS.Face)(explorer.Current())
        surface = BRepAdaptor_Surface(face, True)
        kind = surface.GetType()
        if kind in (GeomAbs_Cylinder, GeomAbs_Cone):
            geometry = surface.Cylinder() if kind == GeomAbs_Cylinder else surface.Cone()
            radius = geometry.Radius() if kind == GeomAbs_Cylinder else geometry.RefRadius()
            if radius >= 2:
                direction = geometry.Axis().Direction()
                origin = geometry.Location()
                entry = {"type": "cylinder" if kind == GeomAbs_Cylinder else "cone",
                         "radius_mm": radius,
                         "axis": [direction.X(), direction.Y(), direction.Z()],
                         "origin_mm": [origin.X(), origin.Y(), origin.Z()],
                         "v_range_mm": [surface.FirstVParameter(), surface.LastVParameter()],
                         "bounds_mm": bbox(face)}
                if kind == GeomAbs_Cone:
                    entry["semi_angle_rad"] = geometry.SemiAngle()
                key = json.dumps(entry, sort_keys=True)
                surfaces[key] = entry
        explorer.Next()
    return list(surfaces.values())


def rotation_for_axis(axis):
    return {"z": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "y": [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
            "x": [[0, 1, 0], [0, 0, 1], [1, 0, 0]]}[axis]


def matmul(left, right):
    return [[sum(a * b for a, b in zip(row, column)) for column in zip(*right)]
            for row in left]


def matvec(rotation, vector):
    return [sum(a * b for a, b in zip(row, vector)) for row in rotation]


def euler_xyz(rotation):
    """URDF fixed-axis roll/pitch/yaw, including a stable gimbal-lock branch."""
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll = math.atan2(-rotation[1][2], rotation[1][1])
        yaw = 0.0
    return [0.0 if abs(value) < 1e-10 else value for value in (roll, pitch, yaw)]


def field_placements(catalog, assets):
    """Preserve official assembly transforms in the simulator's SI floor frame.

    The official CAD has +Y up and foam top at CAD Y=-294.5892 mm.
    Exported mesh coordinates may have an additional normalization offset;
    incorporate that offset into every placement rather than moving vertices.
    """
    global_rotation = rotation_for_axis("y")
    floor_y_mm = -294.5892
    rows = []
    for source_index, instance in enumerate(catalog["instances"]):
        name = instance["name"]
        colors = {}
        if name.startswith("276-9250-80x_"):
            key = "pin"
            colors = ({"upper": "blue", "lower": "red"} if "Red-Blue" in name else
                      {"upper": "red", "lower": "yellow"} if "Red-Neutral" in name else
                      {"upper": "blue", "lower": "yellow"} if "Blue-Neutral" in name else
                      {"upper": "yellow", "lower": "yellow"})
        elif name == "276-9250-81x":
            key, colors = "cup", {"upper": "gray", "lower": "clear"}
        elif name.startswith("276-9250-840_"):
            key = ("goal_center" if "Neutral Goal 0" in name else
                   "goal_neutral" if "Neutral" in name else "goal_alliance")
            colors = {"body": "red" if "Red" in name else "blue" if "Blue" in name else "black"}
        elif name.startswith("276-9250-830_"):
            key = "loader"
            colors = {"body": "red" if "831" in name else "blue"}
        elif name.startswith("276-9250-120_"):
            key = "toggle"
            colors = {"alliance": "red" if "Red" in name else "blue"}
        elif name == "276-7596-014":
            key, colors = "white_panel", {"body": "white"}
        elif name == "276-7596-015":
            key, colors = "white_panel_short", {"body": "white"}
        else:
            continue
        asset_key = "loader_blue" if key == "loader" and colors["body"] == "blue" and "loader_blue" in assets else key
        if asset_key not in assets:
            continue
        asset = assets[asset_key]
        source_transform = instance["transform_mm"]
        source_rotation = [row[:3] for row in source_transform[:3]]
        source_translation = [row[3] for row in source_transform[:3]]
        inverse_mesh_rotation = list(map(list, zip(*asset["cad_to_mesh_rotation"])))
        local_to_world = matmul(global_rotation, matmul(source_rotation, inverse_mesh_rotation))
        normalized_offset = matvec(local_to_world, asset["normalization_offset_oriented_mm"])
        source_origin = matvec(global_rotation, source_translation)
        source_origin[2] -= floor_y_mm
        position = [(a + b) * .001 for a, b in zip(source_origin, normalized_offset)]
        rows.append({"source_instance_index": source_index, "source_definition": name,
                     "source_definition_id": instance["definition"], "kind": key,
                     "asset_key": asset_key,
                     "mesh": asset["mesh"], "colors": colors,
                     "position": position, "rpy": euler_xyz(local_to_world),
                     "rotation_matrix": local_to_world,
                     "assembly_origin_m": [value * .001 for value in source_origin],
                     "in_field": abs(source_translation[0]) < 1900 and abs(source_translation[2]) < 1900})
    return {"units": "metres", "angle_units": "radians",
            "cad_to_world_rotation": global_rotation,
            "cad_floor_top_y_mm": floor_y_mm,
            "formula": "world=(CAD_X, -CAD_Z, CAD_Y+294.5892)*0.001; mesh normalization included in position",
            "tile_pitch_m": .598058534,
            "instances": rows}


def mechanism_metadata(catalog, assets):
    if "toggle_fixed" not in assets or "toggle_rotor" not in assets:
        return {}
    rows = catalog["instances"]
    start = next(i for i, row in enumerate(rows) if row["definition"] == assets["toggle"]["source_definition_id"])
    parent = rows[start]
    rotor = next(row for row in rows[start + 1:]
                 if row["name"] == "276-9250-023_GREY")
    inverse_parent = list(map(list, zip(*[row[:3] for row in parent["transform_mm"][:3]])))
    translation = [rotor["transform_mm"][i][3] - parent["transform_mm"][i][3] for i in range(3)]
    local_position = matvec(inverse_parent, translation)
    rotor_axis = [row[0] for row in rotor["transform_mm"][:3]]
    local_axis = matvec(inverse_parent, rotor_axis)
    asset = assets["toggle"]
    oriented = matvec(asset["cad_to_mesh_rotation"], local_position)
    pivot = [(a - b) * .001 for a, b in zip(oriented, asset["normalization_offset_oriented_mm"])]
    axis = matvec(asset["cad_to_mesh_rotation"], local_axis)
    return {"toggle": {"fixed_mesh": assets["toggle_fixed"]["mesh"],
                        "rotor_mesh": assets["toggle_rotor"]["mesh"],
                        "pivot_m": pivot, "axis": axis,
                        "zero_angle": "Authored official CAD rotor pose",
                        "source_pivot": "Origin and +X axis of component 276-9250-023_GREY",
                        "visual_origin_from_joint_m": [-value for value in pivot]}}


def mesh_sections(path, planes):
    """Measure actual triangle/plane intersections, including radial minima.

    These measurements are of the tessellation, not an exact BRep section.
    They retain the export's stated chord tolerance and do not certify a
    Hausdorff bound for a separately simplified collision mesh.
    """
    vertices, faces = [], []
    with Path(path).open(encoding="utf-8") as source:
        for line in source:
            if line.startswith("v "):
                vertices.append(tuple(map(float, line.split()[1:4])))
            elif line.startswith("f "):
                faces.append(tuple(int(part.split("/")[0]) - 1 for part in line.split()[1:4]))
    result = []
    for plane in planes:
        minimum, maximum = math.inf, 0.0
        low, high = [math.inf, math.inf], [-math.inf, -math.inf]
        count = 0
        for indices in faces:
            points = [vertices[index] for index in indices]
            if plane < min(point[2] for point in points) - 1e-10 or plane > max(point[2] for point in points) + 1e-10:
                continue
            crossings = []
            for a, b in zip(points, points[1:] + points[:1]):
                if abs(a[2] - plane) < 1e-10:
                    crossings.append(a[:2])
                if (a[2] - plane) * (b[2] - plane) < 0:
                    weight = (plane - a[2]) / (b[2] - a[2])
                    crossings.append(tuple(a[i] + weight * (b[i] - a[i]) for i in range(2)))
            if len(crossings) < 2:
                continue
            count += 1
            for a in crossings:
                radius = math.hypot(*a)
                minimum, maximum = min(minimum, radius), max(maximum, radius)
                low = [min(x, y) for x, y in zip(low, a)]
                high = [max(x, y) for x, y in zip(high, a)]
            for a, b in zip(crossings, crossings[1:] + crossings[:1]):
                edge = [b[i] - a[i] for i in range(2)]
                length_squared = sum(value * value for value in edge)
                if length_squared > 1e-20:
                    weight = max(0.0, min(1.0, -sum(x * y for x, y in zip(a, edge)) / length_squared))
                    minimum = min(minimum, math.hypot(*(a[i] + weight * edge[i] for i in range(2))))
        if count:
            result.append({"plane_z_m": plane, "minimum_surface_radius_m": minimum,
                           "maximum_surface_radius_m": maximum, "xy_bounds_m": [low, high],
                           "intersected_triangles": count})
    return result


def measured_profiles(assets, output):
    profiles = {}
    for key, cad_planes in {"pin": [-80, -60, -40, -20, 0, 20, 40, 60, 80],
                            "cup": [-80, -60, -40, -20, 0, 20, 40, 60, 80],
                            "goal_socket": [0, 20, 40, 60, 80]}.items():
        if key not in assets:
            continue
        asset = assets[key]
        offset = asset["normalization_offset_oriented_mm"][2]
        sections = mesh_sections(output / f"{key}.obj", [(value - offset) * .001 for value in cad_planes])
        for section in sections:
            section["cad_axis_coordinate_mm"] = section["plane_z_m"] * 1000 + offset
        profiles[key] = {"source_mesh_sha256": asset["mesh_sha256"],
                         "method": "Triangle/plane intersections; radius around local +Z axis, including point-to-segment minimum",
                         "mesh_chord_tolerance_m": asset["visual_chord_tolerance_m"],
                         "sections": sections}
    return profiles


def export_mesh(shape, destination, axis="y", origin="base", color=None, colors=None,
                linear_deflection_mm=.25, normalization_bounds=None, origin_offset_mm=None):
    import numpy as np
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
    from OCP.TopoDS import TopoDS
    from OCP.TopLoc import TopLoc_Location
    from OCP.BRep import BRep_Tool
    rotation = np.asarray(rotation_for_axis(axis), dtype=float)
    shape_bounds = np.asarray(bbox(shape))
    raw_bounds = np.asarray(normalization_bounds if normalization_bounds is not None else shape_bounds)
    corners = np.array([[x, y, z] for x in raw_bounds[:, 0] for y in raw_bounds[:, 1] for z in raw_bounds[:, 2]])
    indices, signs = {"z": ([0, 1, 2], [1, 1, 1]), "y": ([0, 2, 1], [1, -1, 1]),
                      "x": ([1, 2, 0], [1, 1, 1])}[axis]
    oriented = corners[:, indices] * signs
    low, high = oriented.min(axis=0), oriented.max(axis=0)
    offset = (low + high) / 2
    if origin == "base":
        offset[2] = low[2]
    if origin_offset_mm is not None:
        offset = np.asarray(origin_offset_mm, dtype=float)
    actual_corners = np.array([[x, y, z] for x in shape_bounds[:, 0]
                               for y in shape_bounds[:, 1] for z in shape_bounds[:, 2]])
    actual_oriented = actual_corners[:, indices] * signs
    actual_low, actual_high = actual_oriented.min(axis=0), actual_oriented.max(axis=0)
    BRepMesh_IncrementalMesh(shape, linear_deflection_mm, False, .25, True)
    materials, vertices, triangles = {}, 0, 0
    destination.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"# Derived from official VEX Override 2026-27 CAD; metres; {SOURCE_URL}",
             f"mtllib {destination.with_suffix('.mtl').name}"]
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = (getattr(TopoDS, "Face_s", None) or TopoDS.Face)(explorer.Current())
        location = TopLoc_Location()
        triangulation = BRep_Tool.Triangulation_s(face, location)
        if triangulation is not None:
            points = []
            for index in range(1, triangulation.NbNodes() + 1):
                point = triangulation.Node(index).Transformed(location.Transformation())
                points.append([point.X(), point.Y(), point.Z()])
            points = (np.asarray(points)[:, indices] * signs - offset) * .001
            faces = []
            for index in range(1, triangulation.NbTriangles() + 1):
                ids = list(triangulation.Triangle(index).Get())
                if face.Orientation() == TopAbs_REVERSED:
                    ids[1], ids[2] = ids[2], ids[1]
                faces.append([value - 1 for value in ids])
            faces = np.asarray(faces, dtype=int)
            normals = np.zeros_like(points)
            if len(faces):
                surface_normals = np.cross(points[faces[:, 1]] - points[faces[:, 0]],
                                           points[faces[:, 2]] - points[faces[:, 0]])
                for corner in range(3):
                    np.add.at(normals, faces[:, corner], surface_normals)
                lengths = np.linalg.norm(normals, axis=1)
                normals /= np.maximum(lengths[:, None], 1e-20)
            rgba = color_of(face, colors) if colors is not None else None
            rgba = rgba or color or [0.6, 0.6, 0.62, 1]
            key = tuple(round(float(channel), 5) for channel in rgba)
            if key not in materials:
                materials[key] = f"cad_color_{len(materials)}"
            lines.append(f"usemtl {materials[key]}")
            lines += ["v " + " ".join(f"{value:.8g}" for value in point) for point in points]
            lines += ["vn " + " ".join(f"{value:.8g}" for value in normal) for normal in normals]
            lines += ["f " + " ".join(f"{vertices + index + 1}//{vertices + index + 1}" for index in triangle) for triangle in faces]
            vertices += len(points)
            triangles += len(faces)
        explorer.Next()
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    material_text = []
    for rgba, name in materials.items():
        material_text += [f"newmtl {name}", "Kd " + " ".join(map(str, rgba[:3])),
                          "Ka 0.15 0.15 0.15", "Ks 0.08 0.08 0.08", f"d {rgba[3]}", "Ns 20", ""]
    destination.with_suffix(".mtl").write_text("\n".join(material_text), encoding="utf-8")
    return {"mesh": str(destination), "mesh_sha256": digest(destination), "vertices": vertices,
            "triangles": triangles, "units": "metres", "source_up_axis": axis,
            "origin": origin, "cad_to_mesh_rotation": rotation.tolist(),
            "normalization_offset_oriented_mm": offset.tolist(),
            "bounds_m": [((actual_low - offset) * .001).tolist(), ((actual_high - offset) * .001).tolist()],
            "size_m": ((actual_high - actual_low) * .001).tolist(),
            "visual_chord_tolerance_m": linear_deflection_mm * .001,
            "analytic_surfaces": axial_surfaces(shape)}


def split_solids(shape, axis):
    """Group authored whole solids by their axial center, without cutting faces."""
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopAbs import TopAbs_SOLID
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    limits = bbox(shape)
    middle = (limits[0][axis_index] + limits[1][axis_index]) / 2
    builder = BRep_Builder()
    groups = {"upper": TopoDS_Compound(), "lower": TopoDS_Compound()}
    counts = {"upper": 0, "lower": 0}
    for compound in groups.values():
        builder.MakeCompound(compound)
    explorer = TopExp_Explorer(shape, TopAbs_SOLID)
    while explorer.More():
        solid = explorer.Current()
        bounds = bbox(solid)
        center = (bounds[0][axis_index] + bounds[1][axis_index]) / 2
        key = "upper" if center >= middle else "lower"
        builder.Add(groups[key], solid)
        counts[key] += 1
        explorer.Next()
    if not all(counts.values()):
        raise ValueError("This definition does not contain distinct authored solids for upper/lower halves")
    return groups


def select_components(shape, catalog, definition, indices):
    """Select complete located CAD components, preserving their assembly frame."""
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound, TopoDS_Iterator
    rows = catalog["instances"]
    start = next(i for i, row in enumerate(rows) if row["definition"] == definition)
    depth = len(rows[start]["path"])
    direct = []
    for row in rows[start + 1:]:
        if len(row["path"]) <= depth:
            break
        if len(row["path"]) == depth + 1:
            direct.append(row)
    builder = BRep_Builder()
    result = TopoDS_Compound()
    builder.MakeCompound(result)
    iterator = TopoDS_Iterator(shape)
    index = 0
    selected = []
    while iterator.More():
        if index in indices:
            builder.Add(result, iterator.Value())
            selected.append({"component_index": index, "definition": direct[index]["definition"],
                             "name": direct[index]["name"]})
        index += 1
        iterator.Next()
    if index != len(direct) or len(selected) != len(indices):
        raise ValueError("Component selection does not match the authored assembly hierarchy")
    return result, selected


def export_selection(catalog, cache, output, selection, step, archive=None, reuse_assets=False):
    assets = {}
    manifest_path = output.parent.parent / "cad_manifest.json"
    previous = {}
    if reuse_assets and manifest_path.is_file():
        old = json.loads(manifest_path.read_text(encoding="utf-8"))
        if old["source"]["step_sha256"] != catalog["source_step_sha256"]:
            raise ValueError("Existing assets were generated from a different STEP source")
        previous = old["assets"]
    for spec in selection:
        cached = previous.get(spec["key"])
        required = [spec["key"]] + ([spec["key"] + "_upper", spec["key"] + "_lower"] if spec.get("split_halves") else [])
        if cached and cached.get("selection_spec") == spec and all(
                key in previous and (output / f"{key}.obj").is_file() and
                digest(output / f"{key}.obj") == previous[key]["mesh_sha256"] for key in required):
            assets.update({key: previous[key] for key in required})
            print(f"Verified and reused {spec['key']}", flush=True)
            continue
        name = spec["definition"]
        matches = [entry for entry in catalog["definitions"] if entry["name"] == name and "brep" in entry
                   and ("definition_id" not in spec or entry["id"] == spec["definition_id"])]
        if len(matches) != 1:
            raise ValueError(f"Expected one cached definition '{name}', found {len(matches)}")
        entry = matches[0]
        print(f"Meshing {spec['key']}: {name}", flush=True)
        shape = read_brep(cache / entry["brep"])
        original_bounds = bbox(shape)
        selected_components = None
        if "component_indices" in spec:
            shape, selected_components = select_components(shape, catalog, entry["id"], spec["component_indices"])
        result = export_mesh(shape, output / f"{spec['key']}.obj", spec.get("axis", "y"),
                             spec.get("origin", "base"), spec.get("color", entry.get("color_rgba")),
                             normalization_bounds=original_bounds,
                             linear_deflection_mm=spec.get("linear_deflection_mm", .25),
                             origin_offset_mm=spec.get("origin_offset_mm"))
        result.update(source_definition=name, source_definition_id=entry["id"],
                      source_bounds_mm=entry["bounds_mm"],
                      selection_spec=spec,
                      instances=[item for item in catalog["instances"] if item["definition"] == entry["id"]])
        if selected_components is not None:
            result["selected_components"] = selected_components
        result["mesh"] = str(Path("meshes") / output.name / f"{spec['key']}.obj").replace("\\", "/")
        assets[spec["key"]] = result
        if spec.get("split_halves"):
            for half, compound in split_solids(shape, spec.get("axis", "y")).items():
                key = f"{spec['key']}_{half}"
                half_result = export_mesh(compound, output / f"{key}.obj", spec.get("axis", "y"),
                                          spec.get("origin", "base"), spec.get("color"),
                                          normalization_bounds=bbox(shape),
                                          origin_offset_mm=spec.get("origin_offset_mm"))
                half_result.update(source_definition=name, source_definition_id=entry["id"],
                                   split_method="Whole authored solids grouped by axial center; no geometry was cut",
                                   mesh=f"meshes/{output.name}/{key}.obj", parent_asset=spec["key"])
                assets[key] = half_result
        print(f"  {result['triangles']} triangles, size {result['size_m']} m", flush=True)
    manifest = {"schema_version": 1, "source": {"publisher": "VEX Robotics",
                "title": "V5RC Override 2026-2027 official field CAD",
                "url": SOURCE_URL, "step_file": step.name, "step_sha256": catalog["source_step_sha256"],
                "step_units": "millimetres", "archive_sha256": digest(archive) if archive else None},
                "conversion": {"tool": "tools/import_vex_cad.py", "kernel": "OpenCASCADE / cadquery-ocp",
                "cadquery_ocp_version": "8.0.1.0.0", "angular_deflection_rad": .25,
                "mesh_units": "metres", "visual_chord_tolerance_m": .00025,
                "note": "Visual meshes derive from exact STEP BReps; collision models are authored separately using these measured dimensions."},
                "attribution": "Derived from VEX Robotics official field CAD. Original CAD and design rights remain with their respective owners; this conversion does not apply the repository's software license to VEX geometry.",
                "assets": assets, "assembly_definitions": catalog["definitions"],
                "assembly_instances": catalog["instances"]}
    manifest["field_placements"] = field_placements(catalog, assets)
    manifest["mechanisms"] = mechanism_metadata(catalog, assets)
    manifest["measured_profiles"] = measured_profiles(assets, output)
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {manifest_path}", flush=True)
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path)
    parser.add_argument("--step", type=Path)
    parser.add_argument("--cache", type=Path, default=ROOT / ".sim-tools" / "official-cad-cache")
    parser.add_argument("--output", type=Path, default=ROOT / "assets" / "meshes" / "override")
    parser.add_argument("--catalog-only", action="store_true")
    parser.add_argument("--selection", type=Path, help="JSON list of {key,definition,axis,origin,color} entries")
    parser.add_argument("--reuse-catalog", action="store_true")
    parser.add_argument("--reuse-assets", action="store_true", help="Verify and reuse meshes with identical source and selection")
    args = parser.parse_args()
    args.cache.mkdir(parents=True, exist_ok=True)
    if args.step:
        step = args.step.resolve()
    elif args.archive:
        with zipfile.ZipFile(args.archive) as archive:
            entries = [name for name in archive.namelist() if Path(name).suffix.lower() in {".step", ".stp"}]
            if len(entries) != 1:
                raise ValueError("The official archive must contain exactly one STEP assembly")
            step = args.cache / Path(entries[0]).name
            if not step.exists():
                step.write_bytes(archive.read(entries[0]))
    else:
        parser.error("Supply --archive or --step")
    start = time.perf_counter()
    if args.reuse_catalog:
        catalog = json.loads((args.cache / "catalog.json").read_text(encoding="utf-8"))
        if catalog["source_step_sha256"] != digest(step):
            raise ValueError("Cached catalogue does not match the selected STEP source")
    else:
        catalog, _, _, _ = catalogue(step, args.cache)
    if not args.catalog_only:
        if not args.selection:
            parser.error("Supply --selection with measured part names, or --catalog-only")
        selection = json.loads(args.selection.read_text(encoding="utf-8"))
        export_selection(catalog, args.cache, args.output, selection, step, args.archive, args.reuse_assets)
    print(f"Finished in {time.perf_counter()-start:.1f}s", flush=True)


if __name__ == "__main__":
    main()
