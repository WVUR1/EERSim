"""Generate measured visual LODs from the attributed VEX OBJ exports.

This tool never changes collision geometry or the source CAD exports. It keeps
material partitions and the source mesh's coordinate frame/bounds. The reported
bidirectional distances sample both surfaces; they are NOT a certified Hausdorff
bound, and the original STEP meshing tolerance does not apply to these LODs.

Requires numpy and VTK (the CAD conversion environment supplies both).
"""
from __future__ import annotations

import argparse
from collections import OrderedDict
import hashlib
import json
import math
import os
from pathlib import Path
import time

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("OMP_NUM_THREADS", "1")

import numpy as np
import vtk
from vtk.util.numpy_support import numpy_to_vtk, numpy_to_vtkIdTypeArray, vtk_to_numpy

ROOT = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_obj(path: str | Path):
    """Read triangle/polygon OBJ geometry while preserving each material name.

    Textured inputs are rejected: the official CAD exports use MTL diffuse colors,
    and collapsing a textured mesh requires a different attribute-aware pipeline.
    """
    path = Path(path)
    points = []
    groups: dict[str, list[list[int]]] = OrderedDict()
    material = "default"
    material_files = []
    comments = []
    with path.open(encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            fields = line.split()
            if not fields:
                continue
            kind = fields[0]
            if kind == "#" and len(comments) < 8:
                comments.append(line.rstrip())
            elif kind == "v":
                if len(fields) < 4:
                    raise ValueError(f"{path.name}:{line_number}: incomplete vertex")
                points.append([float(value) for value in fields[1:4]])
            elif kind == "vt":
                raise ValueError(f"{path.name}: texture coordinates require an attribute-preserving decimator")
            elif kind == "usemtl":
                material = " ".join(fields[1:])
                if not material:
                    raise ValueError(f"{path.name}:{line_number}: empty material name")
            elif kind == "mtllib":
                material_files.extend(fields[1:])
            elif kind == "f":
                indices = []
                for field in fields[1:]:
                    index = int(field.split("/", 1)[0])
                    index = index - 1 if index > 0 else len(points) + index
                    if not 0 <= index < len(points):
                        raise ValueError(f"{path.name}:{line_number}: invalid vertex index")
                    indices.append(index)
                if len(indices) < 3:
                    raise ValueError(f"{path.name}:{line_number}: a face needs three vertices")
                destination = groups.setdefault(material, [])
                for index in range(1, len(indices) - 1):
                    destination.append([indices[0], indices[index], indices[index + 1]])
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not groups:
        raise ValueError(f"{path.name}: no surface geometry")
    if not np.isfinite(points).all():
        raise ValueError(f"{path.name}: coordinates must be finite")
    materials = []
    for filename in material_files:
        candidate = (path.parent / filename).resolve()
        if not candidate.is_file():
            raise ValueError(f"{path.name}: missing material library {filename}")
        materials.append(candidate.read_text(encoding="utf-8-sig"))
    if not materials:
        materials = ["newmtl default\nKd 0.65 0.65 0.68\nKa 0.15 0.15 0.15\n"]
    return points, {key: np.asarray(value, dtype=np.int64) for key, value in groups.items()}, "\n".join(materials), comments


def polydata(points: np.ndarray, triangles: np.ndarray):
    poly = vtk.vtkPolyData()
    vertices = vtk.vtkPoints()
    vertices.SetData(numpy_to_vtk(np.ascontiguousarray(points, dtype=np.float64), deep=True))
    poly.SetPoints(vertices)
    cells = vtk.vtkCellArray()
    offsets = np.arange(0, triangles.size + 1, 3, dtype=np.int64)
    cells.SetData(numpy_to_vtkIdTypeArray(offsets, deep=True),
                  numpy_to_vtkIdTypeArray(np.ascontiguousarray(triangles.reshape(-1)), deep=True))
    poly.SetPolys(cells)
    return poly


def arrays(poly):
    points = vtk_to_numpy(poly.GetPoints().GetData()).copy()
    cells = vtk_to_numpy(poly.GetPolys().GetConnectivityArray()).copy()
    offsets = vtk_to_numpy(poly.GetPolys().GetOffsetsArray())
    if not np.all(np.diff(offsets) == 3):
        raise ValueError("Decimation produced non-triangle cells")
    return points, cells.reshape(-1, 3)


def clean_surface(points, faces, weld_tolerance_m=1e-8):
    cleaner = vtk.vtkCleanPolyData()
    cleaner.SetInputData(polydata(points, faces))
    cleaner.ToleranceIsAbsoluteOn()
    cleaner.SetAbsoluteTolerance(weld_tolerance_m)
    cleaner.ConvertPolysToLinesOff()
    cleaner.ConvertLinesToPointsOff()
    cleaner.Update()
    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputConnection(cleaner.GetOutputPort())
    triangles.PassVertsOff()
    triangles.PassLinesOff()
    triangles.Update()
    result = vtk.vtkPolyData()
    result.DeepCopy(triangles.GetOutput())
    return result


def reduce_surface(source, reduction):
    decimator = vtk.vtkQuadricDecimation()
    decimator.SetInputData(source)
    decimator.SetTargetReduction(reduction)
    decimator.VolumePreservationOn()
    decimator.SetBoundaryWeightFactor(10.0)
    decimator.AttributeErrorMetricOff()
    decimator.Update()
    result = vtk.vtkPolyData()
    result.DeepCopy(decimator.GetOutput())
    if result.GetNumberOfPolys() < 1:
        result.DeepCopy(source)
    return result


def joined(parts):
    merger = vtk.vtkAppendPolyData()
    for item in parts:
        merger.AddInputData(item)
    merger.Update()
    result = vtk.vtkPolyData()
    result.DeepCopy(merger.GetOutput())
    return result


def bounds(points):
    return np.stack([points.min(axis=0), points.max(axis=0)])


def surface_samples(surface, count: int, seed: int):
    """All unique vertices plus deterministic area-weighted triangle samples."""
    points, faces = arrays(surface)
    vertices = np.unique(points, axis=0)
    corners = points[faces]
    areas = np.linalg.norm(np.cross(corners[:, 1] - corners[:, 0], corners[:, 2] - corners[:, 0]), axis=1)
    if not areas.sum() > 0:
        raise ValueError("The source has no nonzero-area triangles")
    random = np.random.default_rng(seed)
    chosen = random.choice(len(faces), size=count, p=areas / areas.sum())
    root_u = np.sqrt(random.random(count))
    v = random.random(count)
    weights = np.column_stack([1 - root_u, root_u * (1 - v), root_u * v])
    interiors = np.einsum("ij,ijk->ik", weights, corners[chosen])
    return np.concatenate([vertices, interiors]), len(vertices)


def distance_samples(points, target):
    distance = vtk.vtkImplicitPolyDataDistance()
    distance.SetInput(target)
    values = vtk.vtkDoubleArray()
    values.SetNumberOfComponents(1)
    values.SetNumberOfTuples(len(points))
    distance.FunctionValue(numpy_to_vtk(np.ascontiguousarray(points), deep=True), values)
    return np.abs(vtk_to_numpy(values).copy())


def measure_deviation(original, reduced, samples=10000, seed=7):
    first, source_vertices = surface_samples(original, samples, seed)
    second, lod_vertices = surface_samples(reduced, samples, seed + 1)
    forward = distance_samples(first, reduced)
    backward = distance_samples(second, original)
    values = np.concatenate([forward, backward])
    return {
        "method": "Unsigned nearest-triangle distances in both directions: all unique mesh vertices plus seeded area-weighted surface samples per direction.",
        "limitation": "Sampled mesh-to-mesh deviation, not a certified maximum or Hausdorff bound. The source STEP chord tolerance is not a final LOD tolerance.",
        "surface_samples_per_direction": samples, "seed": seed,
        "source_vertex_samples": source_vertices, "lod_vertex_samples": lod_vertices,
        "source_to_lod_max_m": float(forward.max()), "lod_to_source_max_m": float(backward.max()),
        "combined_max_m": float(values.max()), "combined_rms_m": float(np.sqrt(np.mean(values ** 2))),
        "combined_p95_m": float(np.quantile(values, .95)), "sample_count": len(values),
    }


def write_obj(destination, parts, material_text, source_comments, purpose="visual"):
    destination.parent.mkdir(parents=True, exist_ok=True)
    offset = 0
    vertex_count = triangle_count = 0
    with destination.open("w", encoding="utf-8", newline="\n") as stream:
        for comment in source_comments:
            stream.write(comment + "\n")
        stream.write(f"# Simplified {purpose} mesh produced by tools/decimate_vex_cad.py; see its deviation manifest.\n")
        stream.write(f"mtllib {destination.with_suffix('.mtl').name}\n")
        for name, part in parts.items():
            normals = vtk.vtkPolyDataNormals()
            normals.SetInputData(part)
            normals.SetFeatureAngle(40)
            normals.SplittingOn()
            normals.ConsistencyOn()
            normals.AutoOrientNormalsOff()
            normals.ComputePointNormalsOn()
            normals.ComputeCellNormalsOff()
            normals.Update()
            output = normals.GetOutput()
            points, faces = arrays(output)
            vectors = vtk_to_numpy(output.GetPointData().GetNormals())
            stream.write(f"usemtl {name}\n")
            for point in points:
                stream.write("v " + " ".join(f"{value:.10g}" for value in point) + "\n")
            for vector in vectors:
                stream.write("vn " + " ".join(f"{value:.8g}" for value in vector) + "\n")
            for face in faces:
                ids = face + offset + 1
                stream.write("f " + " ".join(f"{index}//{index}" for index in ids) + "\n")
            offset += len(points)
            vertex_count += len(points)
            triangle_count += len(faces)
    destination.with_suffix(".mtl").write_text(material_text, encoding="utf-8")
    return vertex_count, triangle_count


def decimate_file(source: str | Path, destination: str | Path, *, reduction=.96, samples=10000, seed=7,
                  nominal_bounds=None, purpose="visual"):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source == destination:
        raise ValueError("The LOD destination must differ from the original OBJ")
    if not 0 <= reduction < 1 or samples < 1:
        raise ValueError("reduction must be in [0,1), and samples must be positive")
    source_hash = digest(source)
    points, groups, material_text, comments = read_obj(source)
    originals = {name: clean_surface(points, faces) for name, faces in groups.items()}
    parts = {name: reduce_surface(surface, reduction) for name, surface in originals.items()}
    original = joined(originals.values())
    original_points, original_faces = arrays(original)
    original_bounds = bounds(original_points)
    reduced_points, _ = arrays(joined(parts.values()))
    reduced_bounds = bounds(reduced_points)
    old_extent = reduced_bounds[1] - reduced_bounds[0]
    new_extent = original_bounds[1] - original_bounds[0]
    scale = np.divide(new_extent, old_extent, out=np.ones(3), where=old_extent > 1e-14)
    translation = original_bounds[0] - reduced_bounds[0] * scale
    # Preserve the source coordinate frame and nominal extents. Report the small
    # affine correction explicitly, and measure distances after applying it.
    parts = {name: polydata(arrays(part)[0] * scale + translation, arrays(part)[1])
             for name, part in parts.items()}
    reduced = joined(parts.values())
    lod_points, lod_faces = arrays(reduced)
    deviations = measure_deviation(original, reduced, samples, seed)
    vertex_count, triangle_count = write_obj(destination, parts, material_text, comments, purpose)
    if digest(source) != source_hash:
        raise RuntimeError("The source mesh changed during decimation")
    result = {
        "source": source.name, "source_sha256": source_hash, "mesh": destination.name, "purpose": purpose,
        "mesh_sha256": digest(destination), "material_sha256": digest(destination.with_suffix('.mtl')),
        "units": "metres", "source_triangles": sum(len(group) for group in groups.values()),
        "cleaned_source_triangles": len(original_faces), "lod_triangles": triangle_count,
        "lod_vertices_with_normals": vertex_count, "actual_reduction": 1 - triangle_count / sum(len(g) for g in groups.values()),
        "source_bounds_m": original_bounds.tolist(), "lod_bounds_m": bounds(lod_points).tolist(),
        "bounds_preservation_scale": scale.tolist(), "bounds_preservation_translation_m": translation.tolist(),
        "material_names": list(parts), "deviation": deviations,
    }
    if nominal_bounds is not None:
        nominal = np.asarray(nominal_bounds, dtype=float)
        if nominal.shape == (2, 3):
            result["cad_manifest_nominal_bounds_m"] = nominal.tolist()
            result["lod_vs_cad_nominal_bounds_max_error_m"] = float(np.max(np.abs(bounds(lod_points) - nominal)))
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=ROOT / "assets" / "meshes" / "override")
    parser.add_argument("--output", type=Path, default=ROOT / "assets" / "meshes" / "override" / "lod")
    parser.add_argument("--manifest", type=Path, default=ROOT / "assets" / "cad_manifest.json")
    parser.add_argument("--reduction", type=float, default=.96, help="Fraction of triangles to remove; default .96 = about 25× fewer triangles")
    parser.add_argument("--samples", type=int, default=10000, help="Extra area-weighted samples per direction, in addition to every unique vertex")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--only", nargs="*", help="Optional OBJ stems to process")
    parser.add_argument("--purpose", choices=["visual", "collision"], default="visual", help="Record intended usage; collision outputs still require contact validation")
    args = parser.parse_args(argv)
    if args.source.resolve() == args.output.resolve():
        parser.error("Use a separate output directory to preserve the original exports")
    metadata = json.loads(args.manifest.read_text(encoding="utf-8-sig")) if args.manifest.is_file() else {}
    files = sorted(args.source.glob("*.obj"))
    if args.only:
        selected = set(args.only)
        files = [path for path in files if path.stem in selected]
        missing = selected - {path.stem for path in files}
        if missing:
            parser.error("Missing source OBJ: " + ", ".join(sorted(missing)))
    if not files:
        parser.error("No source OBJ files found")
    args.output.mkdir(parents=True, exist_ok=True)
    record = {
        "schema_version": 1, "tool": "tools/decimate_vex_cad.py", "vtk_version": vtk.vtkVersion.GetVTKVersion(),
        "source_provenance": metadata.get("source", {}), "attribution": metadata.get("attribution", ""),
        "algorithm": "Material-partitioned VTK quadric decimation with volume preservation; original mesh bounds restored; normals regenerated at 40° creases.",
        "requested_reduction": args.reduction, "weld_tolerance_m": 1e-8,
        "usage": args.purpose + " approximation; original CAD exports are unchanged. Collision use requires separate contact tests.",
        "tolerance_warning": "The original CAD export tolerance does not bound this LOD. See each asset's sampled mesh-to-mesh deviation.",
        "assets": {},
    }
    for source in files:
        began = time.perf_counter()
        nominal = metadata.get("assets", {}).get(source.stem, {}).get("bounds_m")
        result = decimate_file(source, args.output / source.name, reduction=args.reduction,
                               samples=args.samples, seed=args.seed, nominal_bounds=nominal, purpose=args.purpose)
        record["assets"][source.stem] = result
        print(f"{source.stem}: {result['source_triangles']} -> {result['lod_triangles']} triangles; sampled max {result['deviation']['combined_max_m'] * 1000:.3f} mm; {time.perf_counter() - began:.1f} s", flush=True)
        (args.output / "lod_manifest.json").write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
