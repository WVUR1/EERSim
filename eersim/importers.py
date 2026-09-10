"""Standard URDF inspection and one-file field import, without physics dependencies.

Inspection never modifies a URDF or its mesh assets. Generated packs and URDFs
use relative asset references so their surrounding asset tree can be relocated.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any
import xml.etree.ElementTree as ET


_MOVABLE = {"continuous", "revolute", "prismatic"}
_JOINT_TYPES = _MOVABLE | {"fixed", "floating", "planar"}


def default_import_directory(workspace: str | Path | None = None) -> Path:
    """Keep generated imports writable outside a frozen installation folder."""
    if workspace:
        return Path(workspace).expanduser().resolve() / ".eersim" / "imports"
    if getattr(sys, "frozen", False):
        from .runtime_paths import user_data_dir
        return user_data_dir() / "imports"
    return Path.cwd() / ".eersim" / "imports"


def _file(path: str | Path, label: str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise ValueError(f"{label} does not exist or is not a file: {resolved}")
    return resolved


def _number(value: Any, label: str, *, minimum: float | None = None,
            positive: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a number") from exc
    if not math.isfinite(result) or (minimum is not None and result < minimum) or (positive and result <= 0):
        requirement = "positive and finite" if positive else f"finite and at least {minimum}" if minimum is not None else "finite"
        raise ValueError(f"{label} must be {requirement}")
    return result


def _vector(value: Any, count: int, label: str, *, positive: bool = False) -> list[float]:
    if isinstance(value, str):
        value = value.split()
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{label} must contain {count} numbers")
    return [_number(item, label, positive=positive) for item in value]


def _asset_path(filename: str, directory: Path, label: str) -> Path:
    if not filename:
        raise ValueError(f"{label} has an empty filename")
    if filename.startswith("package://"):
        package, separator, resource = filename[len("package://"):].partition("/")
        if not package or not separator or not resource:
            raise ValueError(f"{label} uses malformed ROS package URI '{filename}'")
        # Common CAD exports use package/urdf/model.urdf and package/meshes/*.
        # Check explicit known paths only; never scan an entire ROS workspace.
        anchors = [directory, *list(directory.parents)[:3]]
        candidates = {
            (anchor / suffix).resolve()
            for anchor in anchors for suffix in (Path(resource), Path(package) / resource)
        }
        found = sorted(path for path in candidates if path.is_file())
        if len(found) == 1:
            return found[0]
        if len(found) > 1:
            raise ValueError(f"{label} package URI '{filename}' is ambiguous: " +
                             "; ".join(str(path) for path in found) +
                             ". Use a relative mesh filename to select the intended asset.")
        raise ValueError(f"{label} uses unresolved ROS package URI '{filename}'. "
                         "Keep the package's mesh folder beside the URDF or within its nearest three parent folders, "
                         "or update the filename to a local relative path.")
    if "://" in filename:
        raise ValueError(f"{label} uses URI '{filename}'. Use a local absolute or relative file path for this simulator.")
    path = Path(filename).expanduser()
    if not path.is_absolute():
        path = directory / path
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f"{label} file is missing: {path}. Keep the exported mesh folder beside its URDF, "
                         "or update the URDF filename to the correct relative path.")
    return path


def _relative_reference(path: Path, directory: Path) -> str:
    """Keep references portable, except across Windows drives or UNC shares."""
    try:
        return Path(os.path.relpath(path, directory)).as_posix()
    except ValueError:
        return path.as_posix()


def _prepare_urdf(path: Path) -> Path:
    """Normalize locally resolved ROS assets in a generated copy, only if needed."""
    root = ET.parse(path).getroot()
    assets = [element for element in root.iter() if element.tag in {"mesh", "texture"} and element.get("filename")]
    if not any(element.get("filename", "").startswith("package://") for element in assets):
        return path
    destination_directory = default_import_directory()
    for element in assets:
        resolved = _asset_path(element.get("filename", ""), path.parent, f"URDF {element.tag}")
        element.set("filename", _relative_reference(resolved, destination_directory))
    text = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    digest = hashlib.sha256(str(path).encode() + text).hexdigest()[:12]
    destination = destination_directory / f"{path.stem}-{digest}.urdf"
    destination.parent.mkdir(parents=True, exist_ok=True)
    if not destination.is_file() or destination.read_bytes() != text:
        destination.write_bytes(text)
    return destination.resolve()


def _inspect(path: str | Path) -> dict[str, Any]:
    path = _file(path, "URDF")
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise ValueError(f"Cannot read URDF XML '{path.name}': {exc}") from exc
    if root.tag != "robot":
        raise ValueError("A URDF must have a <robot> root element. Export URDF, rather than SDF, MJCF or Xacro.")
    if any(element.tag.startswith("{http://www.ros.org/wiki/xacro}") for element in root.iter()):
        raise ValueError("This file contains Xacro instructions. Expand it into a standard URDF before importing.")
    links: dict[str, ET.Element] = {}
    warnings: list[str] = []
    for link in root.findall("link"):
        name = link.get("name", "").strip()
        if not name:
            raise ValueError("Every URDF link needs a nonempty name")
        if name in links:
            raise ValueError(f"URDF contains duplicate link name '{name}'")
        links[name] = link
    if not links:
        raise ValueError("The URDF contains no links")
    missing_inertia = []
    missing_collision = []
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            missing_inertia.append(name)
        else:
            mass = inertial.find("mass")
            inertia = inertial.find("inertia")
            if mass is None or mass.get("value") is None or inertia is None:
                raise ValueError(f"Link '{name}' has incomplete <inertial>; supply mass and inertia, or remove the empty element.")
            _number(mass.get("value"), f"Link '{name}' mass", minimum=0)
            for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
                if inertia.get(key) is None:
                    raise ValueError(f"Link '{name}' inertia is missing {key}")
                _number(inertia.get(key), f"Link '{name}' inertia.{key}", minimum=0 if key in {"ixx", "iyy", "izz"} else None)
        if link.find("collision") is None:
            missing_collision.append(name)
    if missing_inertia:
        warnings.append("No mass/inertia authored for: " + ", ".join(missing_inertia) +
                        ". Bullet defaults will be used; add measured inertia for accurate dynamics.")
    if missing_collision:
        warnings.append("No collision geometry for: " + ", ".join(missing_collision) +
                        ". These links will not contact the field.")
    for element in root.iter("origin"):
        for key in ("xyz", "rpy"):
            if key in element.attrib:
                _vector(element.attrib[key], 3, f"URDF origin.{key}")
    for mesh in root.iter("mesh"):
        _asset_path(mesh.get("filename", ""), path.parent, "URDF mesh")
        if "scale" in mesh.attrib:
            _vector(mesh.attrib["scale"], 3, "URDF mesh.scale", positive=True)
    package_assets = [element for element in root.iter()
                      if element.tag in {"mesh", "texture"} and element.get("filename", "").startswith("package://")]
    for element in package_assets:
        _asset_path(element.get("filename", ""), path.parent, f"URDF {element.tag}")
    if package_assets:
        warnings.append("Local ROS package assets resolved. Import will use a generated URDF with local mesh references; the original file is preserved.")
    for geometry in root.iter("geometry"):
        shapes = list(geometry)
        if len(shapes) != 1:
            raise ValueError("Every URDF geometry element must contain exactly one box, cylinder, sphere or mesh")
        shape = shapes[0]
        if shape.tag == "box":
            _vector(shape.get("size", ""), 3, "URDF box.size", positive=True)
        elif shape.tag in {"sphere", "cylinder"}:
            _number(shape.get("radius"), f"URDF {shape.tag}.radius", positive=True)
            if shape.tag == "cylinder":
                _number(shape.get("length"), "URDF cylinder.length", positive=True)
        elif shape.tag != "mesh":
            raise ValueError(f"URDF geometry '{shape.tag}' is not standard URDF; export it as a mesh")
    joints = []
    seen_names = set()
    parent_of: dict[str, str] = {}
    for joint in root.findall("joint"):
        name = joint.get("name", "").strip()
        kind = joint.get("type", "")
        if not name or name in seen_names:
            raise ValueError(f"Every URDF joint needs a unique, nonempty name (got '{name}')")
        seen_names.add(name)
        if kind not in _JOINT_TYPES:
            raise ValueError(f"Joint '{name}' has unknown type '{kind}'")
        parent, child = joint.find("parent"), joint.find("child")
        parent_name = parent.get("link") if parent is not None else None
        child_name = child.get("link") if child is not None else None
        if parent_name not in links or child_name not in links:
            raise ValueError(f"Joint '{name}' must reference existing parent and child links")
        if child_name in parent_of:
            raise ValueError(f"Link '{child_name}' has more than one parent joint; URDF must be a tree")
        parent_of[child_name] = parent_name
        axis = joint.find("axis")
        if axis is not None:
            xyz = _vector(axis.get("xyz", ""), 3, f"Joint '{name}' axis")
            if sum(v * v for v in xyz) < 1e-12:
                raise ValueError(f"Joint '{name}' axis must not be zero")
        limit = joint.find("limit")
        info: dict[str, Any] = {"name": name, "type": kind, "lower": None, "upper": None,
                                "effort": None, "velocity": None}
        if limit is not None:
            for key in ("lower", "upper", "effort", "velocity"):
                if key in limit.attrib:
                    info[key] = _number(limit.attrib[key], f"Joint '{name}' {key}",
                                        minimum=0 if key in {"effort", "velocity"} else None)
        if kind in {"revolute", "prismatic"}:
            if info["lower"] is None or info["upper"] is None:
                raise ValueError(f"Joint '{name}' needs lower and upper limits; use type='continuous' for an unlimited hinge")
            if info["lower"] > info["upper"]:
                raise ValueError(f"Joint '{name}' lower limit exceeds its upper limit")
        if kind == "continuous":
            info["lower"] = info["upper"] = None
        if kind in _MOVABLE and (info["effort"] is None or info["velocity"] is None):
            warnings.append(f"Joint '{name}' has no complete effort/velocity limit; set limits for meaningful actuator testing.")
        if kind in {"floating", "planar"}:
            warnings.append(f"Joint '{name}' is {kind}; the motor binding editor supports continuous, revolute and prismatic joints.")
        if joint.find("mimic") is not None:
            warnings.append(f"Joint '{name}' uses mimic. URDF mimic coupling is not automatically enforced; model its linkage explicitly.")
        joints.append(info)
    for name in links:
        visited = set()
        current = name
        while current in parent_of:
            if current in visited:
                raise ValueError(f"URDF contains a joint cycle at link '{current}'")
            visited.add(current)
            current = parent_of[current]
    roots = set(links) - set(parent_of)
    if len(roots) != 1:
        raise ValueError(f"URDF must have one connected root link; found {len(roots)} ({', '.join(sorted(roots))})")
    return {"name": root.get("name") or path.stem, "path": str(path), "joints": joints,
            "warnings": warnings, "links": list(links), "root_link": next(iter(roots))}


def _sidecar(path: Path, joints: list[dict[str, Any]]) -> tuple[Path | None, dict, list[str]]:
    candidate = path.with_suffix(".json")
    if not candidate.is_file():
        return None, {}, []
    try:
        data = json.loads(candidate.read_text(encoding="utf-8-sig"))
    except (ValueError, OSError) as exc:
        return None, {}, [f"Adjacent {candidate.name} could not be read as robot bindings: {exc}"]
    if not isinstance(data, dict) or not any(key in data for key in ("motors", "drive", "pneumatics")):
        return None, {}, []
    if any(key in data for key in ("objects", "field", "robot_urdf", "game_pack")):
        return None, {}, []
    if data.get("schema_version", 1) != 1:
        raise ValueError(f"Robot sidecar {candidate.name} uses an unsupported schema_version")
    if "drive" in data and not isinstance(data["drive"], dict):
        raise ValueError(f"Robot sidecar {candidate.name}: drive must be an object")
    if "pneumatics" in data and not isinstance(data["pneumatics"], list):
        raise ValueError(f"Robot sidecar {candidate.name}: pneumatics must be a list")
    if data.get("urdf"):
        referenced = (candidate.parent / data["urdf"]).resolve()
        if referenced != path:
            return None, {}, [f"Adjacent {candidate.name} refers to a different URDF; its bindings were not imported."]
    motors = data.get("motors", [])
    if not isinstance(motors, list):
        raise ValueError(f"Robot sidecar {candidate.name}: motors must be a list")
    joint_names = {joint["name"] for joint in joints if joint["type"] in _MOVABLE}
    seen_ports = set()
    for motor in motors:
        if not isinstance(motor, dict) or "port" not in motor or "joint" not in motor:
            raise ValueError(f"Robot sidecar {candidate.name}: every motor needs a port and joint")
        raw_port = motor["port"]
        if isinstance(raw_port, bool) or not isinstance(raw_port, int) or not 1 <= abs(raw_port) <= 21:
            raise ValueError(f"Robot sidecar {candidate.name}: motor ports must be integers from +/-1 to +/-21")
        if abs(raw_port) in seen_ports:
            raise ValueError(f"Robot sidecar {candidate.name}: duplicate motor port {abs(raw_port)}")
        seen_ports.add(abs(raw_port))
        if motor["joint"] not in joint_names:
            raise ValueError(f"Robot sidecar {candidate.name}: motor {raw_port} references missing or non-movable joint '{motor['joint']}'")
        for key in ("rpm", "gear_ratio", "stall_torque", "max_power", "efficiency", "lead_m"):
            if key in motor:
                _number(motor[key], f"Motor {raw_port} {key}", positive=True)
    return candidate, data, []


def inspect_robot_urdf(path: str | Path) -> dict[str, Any]:
    """Return links, joint limits and actionable authoring warnings for a URDF."""
    info = _inspect(path)
    _, _, warnings = _sidecar(Path(info["path"]), info["joints"])
    info["warnings"].extend(warnings)
    return info


def create_robot_config(path: str | Path) -> dict[str, Any]:
    """Return an experiment patch; a plain URDF starts with unassigned motors.

    A recognized same-stem robot JSON preserves its full hardware bindings,
    including multiple physical motors attached to the same joint.
    """
    info = _inspect(path)
    sidecar, data, _ = _sidecar(Path(info["path"]), info["joints"])
    return {"robot_urdf": str(_prepare_urdf(Path(info["path"]))), "robot_config": str(sidecar) if sidecar else "",
            "motors": deepcopy(data.get("motors", [])), "motor_overrides": {}}


def _validate_field(data: Any, source: Path) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError("A field JSON file must contain an object")
    if data.get("schema_version", 1) != 1:
        raise ValueError("Only field schema_version 1 is supported")
    if not isinstance(data.get("field"), dict) or not isinstance(data.get("objects"), list):
        raise ValueError("A field JSON must contain a 'field' object and an 'objects' list. "
                         "Select a game field pack, rather than an experiment or robot binding JSON.")
    data = deepcopy(data)
    data.setdefault("schema_version", 1)
    data.setdefault("name", source.stem)
    if "import_metadata" in data:
        if not isinstance(data["import_metadata"], dict):
            raise ValueError("Field import_metadata must be an object")
        for key in ("warnings", "normalized_urdfs"):
            if key in data["import_metadata"] and not isinstance(data["import_metadata"][key], list):
                raise ValueError(f"Field import_metadata.{key} must be a list")
    field = data["field"]
    _vector(field.get("size", [3.6576, 3.6576]), 2, "field.size", positive=True)
    for key in ("wall_height", "tile_size"):
        if key in field:
            _number(field[key], f"field.{key}", positive=True)
    for key in ("walls", "floor"):
        if key in field and not isinstance(field[key], bool):
            raise ValueError(f"field.{key} must be true or false")
    if "mode_durations" in data:
        if not isinstance(data["mode_durations"], dict):
            raise ValueError("mode_durations must be an object")
        for mode, seconds in data["mode_durations"].items():
            _number(seconds, f"mode_durations.{mode}", positive=True)
    for index, item in enumerate(data["objects"]):
        label = f"objects[{index}]"
        if not isinstance(item, dict):
            raise ValueError(f"{label} must be an object")
        for key in ("position", "rotation"):
            _vector(item.get(key, [0, 0, 0]), 3, f"{label}.{key}")
        if "color" in item:
            color = _vector(item["color"], 4, f"{label}.color")
            if any(channel < 0 or channel > 1 for channel in color):
                raise ValueError(f"{label}.color RGBA channels must be between 0 and 1")
        for key in ("mass", "friction", "rolling_friction"):
            if key in item:
                _number(item[key], f"{label}.{key}", minimum=0)
        if "restitution" in item and not 0 <= _number(item["restitution"], f"{label}.restitution") <= 1:
            raise ValueError(f"{label}.restitution must be between 0 and 1")
        for key in ("fixed", "collision"):
            if key in item and not isinstance(item[key], bool):
                raise ValueError(f"{label}.{key} must be true or false")
        if "urdf" in item:
            if not isinstance(item["urdf"], str):
                raise ValueError(f"{label}.urdf must be a path string")
            asset = _asset_path(item["urdf"], source.parent, f"{label}.urdf")
            _inspect(asset)
            normalized = _prepare_urdf(asset)
            item["urdf"] = str(normalized)
            if normalized != asset:
                metadata = data.setdefault("import_metadata", {})
                if not isinstance(metadata, dict):
                    raise ValueError("Field import_metadata must be an object")
                metadata.setdefault("normalized_urdfs", []).append(str(normalized))
            _number(item.get("scale", 1), f"{label}.scale", positive=True)
        else:
            kind = item.get("shape", "box")
            if kind == "box":
                _vector(item.get("size", [.1, .1, .1]), 3, f"{label}.size", positive=True)
            elif kind in {"sphere", "cylinder", "capsule"}:
                _number(item.get("radius", .05), f"{label}.radius", positive=True)
                if kind != "sphere":
                    _number(item.get("height", .1), f"{label}.height", positive=True)
            else:
                raise ValueError(f"{label}.shape '{kind}' is unsupported; use box, sphere, cylinder, capsule or an URDF object")
    return data


def load_field_import(path: str | Path) -> dict[str, Any]:
    """Read a JSON field pack or wrap a standard URDF as a fixed field assembly."""
    path = _file(path, "Field")
    if path.suffix.lower() == ".urdf":
        info = _inspect(path)
        return {"schema_version": 1, "name": info["name"], "season": "Imported field",
                "description": f"Field assembly imported from {path.name}. Geometry and contact surfaces come from its URDF.",
                "field": {"size": [3.6576, 3.6576], "walls": False, "floor": False},
                "objects": [{"name": info["name"], "urdf": str(_prepare_urdf(path)), "fixed": True,
                             "position": [0, 0, 0], "rotation": [0, 0, 0]}],
                "import_metadata": {"source": str(path), "warnings": [
                    "Display bounds default to a 12 ft square; change field.size for a different field.",
                    "The imported URDF supplies its own floor and walls. No additional collision floor is generated.",
                    *info["warnings"]]}}
    if path.suffix.lower() != ".json":
        raise ValueError("Select a field JSON pack or a standard .urdf field assembly")
    try:
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    except (ValueError, OSError) as exc:
        raise ValueError(f"Cannot read field JSON '{path.name}': {exc}") from exc
    return _validate_field(data, path)


def materialize_field_import(path: str | Path, output_path: str | Path) -> Path:
    """Write a normalized JSON field pack to an explicit destination filename."""
    source = _file(path, "Field")
    output = Path(output_path).expanduser().resolve()
    if output.suffix.lower() != ".json":
        raise ValueError("The imported field output filename must end in .json")
    if output == source:
        raise ValueError("Choose a new output filename to preserve the source field file")
    data = load_field_import(source)
    _write_field_pack(data, source, output)
    return output


def _write_field_pack(data: dict[str, Any], source: Path, output: Path) -> None:
    """Serialize resolved import data relative to its new file, leaving it intact."""
    portable = deepcopy(data)
    for item in portable["objects"]:
        if "urdf" in item:
            item["urdf"] = _relative_reference(Path(item["urdf"]), output.parent)
    metadata = portable.setdefault("import_metadata", {})
    metadata.setdefault("source", str(source))

    def metadata_reference(value: Any) -> Any:
        if not isinstance(value, str) or not value or "://" in value:
            return value
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = source.parent / path
        return _relative_reference(path.resolve(), output.parent)

    metadata["source"] = metadata_reference(metadata["source"])
    if "normalized_urdfs" in metadata:
        metadata["normalized_urdfs"] = [metadata_reference(value) for value in metadata["normalized_urdfs"]]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(portable, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def import_field(path: str | Path, dest: str | Path | None = None) -> dict[str, Any]:
    """Import a field; dest is a .json filename or a directory for generated packs.

    JSON imports without dest use the validated source file unless asset paths
    require normalization. Generated packs use per-user EERSim/imports in the
    frozen app, or cwd/.eersim/imports from source. An explicit destination or
    a GUI-selected workspace takes precedence for generated field packs.
    """
    source = _file(path, "Field")
    data = load_field_import(source)
    output = source
    needs_normalization = bool(data.get("import_metadata", {}).get("normalized_urdfs"))
    if dest is not None or source.suffix.lower() == ".urdf" or needs_normalization:
        destination = Path(dest).expanduser().resolve() if dest is not None else default_import_directory()
        if destination.suffix.lower() == ".json" and not destination.is_dir():
            output = destination
        else:
            digest = hashlib.sha256((str(source) + json.dumps(data, sort_keys=True)).encode()).hexdigest()[:10]
            output = destination / f"{source.stem}-{digest}.field.json"
        if output == source:
            raise ValueError("Choose a destination different from the source field file")
        _write_field_pack(data, source, output)
    return {"path": str(output), "metadata": {
        "name": data.get("name", source.stem), "description": data.get("description", ""),
        "object_count": len(data["objects"]), "field": deepcopy(data["field"]),
        "source": str(source), "warnings": list(data.get("import_metadata", {}).get("warnings", []))}}
