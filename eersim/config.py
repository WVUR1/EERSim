"""Portable, versioned simulation experiments. All physical quantities use SI.

Only the start-pose heading and explicitly named sensor noise fields use degrees.
Saved experiments use asset:// references for bundled assets and paths relative
to the experiment JSON file for user files. Windows paths on a different drive
or UNC share remain absolute because those locations have no relative path.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field, fields
import json
import math
import os
from pathlib import Path, PurePosixPath, PureWindowsPath
import sys
from typing import Any

ASSETS = Path(__file__).resolve().parent.parent / "assets"
if not ASSETS.is_dir():
    ASSETS = Path(sys.prefix) / "share" / "eersim" / "assets"

_ASSET_PREFIX = "asset://"
_PATH_FIELDS = ("robot_urdf", "robot_config", "game_pack", "workspace")


def _asset_path(reference: str) -> Path:
    """Resolve a portable asset name without permitting escape from ASSETS."""
    relative = reference[len(_ASSET_PREFIX):]
    if (not relative or "\\" in relative or ":" in relative or "\0" in relative
            or any(part in ("", ".", "..") for part in relative.split("/"))
            or PurePosixPath(relative).is_absolute() or PureWindowsPath(relative).drive):
        raise ValueError(f"Invalid asset reference: {reference!r}; use asset://folder/file")
    root = ASSETS.resolve()
    candidate = (root / relative).resolve()
    if not candidate.is_relative_to(root):
        raise ValueError(f"Asset reference escapes the bundled assets directory: {reference!r}")
    return candidate


def _resolve_path(value: str | Path, directory: Path) -> Path:
    reference = os.fspath(value)
    if reference.startswith(_ASSET_PREFIX):
        return _asset_path(reference)
    candidate = Path(reference).expanduser()
    return (candidate if candidate.is_absolute() else directory / candidate).resolve()


def _portable_path(value: str | Path, directory: Path) -> str:
    # In-memory relative paths have the same working-directory interpretation
    # as the physics API. Loaded profiles already contain resolved paths.
    candidate = _resolve_path(value, Path.cwd())
    root = ASSETS.resolve()
    if candidate.is_relative_to(root):
        return _ASSET_PREFIX + candidate.relative_to(root).as_posix()
    try:
        return Path(os.path.relpath(candidate, directory)).as_posix()
    except ValueError:
        # Windows cannot express paths relative to a different drive/share.
        return candidate.as_posix()


def _field_objects(data: dict[str, Any]):
    """Yield embedded field objects without interpreting unrelated extras."""
    objects = data.get("field_objects", [])
    if isinstance(objects, list):
        yield from (item for item in objects if isinstance(item, dict))


def default_config() -> dict[str, Any]:
    return SimConfig().to_dict()


@dataclass
class SimConfig:
    robot_urdf: str = field(default_factory=lambda: str(ASSETS / "robots" / "xdrive.urdf"))
    robot_config: str = field(default_factory=lambda: str(ASSETS / "robots" / "xdrive.json"))
    game_pack: str = field(default_factory=lambda: str(ASSETS / "games" / "override-2026-27.json"))
    start_pose: list[float] = field(default_factory=lambda: [-1.45, -0.90, 0.085, 0.0])
    physics_hz: int = 240
    physics_backend: str = "auto"
    physics_device: int = 0
    physics_solver_iterations: int = 100
    gravity: float = 9.80665
    friction: float = 0.9
    plastic_friction: float = 0.5
    restitution: float = 0.05
    seed: int = 7
    noise: dict[str, float] = field(default_factory=lambda: {
        "position_std_m": 0.0, "heading_std_deg": 0.0,
        "encoder_std_deg": 0.0, "distance_std_mm": 0.0,
        "latency_ms": 0.0, "dropout": 0.0,
    })
    motor_overrides: dict[str, dict[str, Any]] = field(default_factory=dict)
    motors: list[dict[str, Any]] | None = None
    # Extra application settings (workspace, GUI camera, experiment notes, etc.)
    # survive save/load without coupling the physics module to the GUI.
    extra: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any] | "SimConfig" | None) -> "SimConfig":
        if isinstance(data, cls):
            data = data.to_dict()
        data = deepcopy(data or {})
        names = {f.name for f in fields(cls)} - {"extra"}
        extra = data.pop("extra", {})
        extra.update({key: value for key, value in data.items() if key not in names})
        config = cls(**{key: value for key, value in data.items() if key in names}, extra=extra)
        config.noise = {**cls().noise, **config.noise}
        config.validate()
        return config

    def validate(self) -> None:
        if self.physics_backend not in ("auto", "bullet", "physx_gpu", "physx_cpu"):
            raise ValueError("physics_backend must be auto, bullet, physx_gpu, or physx_cpu")
        if (isinstance(self.physics_device, bool) or not isinstance(self.physics_device, int)
                or self.physics_device < 0):
            raise ValueError("physics_device must be a nonnegative integer CUDA device index")
        if (isinstance(self.physics_solver_iterations, bool)
                or not isinstance(self.physics_solver_iterations, int)
                or not 1 <= self.physics_solver_iterations <= 255):
            raise ValueError("physics_solver_iterations must be an integer between 1 and 255")
        if not isinstance(self.physics_hz, int) or not 120 <= self.physics_hz <= 2000:
            raise ValueError("physics_hz must be an integer between 120 and 2000")
        if len(self.start_pose) != 4 or not all(math.isfinite(float(x)) for x in self.start_pose):
            raise ValueError("start_pose must contain finite x, y, z (metres), heading (degrees)")
        if not 0 <= float(self.gravity) <= 100:
            raise ValueError("gravity must be between 0 and 100 m/s²")
        if not 0 <= float(self.friction) <= 5:
            raise ValueError("friction must be between 0 and 5")
        if not 0 <= float(self.plastic_friction) <= 5:
            raise ValueError("plastic_friction must be between 0 and 5")
        if not 0 <= float(self.restitution) <= 1:
            raise ValueError("restitution must be between 0 and 1")
        for key, value in self.noise.items():
            if not math.isfinite(float(value)) or float(value) < 0:
                raise ValueError(f"noise.{key} must be finite and nonnegative")
        if self.noise["dropout"] > 1:
            raise ValueError("noise.dropout must be between 0 and 1")
        if self.noise["latency_ms"] > 10000:
            raise ValueError("noise.latency_ms cannot exceed 10000")

    def to_dict(self) -> dict[str, Any]:
        result = deepcopy(self.extra)
        result.update({f.name: deepcopy(getattr(self, f.name)) for f in fields(self) if f.name != "extra"})
        return result


def load_config(path: str | Path) -> dict[str, Any]:
    path = Path(path).expanduser().resolve()
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("An experiment must be a JSON object")
    for key in _PATH_FIELDS:
        if data.get(key):
            data[key] = str(_resolve_path(data[key], path.parent))
    for item in _field_objects(data):
        reference = item.get("urdf")
        if isinstance(reference, str) and reference.startswith(_ASSET_PREFIX):
            item["urdf"] = str(_asset_path(reference))
    return SimConfig.from_dict(data).to_dict()


def save_config(config: dict[str, Any] | SimConfig, path: str | Path) -> None:
    path = Path(path).expanduser().resolve()
    data = SimConfig.from_dict(config).to_dict()
    # Embedded object URDFs traditionally resolve against the game-pack
    # directory. Preserve user references; make bundled demo objects portable.
    game_directory = (_resolve_path(data["game_pack"], Path.cwd()).parent
                      if data.get("game_pack") else Path.cwd())
    for item in _field_objects(data):
        if item.get("urdf"):
            candidate = _resolve_path(item["urdf"], game_directory)
            if candidate.is_relative_to(ASSETS.resolve()):
                item["urdf"] = _ASSET_PREFIX + candidate.relative_to(ASSETS.resolve()).as_posix()
    for key in _PATH_FIELDS:
        if data.get(key):
            data[key] = _portable_path(data[key], path.parent)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n", encoding="utf-8")
