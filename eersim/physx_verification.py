"""Verify the live native scene instead of treating requested CUDA as evidence.

Call after a normal-timestep bootstrap, while the scene and runtime are alive.
This module never steps the simulation or initializes a second CUDA context.
"""
from __future__ import annotations

import ctypes
from importlib.metadata import version
import math
import os
from pathlib import Path
import sys

from .physx_probe_build import LIBRARY_NAME, NATIVE, TAG, build_probe, source_digest

PHYSX_HEADER_VERSION = 0x050A0000
REQUIRED_GPU_FLAGS = 1 | 2 | 8


def probe_library_path() -> Path:
    """Use a release payload, explicit override, or a source build in user data."""
    override = os.environ.get("EERSIM_PHYSX_PROBE_LIBRARY")
    if override:
        path = Path(override).expanduser().resolve(strict=True)
        if not path.is_file():
            raise RuntimeError("EERSIM_PHYSX_PROBE_LIBRARY must name a native library file")
        return path
    bundled = NATIVE / LIBRARY_NAME
    if bundled.is_file():
        return bundled
    if getattr(sys, "frozen", False):
        raise RuntimeError("The installed EERSim GPU verification library is missing; reinstall this release")
    from .runtime_paths import user_data_dir
    cache = user_data_dir() / "physics/native-probe" / source_digest()
    output = cache / LIBRARY_NAME
    if not output.is_file():
        build_probe(output, cache=cache)
    return output


def _load_library(path: Path):
    library = ctypes.CDLL(str(path))
    for name, result in (("eersim_scene_gpu_flags", ctypes.c_uint),
                         ("eersim_scene_cuda_device", ctypes.c_int),
                         ("eersim_scene_cuda_name", ctypes.c_char_p)):
        function = getattr(library, name)
        function.argtypes = [ctypes.c_void_p]
        function.restype = result
    library.eersim_probe_physx_version.argtypes = []
    library.eersim_probe_physx_version.restype = ctypes.c_uint
    if library.eersim_probe_physx_version() != PHYSX_HEADER_VERSION:
        raise RuntimeError("PhysX verification library was built against incompatible headers")
    return library


def inspect_scene(physx, scene_path: str = "/World/physicsScene", *, library_path: Path | None = None) -> dict:
    """Return actual native flags/device information for a live scene, including CPU."""
    if version("ovphysx") != "0.5.11":
        raise RuntimeError(f"Live scene verification requires the pinned runtime {TAG}")
    library = _load_library(Path(library_path).resolve(strict=True) if library_path is not None else probe_library_path())
    pointer = _scene_pointer(physx, scene_path)
    flags = int(library.eersim_scene_gpu_flags(pointer))
    device = int(library.eersim_scene_cuda_device(pointer))
    name = library.eersim_scene_cuda_name(pointer)
    return {
        "scene_gpu_flags": flags,
        "logical_cuda_device": device,
        "cuda_device_name": name.decode("utf-8", "replace") if name else "",
        "direct_gpu_api": bool(flags & 4),
        "physx_header_version": PHYSX_HEADER_VERSION,
    }


def _scene_pointer(physx, scene_path: str):
    from ovphysx import _bindings
    get_pointer = physx._lib.ovphysx_get_physx_ptr
    get_pointer.argtypes = [ctypes.c_uint64, _bindings.ovphysx_string_t, ctypes.c_int,
                           ctypes.POINTER(ctypes.c_void_p)]
    get_pointer.restype = _bindings.ovphysx_result_t
    pointer = ctypes.c_void_p()
    status = get_pointer(physx._omni_physx_sdk_handle.value, _bindings.ovphysx_string_t(scene_path),
                         1, ctypes.byref(pointer))  # OVPHYSX_PHYSX_TYPE_SCENE in 0.5.11.
    if status.status != 0 or not pointer.value:
        raise RuntimeError(f"Could not retrieve live PhysX scene {scene_path!r} for GPU verification")
    return pointer


def verify_gpu_scene(physx, scene_path: str = "/World/physicsScene", *, library_path: Path | None = None) -> dict:
    """Raise if GPU dynamics/broadphase/context were not actually enabled."""
    result = inspect_scene(physx, scene_path, library_path=library_path)
    flags = result["scene_gpu_flags"]
    if flags & REQUIRED_GPU_FLAGS != REQUIRED_GPU_FLAGS or result["logical_cuda_device"] < 0:
        raise RuntimeError("PhysX did not enable the requested GPU physics backend: "
                           f"live flags={flags}, CUDA device={result['logical_cuda_device']}. "
                           "GPU dynamics, GPU broadphase and a valid CUDA context are all required; "
                           "CPU fallback cannot run as a GPU session.")
    return result


class _ActivityRow(ctypes.Structure):
    _fields_ = [("name", ctypes.c_char_p), ("kind", ctypes.c_uint),
                ("sleeping", ctypes.c_uint), ("link_count", ctypes.c_uint),
                ("wake_counter", ctypes.c_float), ("linear_speed_squared", ctypes.c_double),
                ("angular_speed_squared", ctypes.c_double)]


_GEOMETRY_TYPES = ("sphere", "plane", "capsule", "box", "convex_core", "convex_mesh",
                   "particle_system", "tetrahedron_mesh", "triangle_mesh", "heightfield", "custom")
_STATISTICS_SCALARS = ("cpu_worker_count", "active_constraints", "active_dynamic_bodies",
    "active_kinematic_bodies", "static_bodies", "dynamic_bodies", "kinematic_bodies", "articulations",
    "axis_solver_constraints", "discrete_contact_pairs_total", "discrete_contact_pairs_with_cache_hits",
    "discrete_contact_pairs_with_contacts", "geometry_count")


class _ActivityStatistics(ctypes.Structure):
    _fields_ = [(name, ctypes.c_uint) for name in _STATISTICS_SCALARS] + [
        ("shape_counts", ctypes.c_uint * 11), ("discrete_contact_pairs", ctypes.c_uint * 121)]


def _activity_functions(library):
    """Require the added ABI only for opt-in activity, preserving old GPU probes."""
    signatures = {
        "eersim_activity_abi_version": ([], ctypes.c_uint),
        "eersim_activity_row_size": ([], ctypes.c_uint),
        "eersim_activity_statistics_size": ([], ctypes.c_uint),
        "eersim_scene_activity": ([ctypes.c_void_p, ctypes.POINTER(_ActivityRow),
                                   ctypes.c_uint, ctypes.POINTER(ctypes.c_uint)], ctypes.c_int),
        "eersim_scene_activity_statistics": ([ctypes.c_void_p, ctypes.POINTER(_ActivityStatistics)], ctypes.c_int),
    }
    for name, (arguments, result) in signatures.items():
        try:
            function = getattr(library, name)
        except AttributeError as exc:
            raise RuntimeError("The native helper lacks activity diagnostics; rebuild or update the helper") from exc
        function.argtypes, function.restype = arguments, result
    if (library.eersim_activity_abi_version() != 1 or
            library.eersim_activity_row_size() != ctypes.sizeof(_ActivityRow) or
            library.eersim_activity_statistics_size() != ctypes.sizeof(_ActivityStatistics)):
        raise RuntimeError("The native activity diagnostics ABI is incompatible")


def _check_activity_status(status: int) -> None:
    if status == -2:
        raise RuntimeError("Native activity diagnostics need CPU readback; DirectGPU getters are unavailable")
    if status != 0:
        raise RuntimeError(f"Native activity diagnostics failed (status={status}); query a quiescent scene after fetch")


def inspect_scene_activity(physx, scene_path: str = "/World/physicsScene", *,
                           library_path: Path | None = None) -> dict:
    """Read sleep/velocity/statistics without stepping or changing the scene.

    The caller must have completed step_sync/fetch and must not concurrently
    step, edit or release the scene. This bounded batch query is intended for
    occasional diagnostics, not every simulation tick. Native names can be null
    and are reported as supplied; they are not assumed to be USD prim paths.
    """
    if version("ovphysx") != "0.5.11":
        raise RuntimeError(f"Live scene activity requires the pinned runtime {TAG}")
    library = _load_library(Path(library_path).resolve(strict=True) if library_path is not None else probe_library_path())
    _activity_functions(library)
    pointer = _scene_pointer(physx, scene_path)
    flags = int(library.eersim_scene_gpu_flags(pointer))
    if flags & 4:
        _check_activity_status(-2)
    required = ctypes.c_uint()
    status = int(library.eersim_scene_activity(pointer, None, 0, ctypes.byref(required)))
    if status not in (0, 1):
        _check_activity_status(status)
    if required.value > 1_000_000:
        raise RuntimeError("Native activity diagnostics exceeded the bounded actor count")
    capacity = required.value
    rows = (_ActivityRow * max(capacity, 1))()
    _check_activity_status(int(library.eersim_scene_activity(pointer, rows, capacity, ctypes.byref(required))))
    if required.value != capacity:
        raise RuntimeError("Native scene topology changed during activity diagnostics")

    counts = dict.fromkeys(("static_actors", "regular_dynamic_actors", "regular_sleeping", "regular_awake",
        "regular_awake_moving", "regular_awake_zero_velocity", "articulations", "articulations_sleeping",
        "articulations_awake", "articulations_awake_moving", "articulations_awake_zero_velocity",
        "articulation_links", "articulation_links_sleeping", "articulation_links_awake"), 0)
    actors = []
    max_linear = max_angular = 0.0
    for row in rows[:capacity]:
        if row.kind not in (0, 1, 2) or row.sleeping not in (0, 1):
            raise RuntimeError("Native activity diagnostics returned an invalid actor type or sleep flag")
        if row.kind != 2 and row.link_count:
            raise RuntimeError("Native activity diagnostics returned links for a regular actor")
        values = (row.wake_counter, row.linear_speed_squared, row.angular_speed_squared)
        if not all(math.isfinite(value) and value >= 0 for value in values):
            raise RuntimeError("Native activity diagnostics returned nonfinite or negative motion values")
        linear, angular = math.sqrt(row.linear_speed_squared), math.sqrt(row.angular_speed_squared)
        moving = bool(row.linear_speed_squared or row.angular_speed_squared)
        max_linear, max_angular = max(max_linear, linear), max(max_angular, angular)
        item = {"name": row.name.decode("utf-8", "replace") if row.name else None,
                "kind": ("rigid_static", "rigid_dynamic", "articulation")[row.kind],
                "sleeping": bool(row.sleeping) if row.kind else None,
                "wake_counter_seconds": float(row.wake_counter) if row.kind else None,
                "link_count": int(row.link_count),
                "linear_speed_m_s": linear if row.kind else None,
                "angular_speed_rad_s": angular if row.kind else None,
                "moving": moving if row.kind else None}
        actors.append(item)
        if not row.kind:
            counts["static_actors"] += 1
            continue
        if row.sleeping and (moving or row.wake_counter):
            raise RuntimeError("Native sleep/motion state is inconsistent; query only after completed fetch")
        prefix = "regular" if row.kind == 1 else "articulations"
        counts["regular_dynamic_actors" if row.kind == 1 else "articulations"] += 1
        counts[prefix + ("_sleeping" if row.sleeping else "_awake")] += 1
        if not row.sleeping:
            counts[prefix + ("_awake_moving" if moving else "_awake_zero_velocity")] += 1
        if row.kind == 2:
            counts["articulation_links"] += row.link_count
            counts["articulation_links_sleeping" if row.sleeping else "articulation_links_awake"] += row.link_count

    native_statistics = _ActivityStatistics()
    _check_activity_status(int(library.eersim_scene_activity_statistics(pointer, ctypes.byref(native_statistics))))
    if native_statistics.geometry_count != len(_GEOMETRY_TYPES):
        raise RuntimeError("Native activity diagnostics returned an incompatible geometry matrix")
    statistics = {name: int(getattr(native_statistics, name)) for name in _STATISTICS_SCALARS
                  if name not in {"cpu_worker_count", "geometry_count"}}
    statistics["shape_counts"] = dict(zip(_GEOMETRY_TYPES, map(int, native_statistics.shape_counts)))
    statistics["discrete_contact_pairs_by_geometry"] = {
        first: {second: int(native_statistics.discrete_contact_pairs[i * 11 + j])
                for j, second in enumerate(_GEOMETRY_TYPES)} for i, first in enumerate(_GEOMETRY_TYPES)}
    return {"counts": counts, "actors": actors, "max_linear_speed_m_s": max_linear,
            "max_angular_speed_rad_s": max_angular,
            "cpu_worker_count": int(native_statistics.cpu_worker_count),
            "simulation_statistics": statistics, "scene_gpu_flags": flags,
            "moving_definition": "any exactly nonzero native linear or angular velocity; no threshold",
            "statistics_scope": "SDK statistics for the last completed step; pair matrix entries must not be summed twice"}
