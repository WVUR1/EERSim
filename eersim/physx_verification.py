"""Verify the live native scene instead of treating requested CUDA as evidence.

Call after a normal-timestep bootstrap, while the scene and runtime are alive.
This module never steps the simulation or initializes a second CUDA context.
"""
from __future__ import annotations

import ctypes
from importlib.metadata import version
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
    from ovphysx import _bindings
    library = _load_library(Path(library_path).resolve(strict=True) if library_path is not None else probe_library_path())
    get_pointer = physx._lib.ovphysx_get_physx_ptr
    get_pointer.argtypes = [ctypes.c_uint64, _bindings.ovphysx_string_t, ctypes.c_int,
                           ctypes.POINTER(ctypes.c_void_p)]
    get_pointer.restype = _bindings.ovphysx_result_t
    pointer = ctypes.c_void_p()
    status = get_pointer(physx._omni_physx_sdk_handle.value, _bindings.ovphysx_string_t(scene_path),
                         1, ctypes.byref(pointer))  # OVPHYSX_PHYSX_TYPE_SCENE in 0.5.11.
    if status.status != 0 or not pointer.value:
        raise RuntimeError(f"Could not retrieve live PhysX scene {scene_path!r} for GPU verification")
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
