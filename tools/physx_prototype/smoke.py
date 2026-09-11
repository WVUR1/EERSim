"""Bounded headless ovphysx 0.5.11 CUDA check; run only on an authorized GPU.

Requires NumPy and scene_gpu_probe.so built from the matching PhysX headers.
This is a cube/contact smoke test, not an EERSim performance prediction.
"""
from __future__ import annotations

import argparse
import ctypes
from importlib.metadata import version
import json
import os
from pathlib import Path
import time


SCENE = '''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)
def Xform "World"
{
    def PhysicsScene "physicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.81
        bool physxScene:enableGPUDynamics = true
        uniform token physxScene:broadphaseType = "GPU"
        uniform token physxScene:solverType = "TGS"
    }
    def Cube "Floor" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        double size = 1
        double3 xformOp:translate = (0, 0, -0.1)
        double3 xformOp:scale = (4, 4, 0.2)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }
    def Xform "Cube" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    )
    {
        double3 xformOp:translate = (0, 0, 2)
        quatd xformOp:orient = (1, 0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
        float physics:mass = 1
        def Cube "Collider" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {
            double size = 0.2
        }
    }
}
'''


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--probe-library", type=Path, required=True)
    args = parser.parse_args()
    mask = os.environ.get("CUDA_VISIBLE_DEVICES", "").strip()
    if not mask or "," in mask or mask == "-1":
        raise RuntimeError("Set CUDA_VISIBLE_DEVICES to exactly one authorized physical GPU first")
    if os.environ.get("OVPHYSX_DISABLE_GPU"):
        raise RuntimeError("OVPHYSX_DISABLE_GPU is set; refusing a CPU-only probe")
    if version("ovphysx") != "0.5.11" or version("ovstage") != "0.1.1.355824":
        raise RuntimeError("This probe requires ovphysx 0.5.11 and ovstage 0.1.1.355824")

    work = args.work_dir.resolve()
    library_path = args.probe_library.resolve(strict=True)
    work.mkdir(parents=True, exist_ok=True)
    # Changes affect only this probe process. Keep generated assets/caches local.
    for key, directory in {
        "XDG_CACHE_HOME": "cache", "XDG_CONFIG_HOME": "config",
        "XDG_DATA_HOME": "data", "CUDA_CACHE_PATH": "cuda-cache",
        "TMPDIR": "tmp", "TMP": "tmp", "TEMP": "tmp",
    }.items():
        path = work / directory
        path.mkdir(exist_ok=True)
        os.environ[key] = str(path)
    os.chdir(work)
    usd_path = work / "cube.usda"
    usd_path.write_text(SCENE, encoding="utf-8")

    import numpy as np
    import ovphysx
    import ovstage
    from ovphysx import PhysX, PhysXConfig
    from ovphysx.types import TensorType
    from ovphysx import _bindings

    probe = ctypes.CDLL(str(library_path))
    for name, restype in (
        ("eersim_scene_gpu_flags", ctypes.c_uint),
        ("eersim_scene_cuda_device", ctypes.c_int),
        ("eersim_scene_cuda_name", ctypes.c_char_p),
    ):
        function = getattr(probe, name)
        function.argtypes = [ctypes.c_void_p]
        function.restype = restype

    # Public C interop exists in this release; the Python convenience wrapper
    # does not expose it. Reuse its exact ctypes ABI declarations and instance.
    ovphysx.set_log_level(ovphysx.LogLevel.INFO)
    setup_started = time.perf_counter()
    physx = PhysX(
        active_cuda_gpus="0",  # Logical ordinal after CUDA_VISIBLE_DEVICES masking.
        config=PhysXConfig(
            num_threads=2,
            cooked_collider_cache_dir=str(work / "cooked-cache"),
            carbonite_overrides={"/physics/suppressReadback": True},
        ),
    )
    stage = None
    attached = False
    binding = None
    result = None
    try:
        stage = ovstage.Stage("eersim-cuda-smoke")
        ovstage.population.open_usd(
            stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
        )
        stage.advance_write_floor(ordinal=1).wait()
        physx.attach_ovstage(stage, read_ordinal=1)
        attached = True
        physx.wait_all()
        # Complete native scene/buffer initialization before inspecting it.
        # Warmup alone is not GPU evidence; the live scene checks below are.
        # Explicit warmup_gpu() in 0.5.11 advances real physics by 1e-9 s.
        # Initialize at the normal fixed timestep before creating/readback bindings.
        physx.step_sync(1.0 / 120.0)
        scene_pointer = ctypes.c_void_p()
        get_scene = physx._lib.ovphysx_get_physx_ptr
        get_scene.argtypes = [
            ctypes.c_uint64, _bindings.ovphysx_string_t,
            ctypes.c_int, ctypes.POINTER(ctypes.c_void_p),
        ]
        get_scene.restype = _bindings.ovphysx_result_t
        status = get_scene(
            physx._omni_physx_sdk_handle.value,
            _bindings.ovphysx_string_t("/World/physicsScene"),
            1,  # OVPHYSX_PHYSX_TYPE_SCENE, exact 0.5.11 enum.
            ctypes.byref(scene_pointer),
        )
        if status.status != 0 or not scene_pointer.value:
            raise RuntimeError("Could not retrieve the live PhysX scene")
        flags = probe.eersim_scene_gpu_flags(scene_pointer)
        device = probe.eersim_scene_cuda_device(scene_pointer)
        device_name = probe.eersim_scene_cuda_name(scene_pointer).decode("utf-8", "replace")
        if flags != 15 or device != 0:
            raise RuntimeError(
                f"GPU verification failed: flags={flags}, CUDA ordinal={device}; "
                "require GPU dynamics, GPU broadphase, DirectGPU, valid CUDA context, ordinal 0"
            )
        binding = physx.create_tensor_binding(
            pattern="/World/Cube", tensor_type=TensorType.RIGID_BODY_POSE,
            raise_if_empty=True,
        )
        if tuple(binding.shape) != (1, 7):
            raise RuntimeError(f"Unexpected cube pose shape: {binding.shape}")
        poses = np.empty(binding.shape, dtype=np.float32)
        binding.read(poses)
        start_z = float(poses[0, 2])
        if not np.isfinite(poses).all() or not 1.99 <= start_z <= 2.01:
            raise RuntimeError(f"Invalid initial cube pose: {poses.tolist()}")
        setup_seconds = time.perf_counter() - setup_started
        dt, steps = 1.0 / 120.0, 360
        step_seconds = []
        heights = []
        loop_started = time.perf_counter()
        for tick in range(steps):
            started = time.perf_counter()
            physx.step_sync(dt)
            step_seconds.append(time.perf_counter() - started)
            binding.read(poses)
            if not np.isfinite(poses).all():
                raise RuntimeError(f"Non-finite pose after tick {tick + 1}")
            heights.append(float(poses[0, 2]))
            if time.perf_counter() - loop_started > 60:
                raise RuntimeError("Smoke exceeded its 60-second stepping budget")
        final_z = heights[-1]
        if not 1.60 < heights[29] < 1.80:
            raise RuntimeError(f"Cube did not fall under gravity: z(0.25s)={heights[29]}")
        if min(heights) < 0.07 or not 0.08 <= final_z <= 0.13:
            raise RuntimeError(f"Floor contact failed: min z={min(heights)}, final z={final_z}")
        if max(heights[-60:]) - min(heights[-60:]) > 0.01:
            raise RuntimeError("Cube did not settle during the final half-second")
        if probe.eersim_scene_gpu_flags(scene_pointer) != 15:
            raise RuntimeError("GPU scene flags changed during stepping")
        elapsed = sum(step_seconds)
        result = {
            "passed": True, "ovphysx": version("ovphysx"),
            "ovstage": version("ovstage"), "numpy": np.__version__,
            "cuda_visible_devices": mask, "logical_cuda_device": device,
            "cuda_device_name": device_name, "scene_gpu_flags": flags,
            "gpu_dynamics": True, "gpu_broadphase": True, "direct_gpu": True,
            "cuda_context_valid": True, "steps": steps, "dt_seconds": dt,
            "simulation_seconds": steps * dt, "setup_and_warmup_seconds": setup_seconds,
            "synchronous_step_seconds": elapsed,
            "step_rtf": steps * dt / elapsed,
            "loop_including_pose_read_seconds": time.perf_counter() - loop_started,
            "first_step_seconds": step_seconds[0],
            "median_step_seconds": float(np.median(step_seconds)),
            "start_z_m": start_z, "z_at_025s_m": heights[29],
            "min_z_m": min(heights), "final_z_m": final_z,
            "scope": "One cube and floor only; does not predict complete EERSim field throughput",
        }
    finally:
        try:
            if binding is not None:
                binding.destroy()
        finally:
            try:
                if attached:
                    physx.detach_ovstage()
            finally:
                try:
                    if stage is not None:
                        stage.destroy()
                finally:
                    physx.release()
    (work / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    main()
