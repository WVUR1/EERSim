"""Headless, bounded ovphysx 0.5.11 field benchmark; does not run PROS code."""
from __future__ import annotations

import argparse
import ctypes
import hashlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import sys
import time


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--work-dir", type=Path, required=True)
    parser.add_argument("--probe-library", type=Path, required=True)
    parser.add_argument("--cooked-geometry-library", type=Path,
                        help="Optional matching read-only cooked-convex inspection library")
    parser.add_argument("--manifest", type=Path, help="Defaults to the adjacent <scene>.manifest.json when present")
    parser.add_argument("--backend", choices=("gpu", "cpu"), default="gpu")
    parser.add_argument("--scene-path", default="/World/physicsScene")
    parser.add_argument("--actor-pattern", default="/World/rigid_*")
    parser.add_argument("--steps", type=int, default=240)
    parser.add_argument("--warmup-steps", type=int, default=2)
    parser.add_argument("--dt", type=float, default=1.0 / 240.0)
    parser.add_argument("--max-wall-seconds", type=float, default=60.0)
    parser.add_argument("--threads", type=int, default=2,
                        help="Verified CPU dispatcher worker count (0 runs tasks inline; default 2)")
    parser.add_argument("--dispatcher", choices=("carbonite", "native"), default="carbonite",
                        help="Carbonite task pool or dedicated PhysX CPU dispatcher")
    parser.add_argument("--position-iterations", type=int, default=100)
    parser.add_argument("--velocity-iterations", type=int, default=1)
    parser.add_argument("--min-z", type=float, default=-0.25)
    parser.add_argument("--max-abs-coordinate", type=float, default=10.0)
    parser.add_argument("--stats-interval", type=int, default=60)
    args = parser.parse_args()
    if min(args.steps, args.stats_interval) <= 0 or args.warmup_steps < 0:
        parser.error("steps and stats-interval must be positive; warmup-steps nonnegative")
    if not 0 <= args.threads <= 255:
        parser.error("threads must be 0..255; 0 runs dispatcher tasks inline")
    if any(not math.isfinite(x) or x <= 0 for x in (args.dt, args.max_wall_seconds, args.max_abs_coordinate)):
        parser.error("dt, max-wall-seconds and max-abs-coordinate must be finite and positive")
    if not math.isfinite(args.min_z):
        parser.error("min-z must be finite")
    if not 1 <= args.position_iterations <= 255 or not 0 <= args.velocity_iterations <= 255:
        parser.error("position-iterations must be1..255 and velocity-iterations0..255")
    return args


def guard_output_paths(work, backend, input_paths, include_cooked_geometry=False):
    """Reject scene/input aliases before creating any benchmark output."""
    names = [backend + ".usda", "result.json", "pose_rows.json", "initial_poses.npy",
             "final_poses.npy", "final_velocities.npy", "failure_poses.npy",
             "last_finite_poses.npy", "last_valid_poses.npy"]
    if include_cooked_geometry:
        names.append("cooked_geometry.json")
    inputs = [path.resolve() for path in input_paths if path is not None]
    for name in names:
        output = (work / name).resolve()
        for source in inputs:
            same_file = output == source
            if not same_file and output.exists() and source.exists():
                same_file = output.samefile(source)
            if same_file:
                raise ValueError(f"Benchmark output would overwrite input: {source}; choose a different --work-dir")


def scene_override(scene, output, scene_path, backend, position_iterations=100,
                   velocity_iterations=1, actor_paths=()):
    """Layer over the original file, retaining the original asset-reference base."""
    source = scene.as_posix()
    if any(char in source for char in "@\r\n"):
        raise ValueError("Scene path contains unsupported USDA asset delimiters")
    root = {"attributes": [], "children": {}}

    def node(prim_path):
        components = prim_path.strip("/").split("/")
        if not components or any(not part.isidentifier() for part in components):
            raise ValueError("Prim paths must contain simple USD prim identifiers")
        current = root
        for component in components:
            current = current["children"].setdefault(component, {"attributes": [], "children": {}})
        return current

    node(scene_path)["attributes"] = [
        f'bool physxScene:enableGPUDynamics = {"true" if backend == "gpu" else "false"}',
        f'uniform token physxScene:broadphaseType = "{"GPU" if backend == "gpu" else "MBP"}"',
        'uniform token physxScene:solverType = "TGS"',
        f'uniform uint physxScene:minPositionIterationCount = {position_iterations}',
        f'uniform uint physxScene:maxPositionIterationCount = {position_iterations}',
        f'uniform uint physxScene:minVelocityIterationCount = {velocity_iterations}',
        f'uniform uint physxScene:maxVelocityIterationCount = {velocity_iterations}',
    ]
    for actor_path in actor_paths:
        node(actor_path)["attributes"].extend([
            f'int physxRigidBody:solverPositionIterationCount = {position_iterations}',
            f'int physxRigidBody:solverVelocityIterationCount = {velocity_iterations}',
        ])
    lines = ['#usda 1.0', '(', f'    subLayers = [@{source}@]', ')']

    def emit(current, depth=0):
        for name, child in current["children"].items():
            indent = '    ' * depth
            lines.extend([indent + f'over "{name}"', indent + '{'])
            lines.extend(indent + '    ' + attribute for attribute in child["attributes"])
            emit(child, depth + 1)
            lines.append(indent + '}')

    emit(root)
    output.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main():
    args = parse_args()
    scene = args.scene.resolve(strict=True)
    library_path = args.probe_library.resolve(strict=True)
    cooked_library_path = args.cooked_geometry_library.resolve(strict=True) if args.cooked_geometry_library else None
    manifest_path = args.manifest.resolve(strict=True) if args.manifest else scene.with_suffix(".manifest.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else None
    iteration_experiment = (args.position_iterations, args.velocity_iterations) != (100, 1)
    if iteration_experiment and not manifest:
        raise RuntimeError("Nondefault iteration experiments require a matching exporter manifest")
    work = args.work_dir.resolve()
    guard_output_paths(work, args.backend,
                       (scene, library_path, cooked_library_path, manifest_path if manifest_path.exists() else None),
                       include_cooked_geometry=cooked_library_path is not None)
    mask = os.environ.get("CUDA_VISIBLE_DEVICES", "").strip()
    if args.backend == "gpu":
        if not mask or "," in mask or mask == "-1":
            raise RuntimeError("GPU mode requires CUDA_VISIBLE_DEVICES set to one authorized physical GPU")
        if os.environ.get("OVPHYSX_DISABLE_GPU"):
            raise RuntimeError("OVPHYSX_DISABLE_GPU is set")
    if version("ovphysx") != "0.5.11" or version("ovstage") != "0.1.1.355824":
        raise RuntimeError("Requires ovphysx==0.5.11 and ovstage==0.1.1.355824")
    work.mkdir(parents=True, exist_ok=True)
    for key, directory in {
        "XDG_CACHE_HOME": "cache", "XDG_CONFIG_HOME": "config", "XDG_DATA_HOME": "data",
        "CUDA_CACHE_PATH": "cuda-cache", "TMPDIR": "tmp", "TMP": "tmp", "TEMP": "tmp",
    }.items():
        path = work / directory
        path.mkdir(exist_ok=True)
        os.environ[key] = str(path)
    os.chdir(work)
    override = work / (args.backend + ".usda")
    actor_paths = [actor["path"] for actor in manifest["actors"] if actor["mass_kg"] > 0] if manifest else []
    scene_override(scene, override, args.scene_path, args.backend,
                   args.position_iterations, args.velocity_iterations, actor_paths)

    import numpy as np
    import ovphysx
    import ovstage
    from ovphysx import PhysX, PhysXConfig, _bindings
    from ovphysx.types import TensorType

    probe = ctypes.CDLL(str(library_path))
    for name in ("eersim_scene_inventory", "eersim_scene_statistics"):
        function = getattr(probe, name)
        function.argtypes = [ctypes.c_void_p]
        function.restype = ctypes.c_char_p

    def inspect(name, pointer):
        payload = json.loads(getattr(probe, name)(pointer).decode("utf-8"))
        if "error" in payload:
            raise RuntimeError(payload["error"])
        return payload

    result = {
        "passed": False, "backend": args.backend, "scene": str(scene),
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "ovphysx": version("ovphysx"), "ovstage": version("ovstage"), "numpy": np.__version__,
        "cuda_visible_devices": mask, "cpu_threads": args.threads,
        "cpu_threads_requested": args.threads, "cpu_dispatcher_requested": args.dispatcher,
        "cpu_thread_setting_path": "/persistent/physics/numThreads",
        "requested_steps": args.steps, "dt_seconds": args.dt,
        "warmup_steps": args.warmup_steps, "solver": "TGS",
        "initialization_mode": "one_normal_fixed_step_before_first_tensor_read",
        "initialization_steps": 1,
        "position_iterations": args.position_iterations, "velocity_iterations": args.velocity_iterations,
        "iteration_experiment": iteration_experiment,
        "reference_iteration_budget": {"position_iterations": 100, "velocity_iterations": 1},
        "min_allowed_z_m": args.min_z, "max_allowed_abs_position_m": args.max_abs_coordinate,
        "notes": [
            "Experimental converted field; no PROS motor control or sensor loop.",
            "PhysX TGS position iterations are not equivalent to Bullet solver iterations.",
            "GPU scene flags do not prove every contact executes on GPU; inspect convex eligibility and native warnings.",
            "Triangle GPU cooking is not verified by this public-header probe.",
            "Some PxSimulationStatistics counters can be unavailable/zero on a backend; zero is not proof of no contacts.",
        ],
    }
    if manifest:
        if manifest.get("scene_sha256") != result["scene_sha256"]:
            raise RuntimeError("Exporter manifest does not match the scene SHA256")
        result["export_manifest_sha256"] = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
        result["exported_counts"] = manifest["counts"]
        result["conversion_differences"] = manifest.get("differences", [])
        result["convex_policy"] = manifest.get("convex_policy")
    if iteration_experiment:
        result["notes"].append("Nondefault counts are an explicit convergence experiment; no accuracy equivalence is established.")
    ovphysx.set_log_level(ovphysx.LogLevel.INFO)
    if args.backend == "cpu":
        PhysX.set_cpu_mode(True)  # Separate process; this setting cannot be reversed.
    physx = stage = binding = velocity_binding = None
    poses = last_finite_poses = last_valid_poses = None
    pose_rows = []
    phase, phase_tick = "setup", 0
    initialization_completed = warmup_completed = measured_completed = 0
    attached = False
    error = None
    setup_started = time.perf_counter()
    try:
        physx = PhysX(
            active_cuda_gpus="0" if args.backend == "gpu" else None,
            config=PhysXConfig(
                num_threads=args.threads, cooked_collider_cache_dir=str(work / "cooked-cache"),
                carbonite_overrides={
                    "/physics/suppressReadback": args.backend == "gpu",
                    # In 0.5.11 the typed field writes /physics/numThreads, but
                    # native Setup reads the persistent path. Keep both aligned.
                    "/persistent/physics/numThreads": args.threads,
                    "/physics/physxDispatcher": args.dispatcher == "native",
                },
            ),
        )
        stage = ovstage.Stage("eersim-field-benchmark")
        ovstage.population.open_usd(stage, str(override), ordinal=1, domains=ovstage.PopulationDomain.PHYSICS)
        stage.advance_write_floor(ordinal=1).wait()
        physx.attach_ovstage(stage, read_ordinal=1)
        attached = True
        physx.wait_all()
        # ovphysx 0.5.11 warmup_gpu() runs a real 1e-9 s physics step. Contact-rich
        # scenes must initialize with the normal timestep instead. step_sync()
        # marks GPU buffers ready, so later tensor reads skip implicit warmup.
        # Do this on CPU too: both comparisons advance the same number of ticks.
        phase, phase_tick = "fixed_step_initialization", 1
        initialization_started = time.perf_counter()
        physx.step_sync(args.dt)
        initialization_completed = 1
        result["initialization_step_seconds"] = time.perf_counter() - initialization_started
        result["initialization_simulation_seconds"] = args.dt
        get_scene = physx._lib.ovphysx_get_physx_ptr
        get_scene.argtypes = [ctypes.c_uint64, _bindings.ovphysx_string_t, ctypes.c_int, ctypes.POINTER(ctypes.c_void_p)]
        get_scene.restype = _bindings.ovphysx_result_t
        pointer = ctypes.c_void_p()
        status = get_scene(physx._omni_physx_sdk_handle.value, _bindings.ovphysx_string_t(args.scene_path), 1, ctypes.byref(pointer))
        if status.status != 0 or not pointer.value:
            raise RuntimeError("Could not retrieve the live PhysX scene")
        inventory = inspect("eersim_scene_inventory", pointer)
        result["initial_inventory"] = inventory
        actual_workers = inventory.get("cpu_dispatcher_workers")
        result["cpu_dispatcher_workers"] = actual_workers
        if actual_workers is None:
            raise RuntimeError("Inspector does not report CPU workers; rebuild scene_inspect.cpp with the dispatcher getter")
        if actual_workers != args.threads:
            raise RuntimeError(
                f"Live CPU dispatcher workers={actual_workers}, requested={args.threads}; "
                "Carbonite may cap workers to its task pool; use --dispatcher native or request fewer workers"
            )
        if cooked_library_path is not None:
            cooked_library = ctypes.CDLL(str(cooked_library_path))
            dump_cooked = cooked_library.eersim_cooked_convex_geometry
            dump_cooked.argtypes = [ctypes.c_void_p]
            dump_cooked.restype = ctypes.c_char_p
            cooked_data = json.loads(dump_cooked(pointer).decode("utf-8"))
            if "error" in cooked_data:
                raise RuntimeError(f"Cooked geometry inspection failed: {cooked_data['error']}")
            (work / "cooked_geometry.json").write_text(json.dumps(cooked_data) + "\n", encoding="utf-8")
            result["cooked_geometry_file"] = "cooked_geometry.json"
            result["cooked_geometry_mesh_count"] = len(cooked_data.get("meshes", []))
            result["cooked_geometry_instance_count"] = len(cooked_data.get("instances", []))
        if manifest:
            counts = manifest["counts"]
            for native_key, exported_key in (("shapes", "collision_shapes"), ("constraints", "joints")):
                if inventory[native_key] != counts.get(exported_key, 0):
                    raise RuntimeError(f"Live {native_key}={inventory[native_key]} != exported {counts.get(exported_key, 0)}")
            # USD cylinders can cook into additional convex or custom shapes.
            if inventory["convex_shapes"] < counts.get("convex_meshes", 0):
                raise RuntimeError("Fewer live convex shapes than authored convex meshes")
            if inventory["triangle_mesh_shapes_gpu_cooking_unverified"] < counts.get("triangle_meshes", 0):
                raise RuntimeError("Fewer live triangle meshes than authored triangle meshes")
            if inventory["dynamic_actors"] + inventory["articulation_links"] != counts["dynamic_actors"]:
                raise RuntimeError("Live dynamic actor count differs from exporter manifest")
            expected_mass = sum(actor["mass_kg"] for actor in manifest["actors"] if actor["mass_kg"] > 0)
            if not math.isclose(inventory["dynamic_mass_kg"], expected_mass, rel_tol=1e-5, abs_tol=1e-5):
                raise RuntimeError("Live total dynamic mass differs from exporter manifest")
        required_flags = 15 if args.backend == "gpu" else 0
        if inventory["gpu_flags"] != required_flags:
            raise RuntimeError(f"Backend flags {inventory['gpu_flags']} != required {required_flags}")
        if args.backend == "gpu" and inventory["cuda_device"] != 0:
            raise RuntimeError("PhysX did not use logical CUDA device0")
        for key, required in (("position_iterations_min", args.position_iterations),
                              ("position_iterations_max", args.position_iterations),
                              ("velocity_iterations_min", args.velocity_iterations),
                              ("velocity_iterations_max", args.velocity_iterations)):
            if inventory[key] != required:
                raise RuntimeError(f"Live {key}={inventory[key]} instead of {required}")
        binding = physx.create_tensor_binding(pattern=args.actor_pattern, tensor_type=TensorType.RIGID_BODY_POSE, raise_if_empty=True)
        expected = inventory["dynamic_actors"] + inventory["articulation_links"]
        if tuple(binding.shape) != (expected, 7):
            raise RuntimeError(f"Pose binding {binding.shape} does not cover all {expected} live dynamic/link actors")
        poses = np.full(binding.shape, np.nan, dtype=np.float32)
        manifest_actors = {actor["path"]: actor for actor in manifest["actors"]} if manifest else {}
        for row, prim_path in enumerate(binding.prim_paths):
            actor = manifest_actors.get(prim_path, {})
            pose_rows.append({"row": row, "path": prim_path, "label": actor.get("label"),
                              "body_id": actor.get("body_id"), "link_index": actor.get("link_index"),
                              "mass_kg": actor.get("mass_kg")})
        if len(pose_rows) != expected:
            raise RuntimeError("Binding path metadata does not match its pose rows")
        (work / "pose_rows.json").write_text(json.dumps(pose_rows, indent=2) + "\n", encoding="utf-8")
        result["pose_rows_file"] = "pose_rows.json"

        def read_poses():
            nonlocal last_finite_poses, last_valid_poses
            binding.read(poses)
            if not np.isfinite(poses).all():
                raise RuntimeError("Non-finite rigid-body pose")
            last_finite_poses = poses.copy()
            positions = poses[:, :3]
            minimum = float(positions[:, 2].min())
            maximum = float(np.abs(positions).max())
            if minimum < args.min_z or maximum > args.max_abs_coordinate:
                raise RuntimeError(f"Actor escaped field bounds: min z={minimum}, max abs coordinate={maximum}")
            norms = np.linalg.norm(poses[:, 3:], axis=1)
            if np.any(np.abs(norms - 1) > 0.01):
                raise RuntimeError("Non-unit rigid-body orientation")
            last_valid_poses = poses.copy()
            return minimum, maximum

        read_poses()
        initial_poses = poses.copy()
        np.save(work / "initial_poses.npy", initial_poses, allow_pickle=False)
        result["initial_pose_sample_simulation_seconds"] = args.dt
        result["pose_count"] = expected
        result["setup_seconds"] = time.perf_counter() - setup_started
        warmup_started = time.perf_counter()
        phase = "warmup"
        for warmup in range(args.warmup_steps):
            phase_tick = warmup + 1
            physx.step_sync(args.dt)
            warmup_completed += 1
            read_poses()
        result["warmup_seconds"] = time.perf_counter() - warmup_started
        step_times, read_times, snapshots = [], [], []
        min_z, max_abs = read_poses()
        loop_started = time.perf_counter()
        phase = "measurement"
        for tick in range(args.steps):
            phase_tick = tick + 1
            started = time.perf_counter()
            physx.step_sync(args.dt)
            measured_completed += 1
            step_times.append(time.perf_counter() - started)
            started = time.perf_counter()
            z, coordinate = read_poses()
            read_times.append(time.perf_counter() - started)
            min_z, max_abs = min(min_z, z), max(max_abs, coordinate)
            if tick == 0 or (tick + 1) % args.stats_interval == 0 or tick == args.steps - 1:
                snapshots.append({"measured_tick": tick + 1, **inspect("eersim_scene_statistics", pointer)})
            if time.perf_counter() - loop_started >= args.max_wall_seconds:
                break
        loop_seconds = time.perf_counter() - loop_started
        np.save(work / "final_poses.npy", poses, allow_pickle=False)
        velocity_binding = physx.create_tensor_binding(
            pattern=args.actor_pattern, tensor_type=TensorType.RIGID_BODY_VELOCITY, raise_if_empty=True
        )
        if tuple(velocity_binding.shape) != (expected, 6) or velocity_binding.prim_paths != [row["path"] for row in pose_rows]:
            raise RuntimeError("Velocity binding does not match pose rows")
        velocities = np.full(velocity_binding.shape, np.nan, dtype=np.float32)
        velocity_binding.read(velocities)
        if not np.isfinite(velocities).all():
            raise RuntimeError("Non-finite final body velocity")
        np.save(work / "final_velocities.npy", velocities, allow_pickle=False)
        result["max_final_linear_speed_m_s"] = float(np.linalg.norm(velocities[:, :3], axis=1).max())
        result["rms_final_linear_speed_m_s"] = float(np.sqrt(np.mean(np.sum(velocities[:, :3] ** 2, axis=1))))
        result["max_final_angular_speed_rad_s"] = float(np.linalg.norm(velocities[:, 3:], axis=1).max())
        count = len(step_times)
        step_seconds = sum(step_times)
        result.update({
            "passed": True, "completed_requested_steps": count == args.steps,
            "measured_steps": count, "simulation_seconds": count * args.dt,
            "synchronous_step_seconds": step_seconds, "pose_read_and_validation_seconds": sum(read_times),
            "loop_seconds": loop_seconds, "step_rtf": count * args.dt / step_seconds,
            "loop_rtf": count * args.dt / loop_seconds,
            "wall_budget_reached": loop_seconds >= args.max_wall_seconds,
            "first_step_seconds": step_times[0], "median_step_seconds": float(np.median(step_times)),
            "p95_step_seconds": float(np.percentile(step_times, 95)),
            "min_observed_z_m": min_z, "max_observed_abs_coordinate_m": max_abs,
            "maximum_body_displacement_m": float(np.linalg.norm(poses[:, :3] - initial_poses[:, :3], axis=1).max()),
            "statistics_samples": snapshots,
            "final_inventory": inspect("eersim_scene_inventory", pointer),
        })
        if result["final_inventory"]["gpu_flags"] != required_flags:
            raise RuntimeError("Backend flags changed during benchmark")
        if result["final_inventory"].get("cpu_dispatcher_workers") != args.threads:
            raise RuntimeError("CPU dispatcher worker count changed during benchmark")
    except Exception as exc:
        error = exc
        result["passed"] = False
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["failure_phase"] = phase
        result["failure_phase_tick"] = phase_tick
        result["initialization_steps_completed"] = initialization_completed
        result["warmup_steps_completed"] = warmup_completed
        result["measured_steps_completed"] = measured_completed
        result["simulation_seconds_before_failure"] = (initialization_completed + warmup_completed + measured_completed) * args.dt
        if poses is not None:
            np.save(work / "failure_poses.npy", poses, allow_pickle=False)
            offenders = []
            for index, pose in enumerate(poses):
                finite = bool(np.isfinite(pose).all())
                low = finite and float(pose[2]) < args.min_z
                far = finite and float(np.abs(pose[:3]).max()) > args.max_abs_coordinate
                bad_orientation = finite and abs(float(np.linalg.norm(pose[3:])) - 1) > 0.01
                if not finite or low or far or bad_orientation:
                    offenders.append({
                        **(pose_rows[index] if index < len(pose_rows) else {"row": index}),
                        "pose": [float(value) if np.isfinite(value) else None for value in pose],
                        "non_finite": not finite, "below_floor_bound": low,
                        "outside_coordinate_bound": far, "non_unit_orientation": bad_orientation,
                    })
            result["failing_pose_rows"] = offenders
        if last_finite_poses is not None:
            np.save(work / "last_finite_poses.npy", last_finite_poses, allow_pickle=False)
        if last_valid_poses is not None:
            np.save(work / "last_valid_poses.npy", last_valid_poses, allow_pickle=False)
        if physx is not None and "pointer" in locals() and pointer.value:
            try:
                result["failure_statistics"] = inspect("eersim_scene_statistics", pointer)
            except Exception as diagnostic_error:
                result["failure_statistics_error"] = str(diagnostic_error)
    finally:
        try:
            try:
                if velocity_binding is not None:
                    velocity_binding.destroy()
            finally:
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
                    if physx is not None:
                        physx.release()
    (work / "result.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)
    if error:
        raise error


if __name__ == "__main__":
    main()
