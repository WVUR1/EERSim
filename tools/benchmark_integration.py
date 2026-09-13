"""Headless PROS/physics validation; rendering and startup are timed separately.

From a source checkout (or an environment with EERSim installed)::

    python tools/benchmark_integration.py --backend physx_gpu --physics-hz 120 --output results/claw.json
    python tools/benchmark_integration.py --backend bullet --fixture practice \
        --no-pros --steps 240 --output results/practice.json

The default is the unchanged bundled claw program for 30 competition seconds.
Short native runs validate startup and finite state only. ``--no-pros`` measures
passive physics and never claims autonomous completion. No window is opened.
"""
from __future__ import annotations

import argparse
import csv
from contextlib import contextmanager
import hashlib
import json
import math
import os
from numbers import Real
from pathlib import Path
import sys
import time
from typing import Any

# A checkout needs no editable install; installed tools use the installed package.
_CHECKOUT = Path(__file__).resolve().parents[1]
if (_CHECKOUT / "eersim" / "__init__.py").is_file():
    sys.path.insert(0, str(_CHECKOUT))


class ValidationError(RuntimeError):
    pass


def json_value(value: Any, path: str = "state") -> Any:
    """Convert NumPy values and reject every nonfinite numerical leaf."""
    if isinstance(value, dict):
        return {str(key): json_value(item, f"{path}.{key}") for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(item, f"{path}[{index}]") for index, item in enumerate(value)]
    if isinstance(value, Real):
        if not math.isfinite(value):
            raise ValidationError(f"Nonfinite value at {path}: {value}")
        return value if isinstance(value, (int, float)) else value.item()
    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "tolist"):
        return json_value(value.tolist(), path)
    raise ValidationError(f"Unsupported report value at {path}: {type(value).__name__}")


def source_hashes(workspace: Path) -> dict[str, str]:
    """Hash the complete source/include trees, excluding build output."""
    files = sorted(path for name in ("src", "include")
                   for path in (workspace / name).rglob("*") if path.is_file()
                   and not any(part in {".eersim", ".git", "__pycache__"}
                               for part in path.relative_to(workspace).parts))
    if not files or not (workspace / "src").is_dir():
        raise ValidationError(f"PROS workspace has no source tree: {workspace}")
    return {path.relative_to(workspace).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files}


def field_states(world) -> list[dict]:
    """Read the authoritative API, never the Bullet geometry shadow."""
    result = []
    for item in world.field_objects:
        body = item["id"]
        position, orientation = world._physics.getBasePositionAndOrientation(body, physicsClientId=world.client)
        linear, angular = world._physics.getBaseVelocity(body, physicsClientId=world.client)
        result.append({"name": item["name"], "id": body, "position": position,
                       "orientation": orientation, "velocity": linear, "angular_velocity": angular})
    return json_value(result, "field_objects")


def validate_claw(evidence: dict, logs: list[str], final: dict) -> None:
    if evidence.get("lift_m", 0) <= .15:
        raise ValidationError("The claw did not physically lift demo_preload by more than 0.15 m")
    placements = evidence.get("final_scoring", {}).get("placements", [])
    if not any(row.get("pin") == "demo_preload" and row.get("alliance") == "red"
               and row.get("goal") == "goal_red_5" for row in placements):
        raise ValidationError("demo_preload is not retained in the intended near red goal (goal_red_5)")
    if evidence.get("final_robot_contact", True):
        raise ValidationError("The robot still contacts demo_preload after placement")
    if not any("Placement complete" in line for line in logs):
        raise ValidationError("The PROS program did not report placement and retreat completion")
    if any("Route timeout" in line for line in logs):
        raise ValidationError("The PROS program reported a route timeout")
    if final.get("running") or final.get("phase") != "disabled":
        raise ValidationError("The robot was not disabled at the mode duration")


def validate_backend(status: dict, expected: str, iterations: int) -> None:
    # Runtime status separates the engine name from the active device type.
    actual = status.get("backend")
    accelerated = bool(status.get("hardware_accelerated"))
    if actual == "physx":
        actual = "physx_gpu" if accelerated else "physx_cpu"
    if actual != expected or accelerated != (expected == "physx_gpu"):
        raise ValidationError(f"Requested {expected}, got runtime status {status}")
    if status.get("solver_iterations") != iterations:
        raise ValidationError(f"Requested {iterations} solver iterations, got {status.get('solver_iterations')}")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    result.add_argument("--backend", choices=("bullet", "physx_cpu", "physx_gpu"), default="physx_gpu")
    result.add_argument("--device", type=int, default=0, help="CUDA device ordinal visible to this process")
    result.add_argument("--iterations", type=int, default=100)
    result.add_argument("--physics-hz", type=int, choices=(120, 240), default=240,
                        help="Fixed physics rate; competition duration is independent of the selected rate")
    result.add_argument("--fixture", choices=("claw", "practice"), default="claw")
    length = result.add_mutually_exclusive_group()
    length.add_argument("--seconds", type=float, default=None, help="Competition seconds; default 30")
    length.add_argument("--steps", type=int, help="Exact competition ticks, or passive ticks with --no-pros")
    result.add_argument("--no-pros", action="store_true", help="Passive physics only; no compilation or Session.start")
    result.add_argument("--workspace", type=Path, help="Optional compatible external PROS workspace")
    result.add_argument("--validation", choices=("auto", "smoke", "claw"), default="auto",
                        help="Auto requires full claw success for native claw runs lasting at least 30 s")
    result.add_argument("--max-wall-seconds", type=float, default=3600,
                        help="Stepping wall budget, checked between fixed ticks")
    result.add_argument("--progress-seconds", type=float, default=1.0)
    result.add_argument("--diagnostics-seconds", type=float, default=0.0,
                        help="Opt-in native PhysX activity and cumulative profiling every N simulated seconds; 0 disables")
    result.add_argument("--output", type=Path, required=True, help="JSON report; adjacent CSV and log are also written")
    return result


def execute(args, *, config: dict | None = None, world_factory=None, session_factory=None, builder=None) -> dict:
    """Run once, always retaining failure evidence and closing owned runtime resources."""
    for name in ("iterations", "max_wall_seconds", "progress_seconds"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name.replace('_', '-')} must be positive and finite")
    if args.device < 0:
        raise ValueError("device must be nonnegative")
    if args.steps is not None and args.steps <= 0:
        raise ValueError("steps must be positive")
    seconds = 30.0 if args.seconds is None else args.seconds
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("seconds must be positive and finite")
    hz = getattr(args, "physics_hz", 240)
    if hz not in (120, 240):
        raise ValueError("physics-hz must be 120 or 240")
    ticks = args.steps if args.steps is not None else math.ceil(seconds * hz - 1e-9)
    seconds = ticks / hz
    diagnostic_seconds = getattr(args, "diagnostics_seconds", 0.0)
    if not math.isfinite(diagnostic_seconds) or diagnostic_seconds < 0:
        raise ValueError("diagnostics-seconds must be finite and nonnegative")
    diagnostic_ticks = max(1, math.ceil(diagnostic_seconds * hz - 1e-9)) if diagnostic_seconds else 0
    if diagnostic_ticks and (args.backend not in ('physx_cpu', 'physx_gpu') or
                             math.ceil(ticks / diagnostic_ticks) + 3 > 4096):
        raise ValueError("Native diagnostics require PhysX and at most 4096 samples; increase diagnostics-seconds")
    scope = "physics_only" if args.no_pros else "native_smoke"
    full = args.validation == "claw" or (args.validation == "auto" and not args.no_pros
                                         and args.fixture == "claw" and seconds >= 30)
    if full:
        if args.no_pros or args.fixture != "claw" or seconds < 30:
            raise ValueError("Claw validation requires native claw fixture and at least 30 competition seconds")
        scope = "native_claw_placement"
    output = args.output.expanduser().resolve()
    if output.suffix.lower() != ".json":
        raise ValueError("output must end in .json")
    output.parent.mkdir(parents=True, exist_ok=True)
    report = {"schema_version": 1, "status": "failed", "validation_scope": scope,
              "request": {"backend": args.backend, "device": args.device, "iterations": args.iterations,
                          "fixture": args.fixture, "physics_hz": hz, "ticks": ticks, "seconds": seconds,
                          "pros": not args.no_pros},
              "timing_seconds": {}, "progress": [], "checks": {}, "notes": [
                  "No rendering; startup and observation are excluded from step-only realtime factor.",
                  "Initialization and competition use the same fixed dt; initialization is excluded from mode time.",
                  "Performance snapshots retain existing lifetime counters, including setup; enable them with EERSIM_PROFILE=1.",
                  "Placement is prototype physical contact detection, not official game scoring.",
                  "Matching fixed dt and solver iteration count does not establish cross-engine trajectory equivalence."]}
    timings = report["timing_seconds"]
    logs: list[str] = []
    session = world = None
    workspace = None
    original_hashes = None
    csv_stream = log_stream = None
    wall_start = time.perf_counter()
    loop_start = None
    initial = final = None
    step_count = mode_count = init_count = 0
    diagnostic_wall_before_loop = 0.0
    previous_profile = os.environ.get('EERSIM_PROFILE')
    if diagnostic_ticks:
        os.environ['EERSIM_PROFILE'] = '1'
        report['diagnostic_sampling'] = {
            'requested_seconds': diagnostic_seconds, 'effective_seconds': diagnostic_ticks / hz,
            'scope': 'Prepared/setup, initialization completion, periodic mode ticks and final state; no counter resets.',
            'timing_scope': 'Queries are outside timed steps; loop time includes queries and separately reports their cost.'}
        report['diagnostic_samples'] = []

    @contextmanager
    def timed(stage):
        began = time.perf_counter()
        try:
            yield
        finally:
            timings[stage] = timings.get(stage, 0.0) + time.perf_counter() - began

    def log(message):
        text = str(message)
        logs.append(text)
        if log_stream:
            log_stream.write(text + "\n")
            log_stream.flush()
        print(text, flush=True)

    def sample_activity(label, state, mode_ticks):
        if not diagnostic_ticks:
            return
        worker = getattr(world, '_process', None)
        if worker is None or getattr(worker, 'failed', False):
            raise ValidationError('Requested native diagnostics need a live PhysX worker')
        began = time.perf_counter()
        with timed('diagnostics'):
            response = worker.call('diagnostics', {'activity': True})
            if 'scene_activity' not in response:
                raise ValidationError('Worker did not return requested native scene activity')
            performance = {'worker': response['performance']}
            for name, owner in (('ipc', worker), ('session', session), ('world', world)):
                if hasattr(owner, 'performance'):
                    performance[name] = owner.performance.snapshot()
            sample = json_value({'label': label, 'tick': state['tick'], 'world_time': state['time'],
                                 'mode_time': mode_ticks / hz, 'performance': performance,
                                 'scene_activity': response['scene_activity']})
        sample['diagnostic_wall_seconds'] = time.perf_counter() - began
        report['diagnostic_samples'].append(sample)

    def finalize_loop_metrics():
        """Retain completed work even when a requested diagnostic query fails."""
        if 'loop_wall_seconds' in report:
            return
        report['loop_wall_seconds'] = 0.0 if loop_start is None else time.perf_counter() - loop_start
        report['steps'] = {'initialization': init_count, 'competition': mode_count if session else 0,
                           'passive': mode_count if session is None else 0, 'total': step_count}
        report['simulation_seconds'] = {'initialization': init_count / hz,
                                        'competition': mode_count / hz if session else 0,
                                        'passive': mode_count / hz if session is None else 0,
                                        'total_advanced': step_count / hz,
                                        'prepared_world_time': initial['time'] if initial is not None else None}
        measured = timings.get('competition_steps' if session else 'passive_steps', 0)
        report['step_realtime_factor'] = (mode_count / hz) / measured if measured > 0 else None
        loop_wall = report['loop_wall_seconds']
        report['loop_realtime_factor'] = (step_count / hz) / loop_wall if loop_wall > 0 else None
        if diagnostic_ticks:
            diagnostic_wall = 0.0 if loop_start is None else timings.get('diagnostics', 0.0) - diagnostic_wall_before_loop
            report['loop_diagnostics_wall_seconds'] = diagnostic_wall
            adjusted = max(0.0, loop_wall - diagnostic_wall)
            report['loop_wall_seconds_excluding_diagnostics'] = adjusted
            report['loop_realtime_factor_excluding_diagnostics'] = (step_count / hz) / adjusted if adjusted > 0 else None
        if final is not None:
            report.setdefault('final_state', final)

    try:
        log_stream = output.with_suffix(".log").open("w", encoding="utf-8")
        csv_stream = output.with_suffix(".csv").open("w", newline="", encoding="utf-8")
        columns = ("time", "tick", "mode_time", "phase", "step_wall_seconds", "x_m", "y_m", "z_m",
                   "yaw_rad", "preload_z_m", "joints_json", "motors_json", "scoring_json")
        writer = csv.DictWriter(csv_stream, fieldnames=columns)
        writer.writeheader()
        with timed("configuration"):
            if config is None:
                from eersim.config import ASSETS
                from eersim.demos import claw_demo_config
                config = claw_demo_config()
                if args.fixture == "practice":
                    config.update(game_pack=str(ASSETS / "games" / "practice.json"),
                                  field_objects=[], start_pose=[0, 0, .085, 0],
                                  notes="Claw robot on the practice field; no preload or competition goals.")
            else:
                from copy import deepcopy
                config = deepcopy(config)
            config.update(physics_backend=args.backend, physics_device=args.device,
                          physics_solver_iterations=args.iterations, physics_hz=hz, renderer="cpu", noise={})
            config.setdefault("duration_overrides", {})["skills_auto"] = seconds
            if args.workspace:
                config["workspace"] = str(args.workspace.expanduser().resolve())
            report["config"] = json_value(config)
        if not args.no_pros:
            workspace = Path(config["workspace"]).expanduser().resolve()
            original_hashes = source_hashes(workspace)
            report["source_before_sha256"] = original_hashes
            if builder is None:
                from eersim.bridge import build_workspace
                builder = build_workspace
            with timed("native_build"):
                executable = builder(workspace, log)
            if session_factory is None:
                from eersim.runner import Session
                session_factory = Session
            with timed("session_construction"):
                session = session_factory(config, log)
                world = session.world
            with timed("session_start"):
                session.start("skills_auto", executable)
        else:
            if world_factory is None:
                from eersim.physics_backend import create_world
                world_factory = create_world
            with timed("world_construction"):
                world = world_factory(config)

        with timed("observation"):
            initial = json_value(world.snapshot())
            report["initial_state"] = initial
            report["initial_field_objects"] = field_states(world)
            report["physics"] = json_value(world.physics_status)
            validate_backend(world.physics_status, args.backend, args.iterations)
            preload_items = [item for item in world.field_objects if item["name"] == "demo_preload"]
            if len(preload_items) > 1 or (full and len(preload_items) != 1):
                raise ValidationError("Full claw validation needs exactly one demo_preload")
            preload = preload_items[0]["id"] if preload_items else None
            initial_height = None if preload is None else next(
                item["position"][2] for item in report["initial_field_objects"] if item["id"] == preload)
            highest = initial_height

        step_count = mode_count = init_count = 0
        sample_activity('prepared', initial, 0)
        diagnostic_wall_before_loop = timings.get('diagnostics', 0.0)
        next_diagnostic = diagnostic_ticks
        next_progress = 0.0
        loop_start = time.perf_counter()
        final = initial
        while (session.running if session else step_count < ticks):
            if time.perf_counter() - loop_start > args.max_wall_seconds:
                raise TimeoutError(f"Stepping exceeded wall budget of {args.max_wall_seconds:g} seconds")
            initializing = bool(session and session.phase == "initialize")
            stage = "initialization_steps" if initializing else "competition_steps" if session else "passive_steps"
            began = time.perf_counter()
            with timed(stage):
                state = session.step() if session else world.step({})
                if state is None:
                    state = world.snapshot()
            step_wall = time.perf_counter() - began
            step_count += 1
            init_count += int(initializing)
            mode_count += int(not initializing)
            with timed("observation"):
                final = json_value(state)
                expected = float(initial["time"]) + step_count / hz
                if not math.isclose(final["time"], expected, abs_tol=1e-9, rel_tol=0):
                    raise ValidationError(f"Clock changed dt: expected {expected}, got {final['time']}")
                if final.get("tick") != initial["tick"] + step_count:
                    raise ValidationError("Tick count does not match the number of fixed physics steps")
                height = None if preload is None else world._physics.getBasePositionAndOrientation(
                    preload, physicsClientId=world.client)[0][2]
                height = json_value(height, "preload_z_m")
                if height is not None:
                    highest = max(highest, height)
                pose = final["pose"]
                writer.writerow({"time": final["time"], "tick": final["tick"],
                                 "mode_time": final.get("mode_time", mode_count / hz),
                                 "phase": final.get("phase", "passive"), "step_wall_seconds": step_wall,
                                 "x_m": pose[0], "y_m": pose[1], "z_m": pose[2], "yaw_rad": pose[3],
                                 "preload_z_m": height, "joints_json": json.dumps(final.get("joints", [])),
                                 "motors_json": json.dumps(final.get("motors", {})),
                                 "scoring_json": json.dumps(final.get("scoring", {}))})
                if mode_count / hz >= next_progress:
                    progress = {"mode_time": mode_count / hz, "world_time": final["time"],
                                "stepping_wall_seconds": sum(timings.get(key, 0) for key in
                                                             ("initialization_steps", "competition_steps", "passive_steps")),
                                "pose": pose, "preload_z_m": height, "joints": final.get("joints", []),
                                "scoring": final.get("scoring", {})}
                    report["progress"].append(progress)
                    print(json.dumps({"progress": progress}, allow_nan=False), flush=True)
                    csv_stream.flush()
                    next_progress += args.progress_seconds
            if initializing and session.phase != 'initialize':
                sample_activity('initialization_complete', final, mode_count)
            if diagnostic_ticks and not initializing and mode_count >= next_diagnostic:
                # Leave the terminal tick for the final sample, avoiding a duplicate query.
                if (session.running if session else step_count < ticks):
                    sample_activity('periodic', final, mode_count)
                next_diagnostic += diagnostic_ticks
        sample_activity('final', final, mode_count)
        finalize_loop_metrics()
        with timed("validation"):
            report["final_state"] = final
            report["final_field_objects"] = field_states(world)
            if mode_count != ticks:
                raise ValidationError(f"Expected {ticks} mode ticks, observed {mode_count}")
            if session and (not session.initialized or session.runtime is not None or final.get("running")):
                raise ValidationError("Native initialization or mode shutdown did not complete")
            if session and any("Route timeout" in line for line in logs):
                raise ValidationError("The PROS program reported a route timeout")
            evidence = {"initial_height_m": initial_height, "maximum_height_m": highest,
                        "lift_m": None if highest is None else highest - initial_height}
            if preload is not None:
                evidence["final_scoring"] = json_value(world._scoring())
                evidence["final_robot_contact"] = bool(world._physics.getContactPoints(
                    preload, world.robot, physicsClientId=world.client))
            report["preload"] = evidence
            if full:
                validate_claw(evidence, logs, final)
            report["checks"].update(fixed_clock=True, finite_state=True, requested_backend=True,
                                     full_claw_placement=full)
        report["status"] = "passed"
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        finalize_loop_metrics()
        if session is not None or world is not None:
            try:
                from types import SimpleNamespace
                from eersim.diagnostics import performance_snapshot
                report["performance"] = json_value(performance_snapshot(
                    session if session is not None else SimpleNamespace(world=world)))
            except Exception as exc:
                report["performance_error"] = f"{type(exc).__name__}: {exc}"
            try:
                with timed("close"):
                    (session or world).close()
            except Exception as exc:
                report["status"] = "failed"
                report["close_error"] = f"{type(exc).__name__}: {exc}"
        # The final disabled callback is part of the run, including on failures.
        if workspace is not None and original_hashes is not None:
            try:
                after = source_hashes(workspace)
                report["source_after_sha256"] = after
                report["checks"]["source_unchanged"] = after == original_hashes
                if after != original_hashes:
                    raise ValidationError("The original PROS source/include files changed during this run")
            except Exception as exc:
                report["status"] = "failed"
                report["source_error"] = f"{type(exc).__name__}: {exc}"
        for stream in (csv_stream, log_stream):
            if stream is not None:
                stream.close()
        report["logs"] = logs
        report["total_wall_seconds"] = time.perf_counter() - wall_start
        if diagnostic_ticks:
            if previous_profile is None:
                os.environ.pop('EERSIM_PROFILE', None)
            else:
                os.environ['EERSIM_PROFILE'] = previous_profile
        output.write_text(json.dumps(json_value(report), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return report


def main(argv=None) -> int:
    arguments = parser().parse_args(argv)
    try:
        report = execute(arguments)
    except (ValueError, OSError) as exc:
        print(f"Benchmark setup failed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({key: report.get(key) for key in
                      ("status", "validation_scope", "step_realtime_factor", "error", "source_error")}))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
