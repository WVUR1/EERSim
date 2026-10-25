"""Measure fixed-step physics throughput without opening a window or robot code."""
from __future__ import annotations

import argparse
import cProfile
import json
from pathlib import Path
import sys
import time

# Support the documented source-checkout invocation from any working directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.benchmark_integration import cpu_threads_argument, hardware_metadata, json_value, validate_backend


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    scene = parser.add_mutually_exclusive_group()
    scene.add_argument('--config', type=Path, help='Saved experiment profile')
    scene.add_argument('--practice', action='store_true', help='Use the simple practice field')
    parser.add_argument('--backend', choices=('bullet', 'physx_cpu', 'physx_gpu', 'mujoco'),
                        default='bullet', help='Explicit engine; never silently use auto fallback')
    parser.add_argument('--physics-hz', type=int, choices=(120, 240), help='Override the profile physics rate')
    parser.add_argument('--iterations', type=int, help='Override solver budget; semantics differ by engine')
    parser.add_argument('--cpu-threads', type=cpu_threads_argument, default=None,
                        help='Native CPU workers (0..1024); 0 runs inline, omission preserves the profile setting')
    parser.add_argument('--device', type=int, default=0, help='Logical CUDA device for PhysX GPU only')
    parser.add_argument('--ticks', type=int, default=240, help='Maximum measured physics ticks')
    parser.add_argument('--warmup', type=int, default=0, help='Unmeasured physics ticks after loading')
    parser.add_argument('--max-wall-seconds', type=float, default=30,
                        help='Stop measurement after this budget, at a complete physics tick')
    parser.add_argument('--json', type=Path, help='Save the report to this file')
    parser.add_argument('--profile', type=Path, help='Also save cProfile data (adds overhead)')
    args = parser.parse_args(argv)
    if args.ticks < 1 or args.warmup < 0:
        parser.error('--ticks must be positive and --warmup must be nonnegative')
    if args.device < 0 or (args.iterations is not None and args.iterations < 1):
        parser.error('--device must be nonnegative and --iterations must be positive')
    import math
    if not math.isfinite(args.max_wall_seconds) or args.max_wall_seconds <= 0:
        parser.error('--max-wall-seconds must be positive and finite')

    from eersim.config import ASSETS, default_config, load_config
    from eersim.demos import claw_demo_config
    from eersim.physics_backend import create_world

    config = load_config(args.config) if args.config else claw_demo_config()
    if args.practice:
        config = default_config()
        config['game_pack'] = str(ASSETS / 'games' / 'practice.json')
    config.update(physics_backend=args.backend, physics_device=args.device, renderer='cpu')
    if args.physics_hz is not None:
        config['physics_hz'] = args.physics_hz
    if args.iterations is not None:
        config['physics_solver_iterations'] = args.iterations
    if args.cpu_threads is not None:
        config['physics_cpu_threads'] = args.cpu_threads
    for output in (args.json, args.profile):
        if output:
            output.parent.mkdir(parents=True, exist_ok=True)
    profiler = cProfile.Profile() if args.profile else None
    hardware = hardware_metadata(args.backend, args.device)
    print('Loading scene; no rendering or PROS execution...', file=sys.stderr, flush=True)
    started = time.perf_counter()
    with create_world(config) as world:
        load_seconds = time.perf_counter() - started
        validate_backend(world.physics_status, args.backend, world.config.physics_solver_iterations)
        prepared_time = world.time
        for _ in range(args.warmup):
            world.step()
        print('Measuring physics...', file=sys.stderr, flush=True)
        if profiler:
            profiler.enable()
        started = time.perf_counter()
        ticks = 0
        while ticks < args.ticks:
            world.step()
            ticks += 1
            elapsed = time.perf_counter() - started
            if elapsed >= args.max_wall_seconds:
                break
        if profiler:
            profiler.disable()
            profiler.dump_stats(str(args.profile))
        report = {
            'scene': world.game.get('name'),
            'robot': Path(world.config.robot_urdf).name,
            'hardware': hardware, 'physics': json_value(world.physics_status),
            'physics_hz': world.config.physics_hz,
            'solver_iterations': world.physics_status['solver_iterations'],
            'load_seconds': load_seconds, 'warmup_ticks': args.warmup,
            'prepared_world_time': prepared_time,
            'requested_ticks': args.ticks, 'measured_ticks': ticks,
            'wall_budget_reached': elapsed >= args.max_wall_seconds,
            'simulated_seconds': ticks * world.dt, 'wall_seconds': elapsed,
            'real_time_factor': ticks * world.dt / elapsed,
            'dynamic_field_bodies': len(world._dynamic_field_bodies),
            'contact_points': len(world._physics.getContactPoints(physicsClientId=world.client)),
            'contact_scope': 'Authoritative backend API queried after measurement; manifold conventions vary by engine',
            'profiled': profiler is not None,
            'includes': 'World.step only; excludes load, warmup, rendering, native code and GUI',
            'notes': ['Passive scene only: this is not the native claw autonomous validation.',
                      'Equal timestep and iteration budgets do not imply equal accuracy across engines.'],
        }
    formatted = json.dumps(report, indent=2) + '\n'
    if args.json:
        args.json.write_text(formatted, encoding='utf-8')
    print(formatted, end='')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
