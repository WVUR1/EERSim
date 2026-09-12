"""Bounded installed/source performance capture, with one isolated process per case."""
from __future__ import annotations

import argparse
import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
import zipfile

from .config import SimConfig, load_config, save_config
from .runner import MODES


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, value) -> None:
    """Readers see either complete progress document, including on Windows."""
    text = json.dumps(value, indent=2, allow_nan=False, default=str) + '\n'
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(text, encoding='utf-8')
    try:
        os.replace(temporary, path)
    except PermissionError:
        # Antivirus/readers can hold the destination briefly on Windows.
        # Progress can miss an update without interrupting a physics capture.
        if path.name != 'progress.json':
            raise


class Redactor:
    """Remove known local roots from the shareable logs; no environment dump."""
    def __init__(self, config, output):
        paths = [(str(Path.home()), '<home>'), (str(Path(output).resolve()), '<diagnostics>'),
                 (str(Path(__file__).resolve().parent.parent), '<application>'),
                 (sys.prefix, '<python>')]
        for key in ('workspace', 'robot_urdf', 'robot_config', 'game_pack'):
            if config.get(key):
                path = Path(config[key]).resolve()
                paths.append((str(path), '<' + key + '>'))
                if path.parent != Path(path.anchor):
                    paths.append((str(path.parent), '<' + key + '_folder>'))
        self.paths = sorted(paths, key=lambda item: len(item[0]), reverse=True)

    def text(self, value) -> str:
        result = str(value)
        for path, replacement in self.paths:
            for form in {path, path.replace('\\', '/'), path.replace('/', '\\')}:
                result = re.sub(re.escape(form), lambda _: replacement, result,
                                flags=re.IGNORECASE if os.name == 'nt' else 0)
        return result[:16000]

    def value(self, value):
        if isinstance(value, dict):
            return {str(key): self.value(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self.value(item) for item in value]
        if isinstance(value, str):
            return self.text(value)
        if hasattr(value, 'tolist'):
            return self.value(value.tolist())
        if isinstance(value, float) and not math.isfinite(value):
            return str(value)
        return value


def public_config(config: dict) -> dict:
    # Extra profile fields can contain arbitrary user notes/secrets. Export an
    # explicit physical-setting allowlist, not all preserved application extras.
    keys = ('physics_hz', 'physics_backend', 'physics_device', 'physics_solver_iterations',
            'start_pose', 'gravity', 'friction', 'plastic_friction', 'restitution',
            'seed', 'noise', 'renderer', 'render_fps', 'duration_overrides')
    result = {key: copy.deepcopy(config[key]) for key in keys if key in config}
    for key, allowed in (('noise', set(SimConfig().noise)), ('duration_overrides', set(MODES))):
        source = config.get(key, {})
        result[key] = {name: value for name, value in source.items() if name in allowed
                       and isinstance(value, (int, float)) and not isinstance(value, bool)
                       and math.isfinite(value)} if isinstance(source, dict) else {}
    for key in ('robot_urdf', 'robot_config', 'game_pack'):
        if config.get(key):
            path = Path(config[key])
            item = {'name': path.name, 'available': path.is_file()}
            if path.is_file():
                with path.open('rb') as stream:
                    item['sha256'] = hashlib.file_digest(stream, 'sha256').hexdigest()
            result[key] = item
    result['workspace_selected'] = bool(config.get('workspace'))
    result['extra_field_object_count'] = len(config.get('field_objects', []))
    return result


def physics_snapshot(session) -> dict:
    state = session.world.snapshot()
    # No full robot config, raw source or arbitrary profile extras in reports.
    return {key: state[key] for key in ('time', 'tick', 'pose', 'velocity', 'motors', 'otos',
            'imu', 'sensor_age_ms', 'scoring') if key in state}


def workspace_fingerprint(workspace) -> dict:
    """Identify source changes without publishing names or file contents."""
    if not workspace:
        return {'selected': False}
    root = Path(workspace)
    digest = hashlib.sha256()
    count = size = 0
    try:
        for subtree in ('src', 'include'):
            for parent, directories, files in os.walk(root / subtree):
                directories[:] = sorted(name for name in directories if name not in {'.git', '.eersim', '__pycache__'})
                for name in sorted(files):
                    path = Path(parent) / name
                    count += 1
                    size += path.stat().st_size
                    if count > 20000 or size > 256 * 1024**2:
                        return {'selected': True, 'status': 'size_limit', 'files': count, 'bytes': size}
                    digest.update(path.relative_to(root).as_posix().encode('utf-8'))
                    digest.update(b'\0')
                    with path.open('rb') as stream:
                        digest.update(hashlib.file_digest(stream, 'sha256').digest())
        return {'selected': True, 'status': 'complete', 'files': count, 'bytes': size,
                'sha256': digest.hexdigest()}
    except OSError as exc:
        return {'selected': True, 'status': 'unavailable', 'error_type': type(exc).__name__}


def performance_snapshot(session, *, reset=False) -> dict:
    result = {}
    worker = getattr(session.world, '_process', None)
    if worker is not None:
        if not getattr(worker, 'failed', False):
            try:
                result['worker'] = worker.call('diagnostics', {'reset': reset})['performance']
            except Exception as exc:
                result['worker_error'] = str(exc)
        if hasattr(worker, 'performance'):
            result['ipc'] = worker.performance.snapshot(reset=reset)
    for name, owner in (('session', session), ('world', session.world)):
        if hasattr(owner, 'performance'):
            result[name] = owner.performance.snapshot(reset=reset)
    return result


def _copy_worker_log(session, destination: Path, redactor: Redactor) -> None:
    process = getattr(session.world, '_process', None)
    source = getattr(process, 'log_path', None)
    if source and Path(source).is_file():
        with Path(source).open('rb') as stream:
            size = Path(source).stat().st_size
            stream.seek(max(0, size - 256_000))
            text = stream.read(256_000).decode('utf-8', 'replace')
        destination.write_text('\n'.join(redactor.text(line) for line in text.splitlines()), encoding='utf-8')


def run_case(config: dict, directory: Path, *, mode='skills_auto', wall_seconds=120,
             include_rendering=False, max_steps=None, session_factory=None,
             builder=None, sampler_factory=None) -> dict:
    """One capture; failure reports are retained and resources always closed."""
    from .runner import Session
    from .bridge import build_workspace
    from .diagnostic_hardware import NvidiaSmiSampler
    from .performance import PerformanceRecorder

    values = SimConfig.from_dict(config).to_dict()
    directory = Path(directory).resolve()
    report_dir = directory / 'report'
    private = directory / 'private'
    report_dir.mkdir(parents=True, exist_ok=True)
    private.mkdir(parents=True, exist_ok=True)
    redactor = Redactor(values, directory)
    result = {'schema_version': 1, 'status': 'running', 'started_utc': utc_now(),
              'scope': 'instrumented performance capture, not accuracy or scoring acceptance',
              'requested': public_config(values), 'mode': mode,
              'wall_budget_seconds': wall_seconds, 'includes_render_probe': include_rendering,
              'timing_seconds': {}, 'steps': 0, 'render_frames': 0,
              'controller': 'neutral', 'stage_timings_are_nested': True}
    events = []
    phase = 'setup'
    session = sampler = None
    total_started = time.perf_counter()
    capture = PerformanceRecorder(enabled=True)

    def log(message):
        # Compiler diagnostics can include source lines. They stay local.
        path = private / 'compiler.log' if phase == 'build' else report_dir / 'events.log'
        with path.open('a', encoding='utf-8') as stream:
            stream.write((str(message) if phase == 'build' else redactor.text(message)) + '\n')
        if phase != 'build' and len(events) < 500:
            events.append(redactor.text(message))

    def progress(message):
        write_json(directory / 'progress.json', {'message': message, 'phase': phase,
                    'steps': result['steps'], 'updated_utc': utc_now()})

    try:
        progress('Loading the scene')
        started = time.perf_counter()
        session = (session_factory or Session)(values, log)
        result['timing_seconds']['scene_load'] = time.perf_counter() - started
        result['physics'] = copy.deepcopy(getattr(session.world, 'physics_status', {}))
        result['scene'] = {'field_objects': len(getattr(session.world, 'field_objects', [])),
                           'robot_joints': len(getattr(session.world, 'joints', [])),
                           'motor_bindings': len(getattr(session.world, 'motor_bindings', []))}
        result['initial'] = physics_snapshot(session)
        if values.get('workspace'):
            phase = 'build'
            progress('Building the selected PROS workspace')
            started = time.perf_counter()
            result['workspace_before'] = workspace_fingerprint(values['workspace'])
            executable = (builder or build_workspace)(Path(values['workspace']), log)
            result['timing_seconds']['native_build'] = time.perf_counter() - started
            phase = 'start'
            started = time.perf_counter()
            session.start(mode, executable)
            result['timing_seconds']['native_start'] = time.perf_counter() - started
        result['setup_performance'] = performance_snapshot(session, reset=True)
        try:
            sampler = (sampler_factory or NvidiaSmiSampler)(report_dir, logical_device=values['physics_device'])
            sampler.start()
        except Exception as exc:
            result['gpu_sampling_start_error'] = redactor.text(exc)
            log('GPU telemetry unavailable; continuing the timing capture: ' + redactor.text(exc))
        phase = 'physics'
        result['physics_started_utc'] = utc_now()
        progress('Measuring physics and PROS execution')
        started = time.perf_counter()
        initial_time = float(session.world.time)
        initial_cpu = time.process_time()
        steps_wall = 0.0
        last_sample = -1.0
        with (report_dir / 'progress.csv').open('w', newline='', encoding='utf-8') as output:
            writer = csv.DictWriter(output, fieldnames=['wall_seconds', 'simulation_seconds',
                'mode_seconds', 'phase', 'ticks', 'step_wall_seconds', 'step_realtime_factor'])
            writer.writeheader()
            while True:
                elapsed = time.perf_counter() - started
                if elapsed >= wall_seconds:
                    result['stop_reason'] = 'wall_budget'
                    break
                if max_steps is not None and result['steps'] >= max_steps:
                    result['stop_reason'] = 'step_limit'
                    break
                if values.get('workspace') and not session.running:
                    result['stop_reason'] = 'competition_complete'
                    break
                step_started = time.perf_counter()
                with capture.span('capture.session_step'):
                    session.step()
                steps_wall += time.perf_counter() - step_started
                result['steps'] += 1
                elapsed = time.perf_counter() - started
                simulated = float(session.world.time) - initial_time
                if elapsed - last_sample >= .25:
                    writer.writerow({'wall_seconds': elapsed, 'simulation_seconds': simulated,
                        'mode_seconds': session.elapsed_ticks * session.dt,
                        'phase': session.phase, 'ticks': result['steps'],
                        'step_wall_seconds': steps_wall,
                        'step_realtime_factor': simulated / steps_wall if steps_wall else 0})
                    output.flush()
                    progress(f'{values["physics_hz"]} Hz: {elapsed:.0f}/{wall_seconds:g} wall seconds, '
                             f'{simulated:.2f} simulated seconds, {simulated / elapsed:.3f}x')
                    last_sample = elapsed
            result['timing_seconds']['physics_loop'] = time.perf_counter() - started
        result['timing_seconds']['physics_steps'] = steps_wall
        result['timing_seconds']['parent_cpu_during_loop'] = time.process_time() - initial_cpu
        result['simulation_seconds'] = float(session.world.time) - initial_time
        result['step_realtime_factor'] = result['simulation_seconds'] / steps_wall if steps_wall else 0
        result['loop_realtime_factor'] = result['simulation_seconds'] / result['timing_seconds']['physics_loop']
        result['native_initialization_ticks'] = session.init_ticks
        result['native_competition_ticks'] = session.elapsed_ticks
        result['physics_finished_utc'] = utc_now()
        result['performance'] = performance_snapshot(session)
        result['capture_performance'] = capture.snapshot()
        result['final'] = physics_snapshot(session)
        # A driver crash in the optional rendering probe must not erase the
        # completed physics timings. The supervisor retains this partial report.
        result['status'] = 'physics_complete_render_pending' if include_rendering else 'complete'
        write_json(report_dir / 'case.json', redactor.value(result))

        if include_rendering:
            phase = 'render'
            progress('Measuring stationary offscreen rendering separately')
            result['render_started_utc'] = utc_now()
            timings = []
            started = time.perf_counter()
            camera = values.get('camera', {})
            camera = {key: camera[key] for key in ('yaw', 'pitch', 'distance', 'target') if key in camera}
            for _ in range(12):
                frame_start = time.perf_counter()
                session.world.render(960, 720, **camera)
                timings.append(time.perf_counter() - frame_start)
                if time.perf_counter() - started >= 30:
                    break
            result['render_frames'] = len(timings)
            result['renderer'] = copy.deepcopy(getattr(session.world, 'renderer_status', {}))
            result['render_probe'] = {'resolution': [960, 720], 'stationary': True,
                'first_frame_seconds': timings[0], 'subsequent_frame_seconds': timings[1:],
                'steady_mean_seconds': sum(timings[1:]) / len(timings[1:]) if len(timings) > 1 else None}
            result['timing_seconds']['render_probe'] = time.perf_counter() - started
            result['render_finished_utc'] = utc_now()
        result['status'] = 'complete'
    except Exception as exc:
        result['status'] = 'failed'
        result['failed_phase'] = phase
        result['error'] = (f'{type(exc).__name__}: native compilation failed; details retained locally under private/'
                           if phase == 'build' else redactor.text(f'{type(exc).__name__}: {exc}'))
        (private / 'failure.txt').write_text(traceback.format_exc(), encoding='utf-8')
        if session is not None:
            try:
                result['performance'] = performance_snapshot(session)
            except Exception:
                pass
    finally:
        phase = 'cleanup'
        if sampler is not None:
            try:
                result['gpu_sampling'] = sampler.stop()
            except Exception as exc:
                result['gpu_sampling_error'] = redactor.text(exc)
        if session is not None:
            started = time.perf_counter()
            try:
                _copy_worker_log(session, report_dir / 'physx.log', redactor)
            except Exception as exc:
                result['worker_log_error'] = redactor.text(exc)
            try:
                session.close()
            except Exception as exc:
                result['status'] = 'failed'
                result['cleanup_error'] = redactor.text(exc)
            result['timing_seconds']['close'] = time.perf_counter() - started
        result['finished_utc'] = utc_now()
        if 'workspace_before' in result:
            result['workspace_after'] = workspace_fingerprint(values.get('workspace'))
            before, after = result['workspace_before'], result['workspace_after']
            result['workspace_unchanged'] = (before['sha256'] == after['sha256']
                if before.get('status') == after.get('status') == 'complete' else None)
        result['timing_seconds']['total'] = time.perf_counter() - total_started
        result['events'] = events
        result = redactor.value(result)
        write_json(report_dir / 'case.json', result)
        progress('Capture ' + result['status'])
    return result


def child_command() -> list[str]:
    return [sys.executable] if getattr(sys, 'frozen', False) else [sys.executable, '-m', 'eersim']


def terminate_case(process) -> None:
    """Stop only the child session and its descendants, including a stuck worker."""
    if os.name == 'nt' and process.poll() is not None:
        return
    if os.name == 'nt':
        subprocess.run([str(Path(os.environ['SYSTEMROOT']) / 'System32/taskkill.exe'),
                        '/PID', str(process.pid), '/T', '/F'], capture_output=True, timeout=15,
                       creationflags=subprocess.CREATE_NO_WINDOW)
    else:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            pass
        # The parent can exit before a child that ignores TERM. The group was
        # created exclusively for this capture, so reap its remaining members.
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def make_archive(directory: Path, cases: list[Path]) -> Path:
    """Archive a fixed report allowlist; never recurse through user inputs."""
    destination = directory / 'EERSim-diagnostics.zip'
    allowed = ['case.json', 'events.log', 'physx.log', 'progress.csv', 'gpu-samples.csv', 'gpu-summary.json']
    with zipfile.ZipFile(destination, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for name in ('summary.json', 'SUMMARY.txt', 'system.json'):
            path = directory / name
            if path.is_file() and not path.is_symlink():
                archive.write(path, name)
        for case in cases:
            if case.is_symlink() or (case / 'report').is_symlink():
                continue
            for name in allowed:
                path = case / 'report' / name
                if path.is_file() and not path.is_symlink() and path.resolve().is_relative_to(case.resolve()):
                    archive.write(path, case.name + '/' + name)
    return destination


def summary_text(summary: dict) -> str:
    lines = ['EERSim performance diagnostics', '',
             'Instrumented, unthrottled physics capacity; not an accuracy or GUI playback test.',
             'Nested timing spans overlap: do not add every row together.',
             'GPU utilization describes the device and may include other applications.', '']
    for case in summary['cases']:
        lines += [f'{case["rate_hz"]} Hz: {case.get("status", "unknown")}',
                  f'  Physics: {case.get("physics", {})}',
                  f'  Stepping RTF: {case.get("step_realtime_factor", "unavailable")}',
                  f'  Loop RTF: {case.get("loop_realtime_factor", "unavailable")}',
                  f'  Simulated seconds: {case.get("simulation_seconds", "unavailable")}',
                  f'  Steps: {case.get("steps", 0)}; stop: {case.get("stop_reason", "failure")}',
                  f'  Detailed report: {case["directory"]}/case.json', '']
    lines += ['Reports exclude raw PROS source, assets and private compiler output.',
              'Logs can contain messages printed by the robot program. Nothing was uploaded.',
              'NVIDIA unavailable/unsupported telemetry does not invalidate recorded CPU stage timings.']
    return '\n'.join(lines) + '\n'


def run_diagnostics(config: dict, directory: Path, *, mode='skills_auto', wall_seconds=120,
                    compare_rates=False, include_rendering=False, max_steps=None) -> dict:
    from .diagnostic_hardware import system_info
    from .diagnostic_process import WindowsJob
    values = SimConfig.from_dict(config).to_dict()
    directory = Path(directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=False)
    redactor = Redactor(values, directory)
    rates = [240, 120] if compare_rates else [values['physics_hz']]
    summary = {'schema_version': 1, 'status': 'running', 'started_utc': utc_now(),
               'cases': [], 'mode': mode, 'wall_seconds_per_case': wall_seconds,
               'rate_order': rates, 'instrumented': True, 'interactive_gui': False}
    cases = []
    current = None
    job = None
    try:
        write_json(directory / 'progress.json', {'message': 'Collecting hardware information'})
        write_json(directory / 'system.json', redactor.value(system_info(logical_device=values['physics_device'])))
        for rate in rates:
            case = directory / f'{rate}hz'
            cases.append(case)
            private = case / 'private'
            private.mkdir(parents=True)
            config_path = private / 'experiment.json'
            settings = copy.deepcopy(values)
            settings['physics_hz'] = rate
            save_config(settings, config_path)
            command = child_command() + ['--headless', '--diagnostic-case', '--config', str(config_path),
                        '--diagnostic-output', str(case), '--mode', mode,
                        '--diagnostic-wall-seconds', str(wall_seconds)]
            if include_rendering:
                command.append('--measure-rendering')
            if max_steps is not None:
                command += ['--diagnostic-max-steps', str(max_steps)]
            environment = dict(os.environ, EERSIM_PROFILE='1',
                               PYTHONPATH=str(Path(__file__).resolve().parent.parent) + os.pathsep + os.environ.get('PYTHONPATH', ''))
            # Windowed frozen stdout/logs and worker caches remain inside this
            # owned case. No installed files or existing user data are changed.
            for key, child in [('LOCALAPPDATA', 'local'), ('APPDATA', 'roaming'),
                               ('XDG_DATA_HOME', 'data'), ('XDG_CACHE_HOME', 'cache')]:
                root = private / 'runtime' / child
                root.mkdir(parents=True)
                environment[key] = str(root)
            flags = {'creationflags': subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {'start_new_session': True}
            write_json(directory / 'progress.json', {'message': f'{rate} Hz: starting isolated capture'})
            with (private / 'stdout.log').open('wb') as stdout, (private / 'stderr.log').open('wb') as stderr:
                job = WindowsJob()
                current = subprocess.Popen(command, stdout=stdout, stderr=stderr, env=environment, **flags)
                job.attach(current)
                # Compiler and worker each have180s setup bounds; steady-state
                # calls and cleanup need a bounded margin beyond the soft budget.
                deadline = time.monotonic() + wall_seconds + 450
                timed_out = False
                while current.poll() is None:
                    if time.monotonic() >= deadline:
                        timed_out = True
                        terminate_case(current)
                        break
                    try:
                        update = json.loads((case / 'progress.json').read_text())
                        update['message'] = f'{rate} Hz: ' + update['message']
                        write_json(directory / 'progress.json', update)
                    except (OSError, ValueError):
                        pass
                    time.sleep(.5)
                code = current.returncode
                job.close()
                job = None
                if os.name != 'nt':
                    terminate_case(current)
            path = case / 'report/case.json'
            if path.is_file():
                result = json.loads(path.read_text())
            else:
                result = {'status': 'failed', 'error': 'Child exited before producing its report; raw startup logs retained locally under private/'}
                path.parent.mkdir(exist_ok=True)
                write_json(path, result)
            result.update(rate_hz=rate, directory=case.name, child_exit_code=code, watchdog_timeout=timed_out)
            if code != 0 or timed_out:
                result['status'] = 'failed'
            write_json(path, result)
            summary['cases'].append({key: result[key] for key in ('rate_hz', 'directory', 'status', 'physics',
                'steps', 'simulation_seconds', 'step_realtime_factor', 'loop_realtime_factor', 'stop_reason',
                'child_exit_code', 'watchdog_timeout', 'error', 'workspace_before', 'workspace_after',
                'workspace_unchanged') if key in result})
            current = None
        summary['status'] = 'complete' if all(case['status'] == 'complete' for case in summary['cases']) else 'partial_failure'
    except (Exception, KeyboardInterrupt) as exc:
        summary['status'] = 'interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed'
        summary['error'] = redactor.text(f'{type(exc).__name__}: {exc}')
    finally:
        if job is not None:
            job.close()
        if current is not None:
            terminate_case(current)
        summary['finished_utc'] = utc_now()
        write_json(directory / 'summary.json', summary)
        (directory / 'SUMMARY.txt').write_text(summary_text(summary), encoding='utf-8')
        make_archive(directory, cases)
        write_json(directory / 'progress.json', {'message': 'Finished: EERSim-diagnostics.zip (' + summary['status'] + ')'})
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Exportable EERSim performance capture')
    actions = parser.add_mutually_exclusive_group(required=True)
    actions.add_argument('--diagnostics', action='store_true')
    actions.add_argument('--diagnostic-case', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--diagnostic-output', type=Path, required=True)
    parser.add_argument('--diagnostic-wall-seconds', type=float, default=120)
    parser.add_argument('--diagnostic-max-steps', type=int, help='Optional exact tick cap for a short smoke capture')
    parser.add_argument('--compare-rates', action='store_true')
    parser.add_argument('--measure-rendering', action='store_true')
    parser.add_argument('--mode', choices=tuple(MODES), default='skills_auto')
    args = parser.parse_args(argv)
    if not math.isfinite(args.diagnostic_wall_seconds) or not 1 <= args.diagnostic_wall_seconds <= 600:
        parser.error('The wall budget must be between 1 and 600 seconds')
    if args.diagnostic_max_steps is not None and args.diagnostic_max_steps < 1:
        parser.error('The tick cap must be positive')
    config = load_config(args.config)
    if args.diagnostic_case:
        os.environ['EERSIM_PROFILE'] = '1'
        result = run_case(config, args.diagnostic_output, mode=args.mode,
            wall_seconds=args.diagnostic_wall_seconds, include_rendering=args.measure_rendering,
            max_steps=args.diagnostic_max_steps)
    else:
        result = run_diagnostics(config, args.diagnostic_output, mode=args.mode,
            wall_seconds=args.diagnostic_wall_seconds, compare_rates=args.compare_rates,
            include_rendering=args.measure_rendering, max_steps=args.diagnostic_max_steps)
    return 0 if result['status'] == 'complete' else 1
