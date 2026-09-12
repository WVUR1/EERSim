"""Diagnostic failures, privacy and process ownership without graphics or CUDA."""
from copy import deepcopy
import csv
import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
import zipfile

import pytest

from eersim import diagnostics, diagnostics_export
from eersim.config import SimConfig, load_config
from eersim.performance import PerformanceRecorder


class FakeWorld:
    def __init__(self):
        self.time = 0.0
        self.physics_status = {'backend': 'test', 'hardware_accelerated': False}
        self.field_objects, self.joints, self.motor_bindings = [], [], []
        self.performance = PerformanceRecorder(enabled=True)
        self.render_calls = 0

    def snapshot(self):
        return {'time': self.time, 'pose': [0, 0, 0, 0], 'velocity': [0, 0, 0], 'motors': {},
                'private_profile_note': 'EXCLUDE_SNAPSHOT_EXTRA'}

    def render(self, *args, **kwargs):
        self.render_calls += 1
        raise AssertionError('Rendering was explicitly disabled')


class FakeSession:
    def __init__(self, values, log):
        self.world = FakeWorld()
        self.performance = PerformanceRecorder(enabled=True)
        self.dt = 1 / values['physics_hz']
        self.elapsed_ticks = self.init_ticks = 0
        self.phase = 'autonomous'
        self.running = True
        self.closed = False
        self.started = None
        self.fail_at = None

    def start(self, mode, executable):
        self.started = (mode, executable)

    def step(self):
        with self.performance.span('session.step'):
            if self.fail_at == self.elapsed_ticks:
                raise RuntimeError('Expected physics failure')
            self.world.time += self.dt
            self.elapsed_ticks += 1

    def close(self):
        self.closed = True


class FakeSampler:
    instances = None

    def __init__(self, directory, *, logical_device=0):
        self.directory = directory
        self.logical_device = logical_device
        self.started = self.stopped = False
        if self.instances is not None:
            self.instances.append(self)

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True
        return {'status': 'query_failed', 'rows_written': 0, 'reason': 'NVIDIA unsupported in this fixture'}


def execute_case(tmp_path, *, config=None, session_hook=None, **options):
    sessions, samplers = [], []

    def factory(values, log):
        session = FakeSession(values, log)
        if session_hook:
            session_hook(session)
        sessions.append(session)
        return session

    class Sampler(FakeSampler):
        instances = samplers

    result = diagnostics.run_case(config or {}, tmp_path / 'capture', session_factory=factory,
                                  sampler_factory=options.pop('sampler_factory', Sampler),
                                  max_steps=options.pop('max_steps', 3),
                                  include_rendering=options.pop('include_rendering', False), **options)
    return result, sessions[0], samplers


def test_unsupported_gpu_metrics_do_not_fail_physics_or_create_graphics(tmp_path):
    result, session, samplers = execute_case(tmp_path)
    assert result['status'] == 'complete' and result['steps'] == 3
    assert result['simulation_seconds'] == pytest.approx(3 / 240)
    assert result['gpu_sampling']['status'] == 'query_failed'
    assert result['physics']['hardware_accelerated'] is False
    assert result['performance']['session']['spans']['session.step']['count'] == 3
    assert result['capture_performance']['spans']['capture.session_step']['count'] == 3
    assert result['render_frames'] == session.world.render_calls == 0
    assert session.closed and samplers[0].started and samplers[0].stopped
    assert 'EXCLUDE_SNAPSHOT_EXTRA' not in json.dumps(result)
    progress = list(csv.DictReader((tmp_path / 'capture/report/progress.csv').open(newline='')))
    assert progress and float(progress[-1]['simulation_seconds']) > 0


def test_sampler_start_error_is_optional_and_preserves_physics_capture(tmp_path):
    class BrokenSampler(FakeSampler):
        def start(self):
            raise OSError('Optional sampler could not start')

    result, session, _ = execute_case(tmp_path, sampler_factory=BrokenSampler)
    assert result['status'] == 'complete' and result['steps'] == 3
    assert session.closed
    assert 'sampl' in json.dumps(result).lower()


def test_physics_failure_retains_partial_timings_and_closes_resources(tmp_path):
    result, session, samplers = execute_case(tmp_path, session_hook=lambda session: setattr(session, 'fail_at', 2))
    assert result['status'] == 'failed' and result['failed_phase'] == 'physics'
    assert result['steps'] == 2 and 'Expected physics failure' in result['error']
    assert result['performance']['session']['spans']['session.step']['count'] == 3
    assert session.closed and samplers[0].stopped
    assert (tmp_path / 'capture/private/failure.txt').is_file()
    assert json.loads((tmp_path / 'capture/report/case.json').read_text())['status'] == 'failed'


def test_log_copy_error_cannot_skip_session_close(monkeypatch, tmp_path):
    def unreadable(*args):
        raise PermissionError('Diagnostic log is locked')

    monkeypatch.setattr(diagnostics, '_copy_worker_log', unreadable)
    result, session, samplers = execute_case(tmp_path)
    assert session.closed, 'Log collection failure must not orphan the physics worker'
    assert samplers[0].stopped and result['timing_seconds']['close'] >= 0


def test_completed_physics_is_checkpointed_before_optional_render_failure(tmp_path):
    checkpoint = []

    def hook(session):
        def render(*args, **kwargs):
            checkpoint.append(json.loads((tmp_path / 'capture/report/case.json').read_text()))
            raise RuntimeError('Mock offscreen render failure')
        session.world.render = render

    result, session, samplers = execute_case(tmp_path, session_hook=hook, include_rendering=True)
    assert checkpoint[0]['status'] == 'physics_complete_render_pending'
    assert checkpoint[0]['steps'] == 3 and checkpoint[0]['simulation_seconds'] > 0
    assert checkpoint[0]['performance']['session']['spans']['session.step']['count'] == 3
    assert result['status'] == 'failed' and result['failed_phase'] == 'render'
    assert session.closed and samplers[0].stopped


def test_compile_error_keeps_source_diagnostics_private_and_closes_scene(tmp_path):
    secret = 'PRIVATE_CPP_SOURCE_SECRET'
    workspace = tmp_path / 'private robot project'
    workspace.mkdir()

    def build(path, log):
        assert path == workspace
        log('main.cpp: secret source line = ' + secret)
        raise RuntimeError(secret)

    result, session, samplers = execute_case(tmp_path, config={'workspace': str(workspace)}, builder=build)
    assert result['status'] == 'failed' and result['failed_phase'] == 'build'
    assert session.closed and not samplers and session.started is None
    assert secret not in json.dumps(result)
    assert secret in (tmp_path / 'capture/private/compiler.log').read_text()
    archive = diagnostics.make_archive(tmp_path, [tmp_path / 'capture'])
    with zipfile.ZipFile(archive) as report:
        assert report.namelist() == ['capture/case.json']
        assert secret.encode() not in b''.join(report.read(name) for name in report.namelist())


def test_public_configuration_drops_unknown_nested_extras_and_uses_hashes(tmp_path):
    asset = tmp_path / 'robot.urdf'
    asset.write_text('PRIVATE_ASSET_CONTENT')
    values = SimConfig.from_dict({'robot_urdf': str(asset), 'notes': 'PRIVATE_TOP_LEVEL',
        'duration_overrides': {'skills_auto': 10, 'notes': 'PRIVATE_NESTED'},
        'noise': {'position_std_m': .01, 'PRIVATE_NOISE_KEY': 123}}).to_dict()
    result = diagnostics.public_config(values)
    assert result['robot_urdf']['name'] == asset.name and len(result['robot_urdf']['sha256']) == 64
    assert result['duration_overrides'] == {'skills_auto': 10}
    assert result['noise']['position_std_m'] == .01
    assert 'PRIVATE_' not in json.dumps(result)
    assert str(tmp_path) not in json.dumps(result)


def test_archive_uses_report_allowlist_and_omits_private_sources_and_assets(tmp_path):
    case = tmp_path / '240hz'
    report = case / 'report'
    report.mkdir(parents=True)
    (case / 'private').mkdir()
    (case / 'private/main.cpp').write_text('PRIVATE_SOURCE')
    (report / 'robot.urdf').write_text('PRIVATE_ASSET')
    (report / 'unrequested.txt').write_text('PRIVATE_EXTRA')
    (report / 'case.json').write_text('{"status":"failed"}')
    (tmp_path / 'system.json').write_text('{"cpu":"fixture"}')
    archive = diagnostics.make_archive(tmp_path, [case])
    with zipfile.ZipFile(archive) as capture:
        assert set(capture.namelist()) == {'system.json', '240hz/case.json'}
        assert 'PRIVATE_' not in ''.join(capture.read(name).decode() for name in capture.namelist())


def test_export_writes_unique_local_recipes_without_loading_world(monkeypatch, tmp_path):
    import eersim.runner
    monkeypatch.setattr(eersim.runner, 'Session', lambda *args: pytest.fail('Export must not start a session'))
    monkeypatch.setattr(subprocess, 'Popen', lambda *args, **kwargs: pytest.fail('Export must not start a process'))
    parent = tmp_path / 'Unicode Ω and spaces'
    parent.mkdir()
    config = {'workspace': str(parent / 'my project'), 'physics_hz': 120, 'notes': 'local recipe notes'}
    first = diagnostics_export.export_diagnostics(config, parent, mode='match_auto',
             wall_seconds=12, compare_rates=False, include_rendering=False)
    second = diagnostics_export.export_diagnostics(config, parent, include_rendering=False)
    assert first != second
    assert load_config(first / 'experiment.json')['workspace'] == config['workspace']
    recipe = json.loads((first / 'launcher.json').read_text())
    assert recipe['mode'] == 'match_auto' and not recipe['compare_rates'] and not recipe['include_rendering']
    assert recipe['wall_seconds'] == 12 and recipe['prefix'] == ['-m', 'eersim']
    assert not list(first.glob('results-*'))


@pytest.mark.parametrize('options', [{'wall_seconds': float('nan')}, {'wall_seconds': 601},
                                    {'wall_seconds': 0}, {'mode': 'unknown'}])
def test_invalid_recipe_options_write_nothing(tmp_path, options):
    with pytest.raises(ValueError):
        diagnostics_export.export_diagnostics({}, tmp_path, **options)
    assert not list(tmp_path.iterdir())


def fake_supervisor(monkeypatch, *, watchdog=False):
    from eersim import diagnostic_hardware, diagnostic_process
    calls, jobs, terminated = [], [], []
    monkeypatch.setattr(diagnostic_hardware, 'system_info', lambda **kwargs: {'cpu': 'fixture'})

    class Job:
        def __init__(self):
            self.closed = False
            jobs.append(self)
        def attach(self, process):
            self.process = process
        def close(self):
            self.closed = True

    class Process:
        def __init__(self, command, **options):
            self.returncode = None if watchdog else 0
            self.pid = 1234
            calls.append((command, options))
            self.case = Path(command[command.index('--diagnostic-output') + 1])
            if not watchdog:
                (self.case / 'report').mkdir()
                diagnostics.write_json(self.case / 'report/case.json', {'status': 'complete', 'steps': 2})
        def poll(self):
            return self.returncode

    def terminate(process):
        terminated.append(process)
        if process.returncode is None:
            process.returncode = -9

    monkeypatch.setattr(diagnostic_process, 'WindowsJob', Job)
    monkeypatch.setattr(diagnostics.subprocess, 'Popen', Process)
    monkeypatch.setattr(diagnostics, 'terminate_case', terminate)
    if watchdog:
        ticks = iter([0, 10000])
        monkeypatch.setattr(diagnostics.time, 'monotonic', lambda: next(ticks))
    return calls, jobs, terminated


def test_supervisor_runs_requested_rates_in_isolated_argv_and_archives(monkeypatch, tmp_path):
    calls, jobs, terminated = fake_supervisor(monkeypatch)
    destination = tmp_path / 'unicode Ω capture & spaces'
    original = SimConfig.from_dict({'physics_hz': 360}).to_dict()
    before = deepcopy(original)
    result = diagnostics.run_diagnostics(original, destination, compare_rates=True, max_steps=2)
    assert result['status'] == 'complete' and result['rate_order'] == [240, 120]
    assert original == before and all(job.closed for job in jobs)
    assert len(terminated) == (0 if os.name == 'nt' else len(calls))
    for (command, options), rate in zip(calls, [240, 120]):
        profile = Path(command[command.index('--config') + 1])
        assert load_config(profile)['physics_hz'] == rate
        assert str(destination) in str(profile) and '--measure-rendering' not in command
        assert '--headless' in command and '--diagnostic-case' in command
        assert not options.get('shell')
        assert options['env']['EERSIM_PROFILE'] == '1'
        assert Path(options['env']['LOCALAPPDATA']).is_relative_to(destination)
    with zipfile.ZipFile(destination / 'EERSim-diagnostics.zip') as report:
        assert {'240hz/case.json', '120hz/case.json', 'summary.json'} <= set(report.namelist())
        assert all('private' not in name for name in report.namelist())


def test_watchdog_failure_still_closes_job_and_produces_archive(monkeypatch, tmp_path):
    calls, jobs, terminated = fake_supervisor(monkeypatch, watchdog=True)
    output = tmp_path / 'timeout'
    result = diagnostics.run_diagnostics({}, output, wall_seconds=1)
    assert result['status'] == 'partial_failure'
    assert result['cases'][0]['watchdog_timeout'] is True
    assert result['cases'][0]['child_exit_code'] == -9
    assert len({id(process) for process in terminated}) == len(calls) == 1 and jobs[0].closed
    assert (output / 'EERSim-diagnostics.zip').is_file()


@pytest.mark.skipif(os.name == 'nt', reason='POSIX owned process-group cleanup semantics')
def test_exited_case_parent_still_has_its_owned_process_group_terminated(monkeypatch):
    killed = []
    process = SimpleNamespace(pid=12345, poll=lambda: 1, wait=lambda **kwargs: 1)
    monkeypatch.setattr(diagnostics.os, 'killpg', lambda pid, signal: killed.append((pid, signal)))
    diagnostics.terminate_case(process)
    assert killed and all(pid == process.pid for pid, _ in killed)


@pytest.mark.skipif(os.name != 'nt', reason='Windows argv quoting and hidden process launch')
def test_exported_powershell_preserves_unicode_spaces_quotes_and_literal_metacharacters(tmp_path):
    parent = tmp_path / 'Unicode Ω spaces & literal $(text)'
    parent.mkdir()
    folder = diagnostics_export.export_diagnostics({}, parent, wall_seconds=1, compare_rates=False, include_rendering=False)
    recorder = parent / 'argv recorder.py'
    captured = parent / 'received.json'
    recorder.write_text('import json, os, sys\nfrom pathlib import Path\n'
                        'Path(sys.argv[1]).write_text(json.dumps(sys.argv[2:]), encoding="utf-8")\n', encoding='utf-8')
    literals = ['with spaces', 'literal"quote', 'trailing\\', '雪 & $(not code)']
    recipe = json.loads((folder / 'launcher.json').read_text())
    recipe['prefix'] = [str(recorder), str(captured), *literals]
    (folder / 'launcher.json').write_text(json.dumps(recipe), encoding='utf-8')
    powershell = Path(os.environ['SystemRoot']) / 'System32/WindowsPowerShell/v1.0/powershell.exe'
    result = subprocess.run([str(powershell), '-NoProfile', '-ExecutionPolicy', 'Bypass',
                             '-File', str(folder / 'run-diagnostics.ps1')], capture_output=True,
                            text=True, errors='replace', timeout=20, creationflags=subprocess.CREATE_NO_WINDOW)
    assert result.returncode == 0, result.stdout + result.stderr
    arguments = json.loads(captured.read_text(encoding='utf-8'))
    assert arguments[:len(literals)] == literals
    assert arguments[arguments.index('--config') + 1] == str(folder / 'experiment.json')
    assert '--measure-rendering' not in arguments and '--compare-rates' not in arguments


@pytest.mark.skipif(os.name != 'nt', reason='Windows kernel Job Object ownership')
def test_windows_job_closes_descendant_after_case_parent_has_exited(tmp_path):
    from ctypes import wintypes
    from eersim.diagnostic_process import WindowsJob
    gate, child_pid = tmp_path / 'attached', tmp_path / 'child-pid'
    script = tmp_path / 'job fixture.py'
    script.write_text('import pathlib, subprocess, sys, time\n'
        'gate, pid = map(pathlib.Path, sys.argv[1:])\n'
        'deadline = time.monotonic() + 10\n'
        'while not gate.exists():\n'
        '    if time.monotonic() > deadline: raise SystemExit(2)\n'
        '    time.sleep(.01)\n'
        'child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], creationflags=subprocess.CREATE_NO_WINDOW)\n'
        'pid.write_text(str(child.pid))\n', encoding='utf-8')
    api = ctypes.WinDLL('kernel32', use_last_error=True)
    api.OpenProcess.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    api.OpenProcess.restype = wintypes.HANDLE
    api.WaitForSingleObject.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    api.WaitForSingleObject.restype = wintypes.DWORD
    api.TerminateProcess.argtypes = [wintypes.HANDLE, wintypes.UINT]
    api.CloseHandle.argtypes = [wintypes.HANDLE]
    process = None
    handle = None
    job = WindowsJob()
    try:
        process = subprocess.Popen([sys.executable, str(script), str(gate), str(child_pid)],
                                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                    creationflags=subprocess.CREATE_NO_WINDOW)
        job.attach(process)
        gate.write_text('ready')
        assert process.wait(timeout=15) == 0
        handle = api.OpenProcess(0x00100000 | 0x1000 | 0x0001, False, int(child_pid.read_text()))
        assert handle, 'Could not open the owned fixture descendant'
        assert api.WaitForSingleObject(handle, 0) == 258  # WAIT_TIMEOUT: still alive
        job.close()
        assert api.WaitForSingleObject(handle, 5000) == 0  # terminated by closing job
    finally:
        job.close()
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        if handle:
            if api.WaitForSingleObject(handle, 0) == 258:
                api.TerminateProcess(handle, 1)
                api.WaitForSingleObject(handle, 5000)
            api.CloseHandle(handle)


def test_source_cli_captures_native_bullet_motion_with_graphics_imports_blocked(tmp_path):
    from eersim.bridge import _compiler, BuildError
    from eersim.config import save_config
    try:
        compiler = _compiler()
    except BuildError:
        pytest.skip('Native diagnostic integration needs a host C++ compiler')
    fixture = tmp_path / 'native fixture with spaces'
    fixture.mkdir()
    robot = fixture / 'robot.urdf'
    robot.write_text('''<robot name="diagnostic_shaft">
<link name="base"><inertial><mass value="1"/><inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
<link name="rotor"><inertial><mass value=".5"/><inertia ixx=".02" iyy=".02" izz=".02" ixy="0" ixz="0" iyz="0"/></inertial>
<collision><geometry><box size=".1 .02 .02"/></geometry></collision></link>
<joint name="shaft" type="continuous"><parent link="base"/><child link="rotor"/><origin xyz="0 0 .2"/>
<axis xyz="0 -1 0"/><limit effort="10" velocity="200"/></joint></robot>''', encoding='utf-8')
    motors = fixture / 'motors.json'
    motors.write_text(json.dumps({'fixed_base': True, 'motors': [
        {'port': 1, 'joint': 'shaft', 'cartridge': 'green', 'gear_ratio': 1, 'efficiency': 1}]}))
    field = fixture / 'field.json'
    field.write_text(json.dumps({'name': 'Diagnostic fixture',
        'field': {'size': [2, 2], 'floor': True, 'walls': False}, 'objects': []}))
    workspace = fixture / 'PROS workspace'
    (workspace / 'src').mkdir(parents=True)
    (workspace / 'include').mkdir()
    source = workspace / 'src/main.cpp'
    source.write_text('''#include "api.h"
pros::Motor shaft(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
extern "C" {
void initialize() {}
void competition_initialize() {}
void autonomous() { while (true) { shaft.move_voltage(6000); pros::delay(10); } }
void opcontrol() { autonomous(); }
void disabled() { shaft.move_voltage(0); }
}
''', encoding='utf-8')
    before = source.read_bytes()
    profile = fixture / 'experiment.json'
    save_config({'robot_urdf': str(robot), 'robot_config': str(motors), 'game_pack': str(field),
                 'workspace': str(workspace), 'start_pose': [0, 0, 1, 0], 'gravity': 0,
                 'physics_backend': 'bullet', 'renderer': 'cpu', 'physics_hz': 240,
                 'duration_overrides': {'skills_auto': .1}}, profile)
    # Every Python process inherits this guard. Any Qt, VTK or OpenGL import is
    # a test failure; NVIDIA collection is disabled rather than querying a GPU.
    guard = fixture / 'guard'
    guard.mkdir()
    (guard / 'sitecustomize.py').write_text('import importlib.abc, sys\n'
        'class RejectGraphics(importlib.abc.MetaPathFinder):\n'
        '    def find_spec(self, fullname, path=None, target=None):\n'
        '        if fullname.split(".")[0] in {"PySide6", "vtk", "vtkmodules", "OpenGL"}:\n'
        '            raise RuntimeError("Graphics import forbidden during diagnostic test: " + fullname)\n'
        'sys.meta_path.insert(0, RejectGraphics())\n'
        'from eersim import diagnostic_hardware\n'
        'diagnostic_hardware._find_nvidia_smi = lambda: None\n', encoding='utf-8')
    output = fixture / 'results'
    package_root = Path(diagnostics.__file__).resolve().parent.parent
    environment = dict(os.environ, CXX=compiler,
        PYTHONPATH=os.pathsep.join([str(guard), str(package_root), os.environ.get('PYTHONPATH', '')]))
    command = [sys.executable, '-m', 'eersim', '--headless', '--diagnostics', '--config', str(profile),
               '--diagnostic-output', str(output), '--diagnostic-wall-seconds', '1', '--diagnostic-max-steps', '40']
    options = {'creationflags': subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {}
    process = subprocess.run(command, env=environment, capture_output=True, text=True,
                             errors='replace', timeout=90, **options)
    assert process.returncode == 0, process.stdout + process.stderr
    result = json.loads((output / '240hz/report/case.json').read_text())
    assert result['status'] == 'complete' and result['physics']['backend'] == 'bullet'
    assert result['native_competition_ticks'] > 0 and result['render_frames'] == 0
    assert result['gpu_sampling']['status'] == 'command_unavailable'
    assert result['final']['motors']['1']['position'] > 0
    assert result['timing_seconds']['native_build'] > 0
    assert source.read_bytes() == before
    with zipfile.ZipFile(output / 'EERSim-diagnostics.zip') as report:
        assert all(not name.endswith(('.cpp', '.urdf')) and '/private/' not in name for name in report.namelist())
