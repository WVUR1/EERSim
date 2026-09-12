"""Real CPU diagnostic lifecycle; no CUDA context or graphics are created."""
import hashlib
import importlib.util
import json

import pytest


pytestmark = pytest.mark.skipif(
    any(importlib.util.find_spec(name) is None for name in ('ovphysx', 'ovstage', 'scipy', 'pybullet')),
    reason='The optional PhysX CPU runtime is unavailable')


def test_native_cpu_tick_cap_records_stages_disables_and_reaps_children(tmp_path, monkeypatch):
    from eersim import physx_world
    from eersim.bridge import BuildError, NativeRuntime, _compiler
    from eersim.diagnostics import run_case
    from eersim.runner import Session

    try:
        _compiler()
    except BuildError as error:
        pytest.skip(str(error))
    monkeypatch.setenv('EERSIM_PROFILE', '1')
    monkeypatch.setenv('EERSIM_RENDERER', 'cpu')
    monkeypatch.setattr(physx_world, 'user_data_dir', lambda: tmp_path / 'runtime')
    robot = tmp_path / 'robot.urdf'
    robot.write_text('''<robot name="diagnostic_rotor">
      <link name="base"><inertial><mass value="1"/>
       <inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <link name="rotor"><inertial><mass value=".5"/>
       <inertia ixx=".02" iyy=".02" izz=".02" ixy="0" ixz="0" iyz="0"/></inertial>
       <collision><geometry><box size=".1 .02 .02"/></geometry></collision></link>
      <joint name="shaft" type="continuous"><parent link="base"/><child link="rotor"/>
       <origin xyz="0 0 .2" rpy=".3 .2 -.4"/><axis xyz="0 -1 0"/>
       <limit effort="10" velocity="200"/></joint></robot>''')
    field = tmp_path / 'field.json'
    field.write_text(json.dumps({'field': {'floor': False, 'walls': False}, 'objects': []}))
    workspace = tmp_path / 'pros-workspace'
    (workspace / 'src').mkdir(parents=True)
    (workspace / 'include').mkdir()
    source = workspace / 'src/main.cpp'
    source.write_text('''#include "api.h"
      pros::Motor shaft(1);
      extern "C" {
      void initialize() { pros::lcd::initialize(); }
      void competition_initialize() {}
      void autonomous() { while (true) { shaft.move_voltage(6000); pros::delay(10); } }
      void opcontrol() { autonomous(); }
      void disabled() { shaft.move_voltage(0); pros::lcd::set_text(0, "DISABLED_CONFIRMED"); }
      }
    ''')
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    config = {'robot_urdf': str(robot), 'robot_config': '', 'game_pack': str(field),
              'workspace': str(workspace), 'fixed_base': True, 'gravity': 0,
              'start_pose': [0, 0, 1, 37], 'physics_backend': 'physx_cpu',
              'physics_hz': 240, 'renderer': 'cpu',
              'motors': [{'port': 1, 'joint': 'shaft', 'cartridge': 'green'}]}
    retained = {}

    class ObservedRuntime(NativeRuntime):
        def __init__(self, executable):
            super().__init__(executable)
            retained['native_process'] = self.process
            self.disabled_results = retained['disabled_results'] = []

        def step(self, snapshot, mode, dt, controller):
            result = super().step(snapshot, mode, dt, controller)
            if mode == 'disabled':
                self.disabled_results.append(result)
            return result

    def session_factory(values, log):
        session = Session(values, log, runtime_factory=ObservedRuntime)
        retained['session'] = session
        retained['worker_process'] = session.world._process.process
        retained['scene_directory'] = session.world._physics_directory
        return session

    class UnavailableSampler:
        def __init__(self, *_args, **_kwargs):
            pass

        def start(self):
            return self

        def stop(self):
            return {'status': 'command_unavailable'}

    result = run_case(config, tmp_path / 'capture', wall_seconds=5, max_steps=12,
                      include_rendering=False, session_factory=session_factory,
                      sampler_factory=UnavailableSampler)
    assert result['status'] == 'complete', result
    assert result['stop_reason'] == 'step_limit' and result['steps'] == 12
    assert result['native_initialization_ticks'] == 1
    assert result['native_competition_ticks'] == 11
    assert result['render_frames'] == 0
    assert result['simulation_seconds'] == pytest.approx(.05, abs=1e-12)
    assert result['final']['tick'] - result['initial']['tick'] == 12
    assert result['final']['motors']['1']['velocity'] > 0
    for name in ('worker.control.write', 'worker.native.step', 'worker.state.read', 'worker.contacts.read_convert'):
        assert result['performance']['worker']['spans'][name]['count'] == 12
    assert result['performance']['ipc']['spans']['physx.rpc.step']['count'] == 12
    assert result['performance']['session']['counters']['session.native.calls'] == 12
    assert result['setup_performance']['worker']['counters']['worker.bootstrap_steps'] == 1
    assert 'worker.bootstrap_steps' not in result['performance']['worker']['counters']
    assert len(retained['disabled_results']) == 1
    disabled = retained['disabled_results'][0]
    assert disabled['lcd']['0'] == 'DISABLED_CONFIRMED'
    assert disabled['motors']['1']['value'] == 0
    assert retained['native_process'].returncode == 0
    assert retained['worker_process'].exitcode == 0
    assert not retained['worker_process'].is_alive()
    assert not retained['scene_directory'].exists()
    assert retained['session'].phase == 'disabled' and not retained['session'].running
    assert hashlib.sha256(source.read_bytes()).hexdigest() == before
    json.dumps(result, allow_nan=False)
