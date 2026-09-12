"""Real PhysX CPU pin placement persists through sleep, then clears on motion."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import traceback

import numpy as np
import pytest


pytestmark = pytest.mark.skipif(
    any(importlib.util.find_spec(name) is None for name in ('ovphysx', 'ovstage', 'scipy', 'pybullet')),
    reason='The optional pinned PhysX CPU runtime is unavailable')


def pose_edit_worker(connection, scene, manifest, config, work):
    """Real isolated runtime with a test-only native pose edit command.

    Production intentionally does not expose arbitrary scene mutation. This
    test uses the SDK pose binding to check observation/cache invalidation,
    independently of the force needed to extract a seated collision shape.
    """
    from eersim.physx_worker import _Runtime
    runtime = None
    try:
        runtime = _Runtime(Path(scene), manifest, config, Path(work))
        connection.send((True, {'state': runtime.state(), 'actor_paths': runtime.actor_paths,
                               'dof_names': runtime.dof_names, 'physics_status': runtime.status}))
        while True:
            command, payload = connection.recv()
            if command == 'close':
                break
            if command == 'test_pose_edit':
                values = runtime.buffers['poses'].copy()
                values[payload['row']] = payload['pose']
                runtime.bindings['poses'].write(values, indices=np.array([payload['row']], dtype=np.int32))
                result = True
            elif command == 'contacts':
                result = runtime.contacts.capture()
            else:
                result = getattr(runtime, command)(payload)
            connection.send((True, result))
    except EOFError:
        pass
    except BaseException as error:
        traceback.print_exc()
        connection.send((False, f'{type(error).__name__}: {error}'))
    finally:
        if runtime is not None:
            runtime.close()
        connection.close()


def test_actual_goal_pin_score_survives_silent_contacts_and_clears_after_native_pose_edit(tmp_path, monkeypatch):
    from eersim import physx_worker, physx_world
    from eersim.config import ASSETS
    from eersim.physx_world import PhysXWorld

    monkeypatch.setenv('EERSIM_PROFILE', '1')
    monkeypatch.setenv('EERSIM_RENDERER', 'cpu')
    monkeypatch.setenv('OPENBLAS_NUM_THREADS', '1')
    monkeypatch.setattr(physx_worker, '_worker_main', pose_edit_worker)
    monkeypatch.setattr(physx_world, 'user_data_dir', lambda: tmp_path / 'runtime')
    game_path = ASSETS / 'games/override-2026-27.json'
    authored = json.loads(game_path.read_text())
    goal = deepcopy(next(item for item in authored['objects'] if item.get('name') == 'goal_red_5'))
    pin = deepcopy(next(item for item in authored['objects'] if item.get('kind') == 'pin'))
    for item in (goal, pin):
        item['urdf'] = str((game_path.parent / item['urdf']).resolve())
        item['rotation'] = [0, 0, 0]
    goal.update(name='test_goal', position=[0, 0, 0])
    pin.update(name='test_pin', position=[0, 0, .0845])
    field = tmp_path / 'field.json'
    field.write_text(json.dumps({'field': {'size': [2, 2], 'floor': True, 'walls': False},
                                'objects': [goal, pin]}))
    robot = tmp_path / 'fixed_robot.urdf'
    robot.write_text('''<robot name="remote_fixed_fixture"><link name="base">
      <inertial><mass value="1"/><inertia ixx=".01" iyy=".01" izz=".01"
       ixy="0" ixz="0" iyz="0"/></inertial>
      <collision><geometry><box size=".1 .1 .1"/></geometry></collision>
      </link></robot>''')
    config = {'robot_urdf': str(robot), 'robot_config': '', 'game_pack': str(field),
              'physics_backend': 'physx_cpu', 'physics_hz': 240, 'physics_solver_iterations': 100,
              'renderer': 'cpu', 'motors': [], 'fixed_base': True, 'start_pose': [.7, .7, 1, 0]}
    observations = []
    try:
        with PhysXWorld(config) as world:
            pin_id = next(item['id'] for item in world.field_objects if item['name'] == 'test_pin')
            goal_id = next(item['id'] for item in world.field_objects if item['name'] == 'test_goal')
            seen_placement = seen_reports = False
            silent_ticks = 0
            sleeping_pose = None
            for _ in range(1920):
                snapshot = world.step()
                gauges = world._process.call('diagnostics')['performance']['gauges']
                raw = gauges['worker.contacts.raw_reports_last']
                pose = world._physics.getBasePositionAndOrientation(pin_id)
                scoring = world._scoring()  # Direct full scorer also covers its 30 Hz cache intervals.
                observations.append({'tick': world.tick, 'raw': raw, 'score': scoring['placed_pins'],
                                     'position': list(pose[0]), 'quaternion': list(pose[1]),
                                     'retained_pairs': gauges['worker.contacts.retained_body_pairs_last']})
                seen_reports |= raw > 0
                seen_placement |= scoring['placed_pins'] == 1
                if raw == 0 and seen_reports and seen_placement:
                    assert scoring['placements'] == [{'pin': 'test_pin', 'goal': 'test_goal', 'alliance': 'red'}]
                    assert world._has_contact(pin_id, goal_id)
                    assert gauges['worker.contacts.retained_body_pairs_last'] >= 1
                    if sleeping_pose is None:
                        sleeping_pose = pose
                    else:
                        np.testing.assert_array_equal(np.asarray(pose[0]), np.asarray(sleeping_pose[0]))
                        np.testing.assert_array_equal(np.asarray(pose[1]), np.asarray(sleeping_pose[1]))
                    silent_ticks += 1
                    if silent_ticks >= 120:
                        assert snapshot['scoring']['placed_pins'] == 1
                        break
                else:
                    silent_ticks = 0
                    sleeping_pose = None
            assert silent_ticks >= 120, 'CAD pin did not retain its score through half a second of silent, stationary native contacts'
            assert not world._physics.getContactPoints(pin_id, goal_id)
            row = world._physics.keys[(pin_id, -1)]
            moved_pose = world._physics._state['poses'][row].copy()
            moved_pose[:3] += [.3, 0, .3]
            assert world._process.call('test_pose_edit', {'row': row, 'pose': moved_pose})
            snapshot = world.step()
            position, _ = world._physics.getBasePositionAndOrientation(pin_id)
            assert position[0] > .25 and position[2] > .3
            assert world._scoring()['placed_pins'] == 0
            assert not world._has_contact(pin_id, goal_id)
            # The public scoring snapshot refreshes at30 Hz.
            for _ in range(8):
                snapshot = world.step()
            assert snapshot['scoring']['placed_pins'] == 0
            json.dumps(snapshot, allow_nan=False)
    finally:
        (tmp_path / 'scoring-sleep-observations.json').write_text(json.dumps(observations, indent=2))
