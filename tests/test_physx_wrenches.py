"""Exercise actual worker submission logic without creating a physics runtime."""
from types import SimpleNamespace

import numpy as np
import pytest

from eersim.performance import PerformanceRecorder
from eersim.physx_worker import _Runtime


DRIVES = ('target_velocity', 'target_position', 'max_force', 'stiffness', 'damping', 'effort')


class Binding:
    def __init__(self, shape):
        self.shape, self.calls = shape, []

    def write(self, values, indices=None):
        assert values.shape == self.shape and values.dtype == np.float32
        if indices is not None:
            assert indices.ndim == 1 and indices.dtype == np.int32
        self.calls.append((values.copy(), None if indices is None else indices.copy()))


def worker(enabled=True):
    runtime = _Runtime.__new__(_Runtime)
    runtime.np = np
    runtime.performance = PerformanceRecorder(enabled)
    runtime.dt = 1 / 240
    # Native binding row order differs from the all-actor observation order.
    runtime.actor_rows = {'field_a': 0, 'robot': 1, 'field_b': 2, 'field_c': 3}
    runtime.regular_paths = ['field_c', 'field_a', 'field_b']
    runtime.articulation_actor_rows = [1]
    runtime.buffers = {key: np.zeros((1, 2), dtype=np.float32) for key in DRIVES}
    runtime.buffers.update(body_wrenches=np.zeros((3, 9), dtype=np.float32),
                           link_wrenches=np.zeros((1, 1, 9), dtype=np.float32))
    runtime.bindings = {key: Binding(value.shape) for key, value in runtime.buffers.items()}
    runtime.native_steps = []
    runtime.physx = SimpleNamespace(step_sync=runtime.native_steps.append)
    runtime.state = lambda: {'observed_after_steps': len(runtime.native_steps)}
    controls = {key: np.full((1, 2), index + 1, dtype=np.float32) for index, key in enumerate(DRIVES)}
    controls['wrenches'] = np.zeros((4, 9), dtype=np.float32)
    return runtime, controls


@pytest.mark.parametrize('enabled', [False, True])
def test_position_only_rows_do_not_wake_regular_bodies_or_change_robot_controls(enabled):
    runtime, controls = worker(enabled)
    controls['wrenches'][:, 6:] = [[1, 2, 3], [4, 5, 6], [-7, 8, 9], [-10, -11, 12]]
    controls['wrenches'][0, 0] = -0.0
    assert runtime.step(controls) == {'observed_after_steps': 1}
    assert runtime.bindings['body_wrenches'].calls == []
    for key in DRIVES:
        values, indices = runtime.bindings[key].calls[0]
        np.testing.assert_array_equal(values, controls[key])
        assert indices is None
    link_values, indices = runtime.bindings['link_wrenches'].calls[0]
    np.testing.assert_array_equal(link_values, controls['wrenches'][[1]][None, ...])
    assert indices is None
    assert runtime.native_steps == [1 / 240]
    stats = runtime.performance.snapshot()
    if enabled:
        assert stats['counters'] == {'worker.control.tensor_writes': 7,
            'worker.control.regular_wrench_rows_submitted': 0,
            'worker.control.regular_wrench_rows_skipped': 3, 'worker.steps': 1}
    else:
        assert stats['counters'] == {} and stats['spans'] == {}


def test_partial_write_maps_binding_rows_and_preserves_arbitrarily_small_nonzero_force():
    runtime, controls = worker()
    controls['wrenches'][0] = [1e-30, 0, 0, 0, 0, 0, 2, 3, 4]
    controls['wrenches'][2, 6:] = [8, 9, 10]  # No force: not an active row.
    controls['wrenches'][3] = [0, 0, 0, 0, -2e-30, 0, 11, 12, 13]
    runtime.step(controls)
    values, indices = runtime.bindings['body_wrenches'].calls[0]
    np.testing.assert_array_equal(indices, [0, 1])
    np.testing.assert_array_equal(values, controls['wrenches'][[3, 0, 2]])
    assert values[1, 0] != 0 and values[0, 4] != 0
    counts = runtime.performance.snapshot()['counters']
    assert counts['worker.control.tensor_writes'] == 8
    assert counts['worker.control.regular_wrench_rows_submitted'] == 2
    assert counts['worker.control.regular_wrench_rows_skipped'] == 1


def test_zero_tick_after_force_pulse_does_not_resubmit_the_previous_wrench():
    runtime, controls = worker()
    controls['wrenches'][3, 2] = .125
    runtime.step(controls)
    controls['wrenches'][:, :6] = 0
    controls['wrenches'][:, 6:] = 15
    runtime.step(controls)
    assert len(runtime.bindings['body_wrenches'].calls) == 1
    assert not np.any(runtime.buffers['body_wrenches'][:, :6])
    assert runtime.native_steps == [1 / 240, 1 / 240]
    counts = runtime.performance.snapshot()['counters']
    assert counts['worker.control.tensor_writes'] == 15
    assert counts['worker.control.regular_wrench_rows_submitted'] == 1
    assert counts['worker.control.regular_wrench_rows_skipped'] == 5
    assert counts['worker.steps'] == 2


def test_no_regular_binding_needs_no_selection_and_metrics_count_completed_writes_on_step_error():
    runtime, controls = worker()
    del runtime.bindings['body_wrenches']
    del runtime.buffers['body_wrenches']

    def failed_step(_dt):
        raise RuntimeError('Synthetic native step failure')

    runtime.physx.step_sync = failed_step
    with pytest.raises(RuntimeError, match='Synthetic native step failure'):
        runtime.step(controls)
    counts = runtime.performance.snapshot()['counters']
    assert counts['worker.control.tensor_writes'] == 7
    assert counts['worker.control.regular_wrench_rows_submitted'] == 0
    assert counts['worker.control.regular_wrench_rows_skipped'] == 0
    assert 'worker.steps' not in counts
