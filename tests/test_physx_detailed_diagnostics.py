"""Timing and read-only activity contracts without starting an SDK or process."""
from copy import deepcopy
from types import SimpleNamespace

import pytest

from eersim.performance import PerformanceRecorder
from eersim.physx_contacts import ContactReader
from eersim.physx_worker import _Runtime, diagnostic_thread_count
from test_physx_contacts import Binding, Physics, actor


def fixture_binding():
    actors = [actor('/World/floor', 0, mass=0), actor('/World/pin', 1), actor('/World/robot', 2)]
    reports = {path: [(11, (0, 0, 0), (0, 0, 1), -.001, 3.)]
               for path in ('/World/pin', '/World/robot')}
    return actors, Binding(['/World/pin', '/World/robot'], reports, {11: '/World/floor/collider_0'})


def test_contact_substages_attribute_binding_and_conversion_without_changing_payload(monkeypatch):
    actors, binding = fixture_binding()
    plain = ContactReader(Physics(deepcopy(binding)), {'actors': actors}, capacity=32,
                          performance=PerformanceRecorder(False))
    now = [0]
    recorder = PerformanceRecorder(True, clock=lambda: now[0])
    reader = ContactReader(Physics(binding), {'actors': actors}, capacity=32, performance=recorder)

    def delayed(owner, name, nanoseconds):
        original = getattr(owner, name)
        def call(*args, **kwargs):
            now[0] += nanoseconds
            return original(*args, **kwargs)
        monkeypatch.setattr(owner, name, call)

    delayed(binding, 'read_raw_contact_data', 10_000_000)
    delayed(binding, 'get_other_actor_paths_from_ids', 20_000_000)
    delayed(reader, '_materialize_contacts', 30_000_000)
    delayed(reader, '_stationary_presence', 40_000_000)
    poses = {path: [0, 0, .1, 0, 0, 0, 1] for path in ('/World/pin', '/World/robot')}
    assert reader.capture_runtime(2, poses) == plain.capture_runtime(2, poses)
    spans = recorder.snapshot()['spans']
    for name, duration in {'binding_read': .01, 'id_resolution': .02,
                           'robot_materialization': .03, 'pair_retention': .04,
                           'payload_validation': 0., 'pair_selection': 0.}.items():
        assert spans['worker.contacts.' + name]['total'] == pytest.approx(duration)
        assert spans['worker.contacts.' + name]['count'] == 1
    # A cache hit avoids the ID resolver; a sleeping contact retains presence only.
    binding.reports = {}
    plain._binding.reports = {}
    assert reader.capture_runtime(2, poses) == plain.capture_runtime(2, poses) == {
        'contacts': [], 'contact_body_pairs': [(0, 1)]}
    assert reader.retained_pair_count == 1
    assert binding.resolutions == [[11]]
    assert recorder.snapshot()['spans']['worker.contacts.id_resolution']['total'] == pytest.approx(.02)
    reader.close()
    plain.close()


def test_disabled_contact_profiling_never_reads_clock():
    actors, binding = fixture_binding()
    def forbidden():
        pytest.fail('Disabled contact timing read a clock')
    recorder = PerformanceRecorder(False, clock=forbidden)
    reader = ContactReader(Physics(binding), {'actors': actors}, capacity=32, performance=recorder)
    assert reader.capture_runtime(2)['contacts']
    assert reader.capture()
    assert recorder.snapshot()['spans'] == {}
    reader.close()


def test_contact_validation_errors_remain_fatal_and_are_timed():
    actors, binding = fixture_binding()
    binding.reports['/World/pin'][0] = (11, (float('nan'), 0, 0), (0, 0, 1), 0, 3.)
    recorder = PerformanceRecorder(True)
    reader = ContactReader(Physics(binding), {'actors': actors}, capacity=32, performance=recorder)
    with pytest.raises(RuntimeError, match='nonfinite'):
        reader.capture_runtime(2)
    assert set(recorder.snapshot()['spans']) == {
        'worker.contacts.binding_read', 'worker.contacts.payload_validation'}
    assert not binding.resolutions
    reader.close()


def test_activity_rpc_is_explicit_read_only_and_maps_only_exact_unique_names(monkeypatch):
    import eersim.physx_verification as verification
    runtime = _Runtime.__new__(_Runtime)
    runtime.performance = PerformanceRecorder(True)
    runtime.performance.increment('worker.steps', 17)
    runtime.physx = object()
    runtime.manifest = {'scene_path': '/World/physicsScene', 'actors': [
        {**actor('/World/rigid_2_base', 2), 'label': 'pin_x/pin'},
        {**actor('/World/rigid_3_base', 3), 'label': 'cup_y/cup'}]}
    calls = []
    def inspect(physx, path):
        assert physx is runtime.physx and path == runtime.manifest['scene_path']
        calls.append(path)
        return {'counts': {'regular_sleeping': 1}, 'actors': [
            {'name': '/World/rigid_2_base', 'sleeping': True},
            {'name': 'rigid_3_base', 'sleeping': False},
            {'name': 'rigid_2', 'sleeping': None}, {'name': None, 'sleeping': None}]}
    monkeypatch.setattr(verification, 'inspect_scene_activity', inspect, raising=False)
    assert runtime.diagnostics()['performance']['counters']['worker.steps'] == 17
    assert calls == []
    result = runtime.diagnostics({'activity': True})
    rows = result['scene_activity']['actors']
    assert rows[0]['manifest_label'] == 'pin_x/pin' and rows[1]['manifest_body_id'] == 3
    assert all('manifest_body_id' not in row for row in rows[2:])
    assert result['scene_activity']['manifest_mapped_actor_rows'] == 2
    assert result['performance']['counters']['worker.steps'] == 17
    assert len(calls) == 1
    assert runtime.diagnostics({'reset': True})['performance']['counters']['worker.steps'] == 17
    assert runtime.diagnostics()['performance']['counters'] == {}
    assert len(calls) == 1


@pytest.mark.parametrize('payload', [{'activity': 1}, {'reset': 'yes'}, {'unknown': True}, []])
def test_invalid_activity_requests_are_rejected_before_native_access(payload):
    with pytest.raises(ValueError, match='boolean'):
        _Runtime.__new__(_Runtime).diagnostics(payload)


@pytest.mark.parametrize('setting,expected', [(None, 2), ('0', 0), ('1', 1), ('2', 2)])
def test_explicit_thread_override_preserves_default_two(monkeypatch, setting, expected):
    if setting is None:
        monkeypatch.delenv('EERSIM_PHYSX_THREADS', raising=False)
    else:
        monkeypatch.setenv('EERSIM_PHYSX_THREADS', setting)
    assert diagnostic_thread_count() == expected


@pytest.mark.parametrize('value', ['-1', '3', 'many', ''])
def test_invalid_thread_override_is_not_silently_ignored(monkeypatch, value):
    monkeypatch.setenv('EERSIM_PHYSX_THREADS', value)
    with pytest.raises(ValueError, match='EERSIM_PHYSX_THREADS'):
        diagnostic_thread_count()
