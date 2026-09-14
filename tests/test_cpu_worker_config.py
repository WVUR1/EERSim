"""Saved worker requests reach PhysX; invalid requests fail before native startup."""
import pytest

from eersim.config import SimConfig, load_config, save_config
from eersim.physx_worker import diagnostic_thread_count


@pytest.mark.parametrize('workers', [0, 1, 2, 8, 64, 256, 1024])
def test_workers_survive_profile_roundtrip(tmp_path, monkeypatch, workers):
    monkeypatch.delenv('EERSIM_PHYSX_THREADS', raising=False)
    profile = tmp_path / 'experiment.json'
    save_config({'physics_cpu_threads': workers}, profile)
    loaded = load_config(profile)
    assert loaded['physics_cpu_threads'] == workers
    assert diagnostic_thread_count(loaded['physics_cpu_threads']) == workers


@pytest.mark.parametrize('workers', [True, False, -1, 1025, 2.0, '2', None])
def test_invalid_workers_rejected_before_native_work(workers):
    with pytest.raises(ValueError, match='physics_cpu_threads'):
        SimConfig.from_dict({'physics_cpu_threads': workers})
    with pytest.raises(ValueError, match='physics_cpu_threads'):
        diagnostic_thread_count(workers)


def test_environment_override_does_not_rewrite_saved_profile(monkeypatch):
    config = SimConfig.from_dict({'physics_cpu_threads': 16})
    monkeypatch.setenv('EERSIM_PHYSX_THREADS', '0')
    assert diagnostic_thread_count(config.physics_cpu_threads) == 0
    assert config.to_dict()['physics_cpu_threads'] == 16


def test_older_profiles_keep_two_workers(monkeypatch):
    monkeypatch.delenv('EERSIM_PHYSX_THREADS', raising=False)
    assert SimConfig.from_dict({}).physics_cpu_threads == 2
    assert diagnostic_thread_count() == 2
