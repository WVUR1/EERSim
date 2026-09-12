"""Missing system CUDA drivers remain a normal, explicit fallback condition."""
from types import SimpleNamespace

import pytest

from eersim import physics_backend, physx_worker


@pytest.mark.parametrize('platform,loader,library', [
    ('win32', 'WinDLL', 'nvcuda.dll'), ('linux', 'CDLL', 'libcuda.so.1')])
@pytest.mark.parametrize('error', [OSError, ImportError])
def test_missing_system_driver_has_actionable_non_bundle_diagnostic(monkeypatch, platform, loader, library, error):
    seen = []

    def unavailable(name):
        seen.append(name)
        raise error('Failed to load dynlib/dll; not found when the application was frozen')

    monkeypatch.setattr(physx_worker, 'sys', SimpleNamespace(platform=platform))
    monkeypatch.setattr(physx_worker.ctypes, loader, unavailable, raising=False)
    result = physx_worker.cuda_device_info(3)
    assert seen == [library]
    assert result['available'] is False and result['ordinal'] == 3
    assert 'NVIDIA CUDA driver' in result['reason'] and library in result['reason']
    assert 'select Bullet physics' in result['reason']
    assert 'frozen' not in result['reason'] and 'bundle' not in result['reason']


def test_loaded_driver_api_error_is_not_misreported_as_missing_driver(monkeypatch):
    monkeypatch.setattr(physx_worker, 'sys', SimpleNamespace(platform='win32'))
    monkeypatch.setattr(physx_worker.ctypes, 'WinDLL', lambda _: SimpleNamespace(cuInit=lambda _: 35), raising=False)
    result = physx_worker.cuda_device_info()
    assert result == {'available': False, 'reason': 'CUDA driver returned error 35', 'ordinal': 0}


def test_auto_falls_back_but_explicit_gpu_preserves_missing_driver_error(monkeypatch):
    def unavailable(_):
        raise ImportError('Simulated frozen dynamic-library loader')

    monkeypatch.setattr(physx_worker, 'sys', SimpleNamespace(platform='win32'))
    monkeypatch.setattr(physx_worker.ctypes, 'WinDLL', unavailable, raising=False)
    monkeypatch.setattr(physics_backend, 'find_spec', lambda _: object())
    monkeypatch.setattr(physics_backend, 'World', lambda values: SimpleNamespace(
        physics_status={'backend': 'bullet', 'hardware_accelerated': False}))
    world = physics_backend.create_world({'physics_backend': 'auto'})
    assert world.physics_status['backend'] == 'bullet'
    assert 'NVIDIA CUDA driver' in world.physics_status['reason']
    with pytest.raises(RuntimeError, match='NVIDIA CUDA driver'):
        physics_backend.create_world({'physics_backend': 'physx_gpu'})
