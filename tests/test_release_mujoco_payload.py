"""A release must contain the pinned MuJoCo and the current application."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from eersim import __version__

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('mujoco_archive_payload', ROOT / 'packaging/inspect_mujoco_archive.py')
archive_payload = importlib.util.module_from_spec(spec)
spec.loader.exec_module(archive_payload)


def payload_files():
    files = {
        'mujoco/mujoco.dll': b'MZ-native-library',
        'mujoco-3.17.0.dist-info/METADATA': b'Name: mujoco\nVersion: 3.17.0\n',
        f'eerkit_simulator-{__version__}.dist-info/METADATA':
            f'Name: eerkit-simulator\nVersion: {__version__}\n'.encode(),
    }
    for module in ('_callbacks', '_constants', '_enums', '_errors', '_functions', '_specs', '_structs'):
        files[f'mujoco/{module}.cp311-win_amd64.pyd'] = b'MZ-python-binding'
    return files


def damaged_payload(problem):
    files = payload_files()
    if problem == 'missing_library':
        del files['mujoco/mujoco.dll']
    elif problem == 'empty_library':
        files['mujoco/mujoco.dll'] = b''
    elif problem == 'missing_binding':
        del files['mujoco/_structs.cp311-win_amd64.pyd']
    elif problem == 'empty_binding':
        files['mujoco/_structs.cp311-win_amd64.pyd'] = b''
    elif problem == 'duplicate_binding':
        files['mujoco/_structs.cp312-win_amd64.pyd'] = b'MZ-another-ABI'
    elif problem == 'wrong_mujoco_version':
        files['mujoco-3.17.0.dist-info/METADATA'] = b'Name: mujoco\nVersion: 3.16.0\n'
    elif problem == 'wrong_app_version':
        files[f'eerkit_simulator-{__version__}.dist-info/METADATA'] = b'Name: eerkit-simulator\nVersion: 0.0.0\n'
    elif problem == 'missing_metadata':
        del files['mujoco-3.17.0.dist-info/METADATA']
    elif problem == 'duplicate_metadata':
        files['mujoco-3.16.0.dist-info/METADATA'] = b'Name: mujoco\nVersion: 3.16.0\n'
    elif problem == 'wrong_package_name':
        files['mujoco-3.17.0.dist-info/METADATA'] = b'Name: different-package\nVersion: 3.17.0\n'
    elif problem in ('egg_metadata', 'wrong_egg_version', 'mixed_app_metadata'):
        name = f'eerkit_simulator-{__version__}.dist-info/METADATA'
        content = files[name]
        if problem != 'mixed_app_metadata':
            del files[name]
        if problem == 'wrong_egg_version':
            content = b'Name: eerkit-simulator\nVersion: 0.0.0\n'
        files['eerkit_simulator.egg-info/PKG-INFO'] = content
    elif problem is not None:
        raise ValueError(problem)
    return files


class Archive:
    def __init__(self, files):
        self.files = {key.replace('/', '\\'): value for key, value in files.items()}
        self.toc = dict.fromkeys(self.files)

    def extract(self, name):
        return self.files[name]


def test_onefile_reports_versions_and_library_hash_from_payload():
    result = archive_payload.inspect_archive(Archive(payload_files()), __version__)
    assert result['includes_mujoco'] is True
    assert result['mujoco_version'] == '3.17.0'
    assert result['application_version'] == __version__
    assert result['mujoco_version_source'] == 'bundled-dist-info'
    assert len(result['mujoco_library_sha256']) == 64
    # A previously valid PhysX-only archive cannot pass the new release guard.
    with pytest.raises(ValueError, match='missing or empty mujoco'):
        archive_payload.inspect_archive(Archive({'ovphysx/lib/ovphysx.dll': b'old GPU bundle'}), __version__)


@pytest.mark.parametrize('problem', ['missing_library', 'empty_library', 'missing_binding', 'empty_binding',
                                     'duplicate_binding', 'wrong_mujoco_version', 'wrong_app_version',
                                     'missing_metadata', 'duplicate_metadata', 'wrong_package_name',
                                     'wrong_egg_version', 'mixed_app_metadata'])
def test_onefile_rejects_missing_stale_and_mixed_runtime_payloads(problem):
    with pytest.raises(ValueError):
        archive_payload.inspect_archive(Archive(damaged_payload(problem)), __version__)


@pytest.mark.parametrize('metadata', [None, 'egg_metadata'])
def test_real_pyinstaller_archive_is_inspected_without_loading_native_code(tmp_path, metadata):
    readers = pytest.importorskip('PyInstaller.archive.readers')
    writers = pytest.importorskip('PyInstaller.archive.writers')
    entries = []
    for index, (name, data) in enumerate(damaged_payload(metadata).items()):
        source = tmp_path / f'input-{index}'
        source.write_bytes(data)
        entries.append((name, str(source), True, 'x'))
    path = tmp_path / 'payload.pkg'
    writers.CArchiveWriter(str(path), entries, 'python311.dll')
    result = archive_payload.inspect_archive(readers.CArchiveReader(path), __version__)
    assert result['includes_mujoco'] is True
    assert result['mujoco_version'] == '3.17.0'


@pytest.mark.skipif(sys.platform != 'win32', reason='Runs the read-only Windows release guard')
@pytest.mark.parametrize('problem', [None, 'egg_metadata', 'missing_library', 'empty_binding', 'duplicate_binding',
                                    'wrong_mujoco_version', 'wrong_app_version', 'duplicate_metadata',
                                    'wrong_egg_version', 'mixed_app_metadata'])
def test_directory_guard_used_by_skip_bundle_build_rejects_stale_payload(tmp_path, problem):
    for relative, data in damaged_payload(problem).items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    script = tmp_path / 'inspect.ps1'
    script.write_text("param($Helper, $Payload, $Expected)\n$ErrorActionPreference='Stop'\n"
                      ". $Helper\nGet-EERSimMuJoCoPayload -PayloadRoot $Payload -ExpectedAppVersion $Expected "
                      "| ConvertTo-Json -Depth 4\n", encoding='utf-8')
    powershell = Path(os.environ['SYSTEMROOT']) / 'System32/WindowsPowerShell/v1.0/powershell.exe'
    result = subprocess.run([str(powershell), '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(script),
                             str(ROOT / 'packaging/check-mujoco-payload.ps1'), str(tmp_path), __version__],
                            capture_output=True, text=True, timeout=20, creationflags=subprocess.CREATE_NO_WINDOW)
    if problem in (None, 'egg_metadata'):
        assert result.returncode == 0, result.stderr
        actual = json.loads(result.stdout)
        assert actual['includes_mujoco'] is True
        assert actual['mujoco_version'] == '3.17.0'
        assert actual['application_version'] == __version__
    else:
        assert result.returncode != 0
        assert 'MuJoCo payload' in result.stderr
