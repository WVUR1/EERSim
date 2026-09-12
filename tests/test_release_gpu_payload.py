"""Release guards reject an old CPU-only payload before installer compilation."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('archive_payload', ROOT / 'packaging/inspect_gpu_archive.py')
archive_payload = importlib.util.module_from_spec(spec)
spec.loader.exec_module(archive_payload)


def payload_files():
    return {'ovphysx/lib/ovphysx.dll': b'MZ-runtime', 'ovstage/bin/ovstage.dll': b'MZ-stage',
            'eersim/_native/physx_scene_probe.dll': b'MZ-verification',
            'ovphysx-0.5.11.dist-info/METADATA': b'Name: ovphysx\nVersion: 0.5.11\n',
            'ovstage-0.1.1.355824.dist-info/METADATA': b'Name: ovstage\nVersion: 0.1.1.355824\n'}


class Archive:
    def __init__(self, files):
        self.files = {key.replace('/', '\\'): value for key, value in files.items()}
        self.toc = dict.fromkeys(self.files)

    def extract(self, name):
        return self.files[name]


def test_onefile_metadata_is_read_from_archive_and_old_cpu_payload_fails():
    result = archive_payload.inspect_archive(Archive(payload_files()))
    assert result['includes_gpu_physics']
    assert result['gpu_runtime_versions'] == {'ovphysx': '0.5.11', 'ovstage': '0.1.1.355824'}
    assert len(result['gpu_verification_helper_sha256']) == 64
    with pytest.raises(ValueError, match='missing ovphysx'):
        archive_payload.inspect_archive(Archive({}))


def test_inspector_reads_real_pyinstaller_archive_without_executing_it(tmp_path):
    readers = pytest.importorskip('PyInstaller.archive.readers')
    writers = pytest.importorskip('PyInstaller.archive.writers')
    entries = []
    for index, (name, data) in enumerate(payload_files().items()):
        source = tmp_path / f'input-{index}'
        source.write_bytes(data)
        entries.append((name, str(source), True, 'x'))
    path = tmp_path / 'payload.pkg'
    writers.CArchiveWriter(str(path), entries, 'python311.dll')
    result = archive_payload.inspect_archive(readers.CArchiveReader(path))
    assert result['includes_gpu_physics'] is True
    assert result['gpu_runtime_versions']['ovphysx'] == '0.5.11'


@pytest.mark.parametrize('problem', ['missing_probe', 'wrong_version', 'duplicate_metadata', 'empty_probe'])
def test_onefile_rejects_incomplete_or_unpinned_gpu_runtime(problem):
    files = payload_files()
    if problem == 'missing_probe':
        del files['eersim/_native/physx_scene_probe.dll']
    elif problem == 'wrong_version':
        files['ovphysx-0.5.11.dist-info/METADATA'] = b'Name: ovphysx\nVersion: 0.4.0\n'
    elif problem == 'duplicate_metadata':
        files['ovphysx-0.4.0.dist-info/METADATA'] = b'Name: ovphysx\nVersion: 0.4.0\n'
    else:
        files['eersim/_native/physx_scene_probe.dll'] = b''
    with pytest.raises(ValueError):
        archive_payload.inspect_archive(Archive(files))


@pytest.mark.skipif(sys.platform != 'win32', reason='Runs only a read-only PowerShell payload validator')
@pytest.mark.parametrize('problem', [None, 'missing_runtime', 'missing_probe', 'wrong_version', 'missing_metadata'])
def test_directory_release_guard_checks_actual_payload_without_launching_it(tmp_path, problem):
    files = payload_files()
    if problem == 'missing_runtime':
        del files['ovphysx/lib/ovphysx.dll']
    elif problem == 'missing_probe':
        del files['eersim/_native/physx_scene_probe.dll']
    elif problem == 'wrong_version':
        files['ovstage-0.1.1.355824.dist-info/METADATA'] = b'Name: ovstage\nVersion: 0.1.0\n'
    elif problem == 'missing_metadata':
        del files['ovphysx-0.5.11.dist-info/METADATA']
    for relative, data in files.items():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    script = tmp_path / 'validate.ps1'
    script.write_text("param($Helper, $Payload)\n$ErrorActionPreference='Stop'\n"
                      ". $Helper\nGet-EERSimGpuPayload -PayloadRoot $Payload | ConvertTo-Json -Depth 4\n", encoding='utf-8')
    powershell = Path(os.environ['SYSTEMROOT']) / 'System32/WindowsPowerShell/v1.0/powershell.exe'
    result = subprocess.run([str(powershell), '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', str(script),
                             str(ROOT / 'packaging/check-gpu-payload.ps1'), str(tmp_path)],
                            capture_output=True, text=True, timeout=20, creationflags=subprocess.CREATE_NO_WINDOW)
    if problem is None:
        assert result.returncode == 0, result.stderr
        actual = json.loads(result.stdout)
        assert actual['includes_gpu_physics'] is True
        assert actual['gpu_runtime_versions']['ovstage'] == '0.1.1.355824'
    else:
        assert result.returncode != 0
        assert 'GPU physics payload' in result.stderr
