"""Validate the actual one-file payload without extracting or executing the app."""
from email.parser import Parser
import hashlib
import json
from pathlib import Path
import sys


def inspect_archive(archive):
    required = ('ovphysx/lib/ovphysx.dll', 'ovstage/bin/ovstage.dll',
                'eersim/_native/physx_scene_probe.dll')
    # PyInstaller stores Windows archive separators as backslashes.
    names = {name.replace('\\', '/'): name for name in archive.toc}
    for name in required:
        if name not in names:
            raise ValueError(f'Incomplete GPU physics archive: missing {name}')
    versions = {}
    for package, expected in {'ovphysx': '0.5.11', 'ovstage': '0.1.1.355824'}.items():
        candidates = [name for name in names
                      if name.startswith(package + '-') and name.endswith('.dist-info/METADATA')
                      and name.count('/') == 1]
        if len(candidates) != 1:
            raise ValueError(f'GPU archive requires exactly one {package} METADATA file')
        metadata = Parser().parsestr(archive.extract(names[candidates[0]]).decode('utf-8'))
        if metadata.get('Name', '').lower() != package or metadata.get('Version') != expected:
            raise ValueError(f'GPU archive requires {package}=={expected}; found {metadata.get("Version")}')
        versions[package] = metadata['Version']
    helper = archive.extract(names[required[-1]])
    if not helper:
        raise ValueError('GPU verification helper is empty')
    return {'includes_gpu_physics': True, 'gpu_runtime_versions': versions,
            'gpu_runtime_versions_source': 'bundled-dist-info',
            'gpu_verification_helper_sha256': hashlib.sha256(helper).hexdigest()}


if __name__ == '__main__':
    from PyInstaller.archive.readers import CArchiveReader
    if len(sys.argv) != 2:
        raise SystemExit('Usage: inspect_gpu_archive.py EERSim.exe')
    print(json.dumps(inspect_archive(CArchiveReader(Path(sys.argv[1])))))
