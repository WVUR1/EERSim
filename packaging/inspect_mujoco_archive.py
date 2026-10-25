"""Inspect MuJoCo and application versions in a frozen Windows archive."""
from email.parser import Parser
import hashlib
import json
from pathlib import Path
import re
import sys


def inspect_archive(archive, expected_app_version):
    """Validate real bundled files without executing the app or loading DLLs."""
    if not expected_app_version:
        raise ValueError('Expected application version is required')
    names = {name.replace('\\', '/'): name for name in archive.toc}
    library = 'mujoco/mujoco.dll'
    if library not in names or not archive.extract(names[library]):
        raise ValueError('Incomplete MuJoCo archive: missing or empty mujoco/mujoco.dll')
    for module in ('_callbacks', '_constants', '_enums', '_errors', '_functions', '_specs', '_structs'):
        pattern = re.compile(r'^mujoco/' + re.escape(module) + r'(\.[^.]+)?\.pyd$')
        candidates = [name for name in names if pattern.fullmatch(name)]
        if len(candidates) != 1 or not archive.extract(names[candidates[0]]):
            raise ValueError(f'Incomplete MuJoCo archive: requires one nonempty {module} Python binding')
    versions = {}
    for package, expected in {'mujoco': '3.17.0', 'eerkit-simulator': expected_app_version}.items():
        prefix = '[-_.]+'.join(package.split('-'))
        suffix = r'-[^/]+\.dist-info/METADATA$'
        if package == 'eerkit-simulator':
            # Editable setuptools installs may collect the checkout's unversioned
            # egg-info/PKG-INFO instead of site-packages dist-info/METADATA.
            suffix = r'(?:-[^/]+\.dist-info/METADATA|(?:-[^/]+)?\.egg-info/PKG-INFO)$'
        pattern = re.compile('^' + prefix + suffix, re.IGNORECASE)
        candidates = [name for name in names if pattern.fullmatch(name)]
        if len(candidates) != 1:
            raise ValueError(f'MuJoCo archive requires exactly one {package} distribution metadata file')
        metadata = Parser().parsestr(archive.extract(names[candidates[0]]).decode('utf-8'))
        name = re.sub(r'[-_.]+', '-', metadata.get('Name', '').lower())
        if name != package or metadata.get('Version') != expected:
            raise ValueError(f'MuJoCo archive requires {package}=={expected}; found {metadata.get("Version")}')
        versions[package] = metadata['Version']
    return {
        'includes_mujoco': True,
        'mujoco_version': versions['mujoco'],
        'mujoco_version_source': 'bundled-dist-info',
        'application_version': versions['eerkit-simulator'],
        'mujoco_library_sha256': hashlib.sha256(archive.extract(names[library])).hexdigest(),
    }


if __name__ == '__main__':
    from PyInstaller.archive.readers import CArchiveReader
    if len(sys.argv) != 3:
        raise SystemExit('Usage: inspect_mujoco_archive.py EERSim.exe EXPECTED_APP_VERSION')
    print(json.dumps(inspect_archive(CArchiveReader(Path(sys.argv[1])), sys.argv[2])))
