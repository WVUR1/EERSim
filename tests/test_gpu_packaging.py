"""The Windows bundle keeps the runtime's native layout and lazy imports."""
from pathlib import Path
import runpy
from types import SimpleNamespace

import pytest


@pytest.fixture
def collector():
    helper = Path(__file__).resolve().parents[1] / "packaging" / "gpu_payload.py"
    return runpy.run_path(str(helper))["collect_gpu_payload"]


@pytest.fixture
def packages(tmp_path):
    files = {
        "ovphysx": ("__init__.py", "api.py", "lib/ovphysx.dll", "lib/config.toml",
                    "plugins/gpu/PhysXGpu.dll", "plugins/usd/physx/resources/plugInfo.json",
                    "docs/example.py", "__pycache__/api.cpython-311.pyc"),
        "ovstage": ("__init__.py", "_src/__init__.py", "_src/bindings.py", "LICENSE",
                    "bin/ovstage.dll", "bin/plugins/ov_usd.dll",
                    "bin/plugins/usd/resources/plugInfo.json"),
    }
    versions = {"ovphysx": "0.5.11", "ovstage": "0.1.1.355824"}
    distributions = {}
    for name, paths in files.items():
        for relative in paths:
            file = tmp_path / name / relative
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_text("# fixture payload")
        distributions[name] = SimpleNamespace(version=versions[name], locate_file=lambda name: tmp_path / name)
    return tmp_path, distributions


def test_full_payload_keeps_nested_native_paths_and_license(collector, packages):
    root, distributions = packages
    payload, imports = collector(distributions.__getitem__)
    entries = {target: (Path(source), kind) for target, source, kind in payload}
    assert len(entries) == len(payload)
    for relative in ("ovphysx/plugins/gpu/PhysXGpu.dll",
                     "ovphysx/plugins/usd/physx/resources/plugInfo.json",
                     "ovstage/bin/plugins/ov_usd.dll", "ovstage/LICENSE"):
        assert entries[relative] == (root / relative, "DATA")
    assert not any("__pycache__" in name for name in entries)
    assert "ovphysx.api" in imports
    assert "ovstage._src.bindings" in imports
    assert "ovphysx.docs.example" not in imports
    # Documentation travels with the package without importing its examples.
    assert "ovphysx/docs/example.py" in entries


def test_mismatched_runtime_pair_cannot_produce_a_bundle(collector, packages):
    _, distributions = packages
    distributions["ovstage"].version = "0.1.0"
    with pytest.raises(ValueError, match="ovstage==0.1.1.355824"):
        collector(distributions.__getitem__)


def test_incomplete_gpu_runtime_fails_before_packaging(collector, packages):
    root, distributions = packages
    (root / "ovphysx/lib/ovphysx.dll").unlink()
    with pytest.raises(ValueError, match="missing lib/ovphysx.dll"):
        collector(distributions.__getitem__)
