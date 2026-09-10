"""Frozen startup must not recurse into Qt or write builds into bundle data."""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest

from eersim import bridge, demos, gpu_probe, runtime_paths


@pytest.fixture(autouse=True)
def clear_gpu_cache():
    gpu_probe.probe_gpu.cache_clear()
    yield
    gpu_probe.probe_gpu.cache_clear()


def test_frozen_probe_uses_private_file_protocol_and_cleans_up(monkeypatch):
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(sys, "executable", "C:/Program Files/EERSim/EERSim.exe")
    observed = []
    expected = {"available": True, "hardware_accelerated": True, "device": "Test GPU"}

    def child(command, **options):
        assert command[:2] == [sys.executable, "--gpu-probe-output"]
        assert len(command) == 3 and options["timeout"] == 25
        path = Path(command[2])
        observed.append(path)
        path.write_text(json.dumps(expected), encoding="utf-8")
        # A windowed executable does not provide console output.
        return subprocess.CompletedProcess(command, 0, stdout=None, stderr=None)

    monkeypatch.setattr(gpu_probe.subprocess, "run", child)
    assert gpu_probe.probe_gpu() == expected
    assert gpu_probe.probe_gpu() == expected
    assert len(observed) == 1 and not observed[0].parent.exists()


@pytest.mark.parametrize("failure", ["crash", "missing", "malformed"])
def test_frozen_probe_failure_becomes_cpu_fallback(monkeypatch, failure):
    monkeypatch.setattr(sys, "frozen", True, raising=False)

    def child(command, **options):
        if failure != "missing":
            payload = [] if failure == "malformed" else {"available": True, "hardware_accelerated": True}
            Path(command[2]).write_text(json.dumps(payload), encoding="utf-8")
        return subprocess.CompletedProcess(command, 17 if failure == "crash" else 0, stdout="", stderr="")

    monkeypatch.setattr(gpu_probe.subprocess, "run", child)
    info = gpu_probe.probe_gpu()
    assert info["available"] is False and info["hardware_accelerated"] is False
    assert info["error"]


def test_source_probe_retains_python_module_protocol(monkeypatch):
    monkeypatch.delattr(sys, "frozen", raising=False)

    def child(command, **options):
        assert command == [sys.executable, "-m", "eersim.gpu_probe"]
        return subprocess.CompletedProcess(command, 0, stdout='driver diagnostic\nEERSIM_GPU {"available": true, "hardware_accelerated": false}\n', stderr="")

    monkeypatch.setattr(gpu_probe.subprocess, "run", child)
    assert gpu_probe.probe_gpu() == {"available": True, "hardware_accelerated": False}


def test_probe_entrypoint_has_no_gui_or_console_dependency(monkeypatch, tmp_path):
    from eersim.__main__ import main

    def broken_driver():
        raise RuntimeError("Driver refused the offscreen context")

    monkeypatch.setattr(gpu_probe, "inspect_context", broken_driver)
    # Importing Qt/the app would fail, making accidental GUI recursion visible.
    monkeypatch.setitem(sys.modules, "eersim.gui", None)
    output = tmp_path / "probe result.json"
    with monkeypatch.context() as context:
        context.setattr(sys, "stdout", None)
        context.setattr(sys, "stderr", None)
        code = main(["--gpu-probe-output", str(output)])
    assert code == 0
    assert json.loads(output.read_text())["error"] == "Driver refused the offscreen context"
    assert main(["--gpu-probe-output"]) == 2
    assert not (tmp_path / "logs").exists()


def test_frozen_demo_is_persistent_and_never_overwrites_edits(monkeypatch, tmp_path):
    bundle = tmp_path / "extracted release"
    source = bundle / "examples" / "claw_demo"
    (source / "src").mkdir(parents=True)
    (source / "include").mkdir()
    (source / "src" / "main.cpp").write_text("original demo")
    (source / "include" / "main.h").write_text("original header")
    (source / ".eersim").mkdir()
    (source / ".eersim" / "old-build.exe").write_text("must not be copied")
    user = tmp_path / "per-user data"
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(demos, "__file__", str(bundle / "eersim" / "demos.py"))
    monkeypatch.setattr(runtime_paths, "user_data_dir", lambda: user)
    workspace = demos._demo_workspace()
    assert workspace == user / "examples" / "claw_demo"
    assert not (workspace / ".eersim").exists()
    assert (workspace / "src" / "main.cpp").read_text() == "original demo"
    (workspace / "src" / "main.cpp").write_text("user-edited autonomous")
    (source / "src" / "new.cpp").write_text("new bundled helper")
    assert demos._demo_workspace() == workspace
    assert (workspace / "src" / "main.cpp").read_text() == "user-edited autonomous"
    assert (workspace / "src" / "new.cpp").read_text() == "new bundled helper"
    assert (source / "src" / "main.cpp").read_text() == "original demo"


def test_bundled_compiler_wins_over_path_but_respects_explicit_override(monkeypatch, tmp_path):
    bundle = tmp_path / "unpacked release"
    compiler = bundle / "compiler" / "bin" / ("clang++.exe" if sys.platform == "win32" else "clang++")
    compiler.parent.mkdir(parents=True)
    compiler.write_bytes(b"compiler fixture")
    monkeypatch.setattr(sys, "frozen", True, raising=False)
    monkeypatch.setattr(bridge, "__file__", str(bundle / "eersim" / "bridge.py"))
    monkeypatch.delenv("CXX", raising=False)
    monkeypatch.setattr(bridge.shutil, "which", lambda value: "system compiler")
    assert bridge._compiler() == str(compiler)
    monkeypatch.setenv("CXX", "explicit compiler")
    monkeypatch.setattr(bridge.shutil, "which", lambda value: value if value == "explicit compiler" else None)
    assert bridge._compiler() == "explicit compiler"


def test_windowed_streams_log_without_replacing_a_real_console(monkeypatch, tmp_path):
    monkeypatch.setattr(runtime_paths, "user_data_dir", lambda: tmp_path)
    original_stdout = sys.stdout
    with monkeypatch.context() as context:
        context.setattr(sys, "stderr", None)
        path = runtime_paths.ensure_console_streams()
        assert sys.stdout is original_stdout
        log_stream = sys.stderr
        print("headless diagnostic", file=log_stream)
        assert runtime_paths.ensure_console_streams() is None
    log_stream.close()
    assert path == tmp_path / "logs" / "eersim.log"
    assert "headless diagnostic" in path.read_text(encoding="utf-8")
