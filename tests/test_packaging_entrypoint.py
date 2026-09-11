"""Frozen headless failures must terminate without displaying modal dialogs."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest

from eersim import __main__ as cli, gpu_probe, runtime_paths


@pytest.fixture
def entrypoint(monkeypatch):
    source = Path(__file__).resolve().parents[1] / "packaging" / "entrypoint.py"
    spec = importlib.util.spec_from_file_location("eersim_packaging_entrypoint_test", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module.multiprocessing, "freeze_support", lambda: None)
    monkeypatch.setattr(runtime_paths, "ensure_console_streams", lambda: None)
    monkeypatch.setitem(sys.modules, "eersim.gui", None)
    return module


@pytest.mark.parametrize("failure", ["returncode", "exception", "parser_exit"])
@pytest.mark.parametrize("arguments", [["--headless"], ["--renderer", "cpu", "--headless"]])
def test_headless_failure_returns_without_a_dialog(entrypoint, monkeypatch, arguments, failure):
    monkeypatch.setattr(sys, "argv", ["EERSim.exe", *arguments])
    dialogs = []
    monkeypatch.setattr(entrypoint, "_show_failure", dialogs.append)

    def fail():
        if failure == "exception":
            raise RuntimeError("Synthetic startup failure")
        if failure == "parser_exit":
            raise SystemExit(2)
        return 7

    monkeypatch.setattr(cli, "main", fail)
    assert entrypoint.launch() == {"returncode": 7, "exception": 1, "parser_exit": 2}[failure]
    assert dialogs == []


@pytest.mark.parametrize("failure", ["exception", "returncode", "missing_argument"])
def test_probe_failures_do_not_open_dialogs(entrypoint, monkeypatch, failure):
    arguments = ["EERSim.exe", "--gpu-probe-output"]
    if failure != "missing_argument":
        arguments.append("unused-probe.json")
    monkeypatch.setattr(sys, "argv", arguments)
    dialogs = []
    monkeypatch.setattr(entrypoint, "_show_failure", dialogs.append)

    def fail(_path):
        if failure == "exception":
            raise RuntimeError("Synthetic graphics helper failure")
        return 1

    monkeypatch.setattr(gpu_probe, "write_probe_result", fail)
    monkeypatch.setattr(cli, "main", lambda: pytest.fail("Probe must not invoke the application CLI"))
    assert entrypoint.launch() == (2 if failure == "missing_argument" else 1)
    assert dialogs == []


def test_desktop_failure_keeps_its_error_dialog(entrypoint, monkeypatch):
    monkeypatch.setattr(sys, "argv", ["EERSim.exe"])
    monkeypatch.setattr(cli, "main", lambda: 1)
    dialogs = []
    monkeypatch.setattr(entrypoint, "_show_failure", dialogs.append)
    assert entrypoint.launch() == 1
    assert dialogs == [None]
