"""Exercise the Windows launcher with real interpreters and no GUI or installs."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import venv

import pytest

import eersim


pytestmark = pytest.mark.skipif(sys.platform != "win32", reason="Windows PowerShell launcher")
ROOT = Path(__file__).resolve().parents[1]
POWERSHELL = shutil.which("powershell") or shutil.which("pwsh")


@pytest.fixture
def checkout(tmp_path):
    if not POWERSHELL:
        pytest.skip("PowerShell is unavailable")
    root = tmp_path / "simulator checkout with spaces"
    (root / "tools").mkdir(parents=True)
    (root / "eersim").mkdir()
    shutil.copy2(ROOT / "launch.ps1", root / "launch.ps1")
    shutil.copy2(ROOT / "tools" / "check_environment.py", root / "tools" / "check_environment.py")
    # Keep the actual native-library bootstrap. Only the GUI entry point is
    # replaced, so dependency checks still load the real installed packages.
    shutil.copy2(Path(eersim.__file__), root / "eersim" / "__init__.py")
    (root / "eersim" / "__main__.py").write_text(
        "import json, os, pathlib, sys\n"
        "pathlib.Path('main-called').write_text('yes')\n"
        "print('LAUNCH_RESULT ' + json.dumps({\n"
        "    'argv': sys.argv[1:], 'cwd': os.getcwd(), 'python': sys.executable,\n"
        "    'pythonpath': os.environ.get('PYTHONPATH'), 'cxx': os.environ.get('CXX')}))\n"
        "raise SystemExit(int(os.environ.get('EERSIM_TEST_EXIT_CODE', '0')))\n",
        encoding="utf-8",
    )
    # This tripwire makes any accidental automatic installation observable.
    (root / "setup.ps1").write_text("Set-Content -LiteralPath (Join-Path $PSScriptRoot 'setup-called') -Value 'unexpected'\nexit 99\n")
    return root


def environment(**updates):
    result = os.environ.copy()
    for name in ("EERSIM_PYTHON", "PYTHONHOME", "PYTHONPATH", "EERSIM_TEST_EXIT_CODE"):
        result.pop(name, None)
    result["PYTHONDONTWRITEBYTECODE"] = "1"
    result.update(updates)
    return result


def run_script(script, *arguments, env, cwd):
    return subprocess.run(
        [POWERSHELL, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(script), *map(str, arguments)],
        env=env, cwd=cwd, capture_output=True, text=True, encoding="utf-8", errors="replace",
        timeout=60, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )


def output_record(result, prefix):
    records = [line[len(prefix):] for line in result.stdout.splitlines() if line.startswith(prefix)]
    assert len(records) == 1, result.stdout + result.stderr
    return json.loads(records[0])


def make_environment(root, *, ready, prefix_executable=False):
    # A venv with no site packages reproduces a fresh Python missing PySide6.
    # A second venv can share the test environment's real simulator packages.
    venv.EnvBuilder(with_pip=False, system_site_packages=ready).create(root)
    executable = root / "Scripts" / "python.exe"
    if prefix_executable:
        # CPython's Windows redirector supports pyvenv.cfg beside the executable;
        # this gives the launcher the same layout as a local Conda environment.
        target = root / "python.exe"
        shutil.copy2(executable, target)
        executable = target
    return executable


def test_explicit_python_forwards_arguments_and_child_exit_status(checkout, tmp_path):
    arguments = ["--headless", "--workspace", "robot path & literal $HOME `text`", "--seconds", "0.25"]
    env = environment(EERSIM_PYTHON=sys.executable, CXX="caller compiler choice", EERSIM_TEST_EXIT_CODE="23")
    result = run_script(checkout / "launch.ps1", *arguments, env=env, cwd=tmp_path)
    assert result.returncode == 23, result.stdout + result.stderr
    child = output_record(result, "LAUNCH_RESULT ")
    assert child["argv"] == arguments
    assert Path(child["cwd"]) == checkout
    assert Path(child["python"]).resolve() == Path(sys.executable).resolve()
    assert child["cxx"] == env["CXX"]
    assert Path(child["pythonpath"].split(os.pathsep)[0]) == checkout


@pytest.mark.parametrize(("explicit_incomplete", "corrupt_local"), [(False, False), (True, False), (False, True)])
def test_incomplete_local_environment_and_explicit_override(checkout, tmp_path, explicit_incomplete, corrupt_local):
    incomplete = make_environment(checkout / ".venv", ready=False)
    if corrupt_local:
        incomplete.write_bytes(b"MZ")
    ready = make_environment(checkout / ".sim-tools" / "physics-env", ready=True, prefix_executable=True)
    env = environment(**({"EERSIM_PYTHON": str(incomplete)} if explicit_incomplete else {}))
    result = run_script(checkout / "launch.ps1", "--check", env=env, cwd=tmp_path)
    output = result.stdout + result.stderr
    if explicit_incomplete:
        assert result.returncode != 0, output
        assert "PySide6" in output and "setup.ps1" in output
        assert "Using Python:" not in output
    else:
        assert result.returncode == 0, output
        assert f"Using Python: {ready}" in output
        assert "runtime dependencies are ready" in output
    assert not (checkout / "main-called").exists()
    assert not (checkout / "setup-called").exists()


def test_invalid_explicit_interpreter_has_actionable_error(checkout, tmp_path):
    result = run_script(checkout / "launch.ps1", "--check", cwd=tmp_path,
                        env=environment(EERSIM_PYTHON=str(tmp_path / "missing python.exe")))
    output = result.stdout + result.stderr
    assert result.returncode != 0
    assert "EERSIM_PYTHON" in output and "setup.ps1" in output
    assert not (checkout / "main-called").exists()
    assert not (checkout / "setup-called").exists()


def test_unstartable_interpreter_cannot_reuse_a_successful_exit_code(checkout, tmp_path):
    incomplete_executable = tmp_path / "partial python.exe"
    incomplete_executable.write_bytes(b"MZ")
    harness = tmp_path / "previous-success.ps1"
    harness.write_text("param([string]$Launcher)\n$global:LASTEXITCODE = 0\n& $Launcher --check\nexit $LASTEXITCODE\n")
    result = run_script(harness, "-Launcher", checkout / "launch.ps1", cwd=tmp_path,
                        env=environment(EERSIM_PYTHON=str(incomplete_executable)))
    output = result.stdout + result.stderr
    assert result.returncode != 0, output
    assert "partial python.exe" in output
    assert "setup.ps1" in output
    assert "Using Python:" not in output
    assert not (checkout / "setup-called").exists()


@pytest.mark.parametrize("ready", [True, False])
def test_calling_powershell_environment_and_location_are_restored(checkout, tmp_path, ready):
    interpreter = Path(sys.executable) if ready else make_environment(checkout / "incomplete", ready=False)
    harness = tmp_path / "caller.ps1"
    harness.write_text(
        "param([string]$Launcher)\n"
        "$before = @{ location=(Get-Location).Path; path=$env:PATH; pythonpath=$env:PYTHONPATH; cxx=$env:CXX }\n"
        "& $Launcher --headless --seconds 0\n"
        "$code = $LASTEXITCODE\n"
        "$after = @{ location=(Get-Location).Path; path=$env:PATH; pythonpath=$env:PYTHONPATH; cxx=$env:CXX }\n"
        "Write-Output ('CALLER_RESULT ' + (@{ before=$before; after=$after; code=$code } | ConvertTo-Json -Compress))\n"
        "exit 0\n", encoding="utf-8",
    )
    env = environment(EERSIM_PYTHON=str(interpreter), CXX="preserve this compiler", PYTHONPATH=str(tmp_path / "other python modules"))
    result = run_script(harness, "-Launcher", checkout / "launch.ps1", env=env, cwd=tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    caller = output_record(result, "CALLER_RESULT ")
    assert caller["after"] == caller["before"]
    assert caller["code"] == (0 if ready else 1), result.stdout + result.stderr
    assert (checkout / "main-called").exists() == ready
    assert not (checkout / "setup-called").exists()
