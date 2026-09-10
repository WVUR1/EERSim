"""Check native signing-tool exit handling without using a signing certificate."""
from pathlib import Path
import os
import shutil
import subprocess
import sys

import pytest


pytestmark = pytest.mark.skipif(sys.platform != "win32", reason="Windows signing helper")
ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("exit_code", [0, 1, 2])
def test_signing_native_success_failure_and_warning_exit_codes(tmp_path, exit_code):
    powershell = shutil.which("powershell") or shutil.which("pwsh")
    if not powershell:
        pytest.skip("PowerShell is unavailable")
    harness = tmp_path / "signing status harness.ps1"
    harness.write_text(
        "param([string]$Helper, [string]$Interpreter, [int]$ExitCode)\n"
        "$ErrorActionPreference = 'Stop'\n"
        "$tokens = $null; $errors = $null\n"
        "$ast = [System.Management.Automation.Language.Parser]::ParseFile($Helper, [ref]$tokens, [ref]$errors)\n"
        "if ($errors.Count) { throw 'Signing helper did not parse' }\n"
        "$definition = $ast.Find({ param($node) $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Invoke-SignToolChecked' }, $true)\n"
        "if (-not $definition) { throw 'Signing command helper was not found' }\n"
        "Set-Item Function:\\Invoke-SignToolChecked -Value ($definition.Body.GetScriptBlock())\n"
        "$script:resolvedSignTool = $Interpreter\n"
        "$global:LASTEXITCODE = 77\n"
        "$rejected = $false\n"
        "try {\n"
        "    $program = \"import sys; sys.stderr.write('native tool diagnostic\\n'); sys.exit($ExitCode)\"\n"
        "    Invoke-SignToolChecked -Arguments @('-c', $program) -Operation 'Native status test'\n"
        "} catch { $rejected = $true; Write-Output $_.Exception.Message }\n"
        "if ($rejected -ne ($ExitCode -ne 0)) { throw \"Incorrect signing-tool rejection for exit $ExitCode\" }\n"
        "Write-Output \"SIGNING_STATUS_OK $ExitCode\"\n"
        "exit 0\n", encoding="utf-8",
    )
    result = subprocess.run(
        [powershell, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(harness),
         "-Helper", str(ROOT / "packaging" / "sign-artifact.ps1"),
         "-Interpreter", sys.executable, "-ExitCode", str(exit_code)],
        env=os.environ.copy(), capture_output=True, text=True, encoding="utf-8", errors="replace",
        timeout=30, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert f"SIGNING_STATUS_OK {exit_code}" in result.stdout
