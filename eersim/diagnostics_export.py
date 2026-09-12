"""Export a runnable local diagnostic recipe without starting a simulation."""
from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import tempfile

from .config import SimConfig, save_config
from .runner import MODES


POWERSHELL = r'''<# Run this recipe on the machine where it was exported. No administrator access needed. #>
[CmdletBinding()]
param([string]$Executable)
$ErrorActionPreference = 'Stop'
$recipe = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'launcher.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$program = if ($Executable) { $Executable } else { [string]$recipe.executable }
if (-not (Test-Path -LiteralPath $program -PathType Leaf)) {
    throw 'EERSim was moved or uninstalled. Run again with -Executable pointing to EERSim.exe (or the source Python executable).'
}
$result = Join-Path $PSScriptRoot ('results-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss') + '-' + [Guid]::NewGuid().ToString('N').Substring(0, 6))
$arguments = @($recipe.prefix) + @('--headless', '--diagnostics', '--config', (Join-Path $PSScriptRoot 'experiment.json'), '--diagnostic-output', $result, '--mode', [string]$recipe.mode, '--diagnostic-wall-seconds', [string]$recipe.wall_seconds)
if ($recipe.compare_rates) { $arguments += '--compare-rates' }
if ($recipe.include_rendering) { $arguments += '--measure-rendering' }

# Start-Process joins its arguments. Quote using the Windows argv rules, rather
# than interpreting exported paths or profile values as PowerShell code.
function Quote-NativeArgument([string]$Value) {
    $text = [Text.StringBuilder]::new()
    [void]$text.Append('"')
    $slashes = 0
    foreach ($character in $Value.ToCharArray()) {
        if ($character -eq '\') { $slashes++; continue }
        if ($character -eq '"') {
            [void]$text.Append(('\' * (2 * $slashes + 1)))
        } else {
            [void]$text.Append(('\' * $slashes))
        }
        [void]$text.Append($character)
        $slashes = 0
    }
    [void]$text.Append(('\' * (2 * $slashes)))
    [void]$text.Append('"')
    return $text.ToString()
}
$encoded = ($arguments | ForEach-Object { Quote-NativeArgument ([string]$_) }) -join ' '
$oldPythonPath = $env:PYTHONPATH
$child = $null
try {
    if ($recipe.module_path) { $env:PYTHONPATH = [string]$recipe.module_path + [IO.Path]::PathSeparator + $oldPythonPath }
    Write-Host 'Close other EERSim runs before capture so they do not compete for the GPU.'
    Write-Host ('Collecting diagnostics in ' + $result)
    Write-Host 'Scene loading and compilation have separate time limits. Nothing is uploaded.'
    $child = Start-Process -FilePath $program -ArgumentList $encoded -WindowStyle Hidden -PassThru
    $previous = ''
    while (-not $child.WaitForExit(1000)) {
        $statusPath = Join-Path $result 'progress.json'
        if (Test-Path -LiteralPath $statusPath) {
            try {
                $status = Get-Content -LiteralPath $statusPath -Raw -Encoding UTF8 | ConvertFrom-Json
                if ($status.message -ne $previous) { Write-Host $status.message; $previous = $status.message }
            } catch { } # The writer may be replacing the small progress file.
        }
    }
    $child.Refresh()
    $code = $child.ExitCode
    $archive = Join-Path $result 'EERSim-diagnostics.zip'
    if (Test-Path -LiteralPath $archive) { Write-Host ('Report to send back: ' + $archive) }
    if ($code -ne 0) { throw ('A diagnostic case failed (exit ' + $code + '). The report retains the available evidence; see the results folder.') }
} finally {
    $env:PYTHONPATH = $oldPythonPath
    if ($child -and -not $child.HasExited) {
        # Only the process this script started and its descendants are stopped.
        & (Join-Path $env:SystemRoot 'System32/taskkill.exe') /PID $child.Id /T /F | Out-Null
    }
}
'''

PYTHON_LAUNCHER = '''"""Portable launcher for source installations; Windows installs can use the PS1."""
from pathlib import Path
import json
import os
import subprocess
from datetime import datetime, timezone
import uuid

root = Path(__file__).resolve().parent
recipe = json.loads((root / 'launcher.json').read_text(encoding='utf-8'))
output = root / ('results-' + datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6])
command = [recipe['executable'], *recipe['prefix'], '--headless', '--diagnostics',
           '--config', str(root / 'experiment.json'), '--diagnostic-output', str(output),
           '--mode', recipe['mode'], '--diagnostic-wall-seconds', str(recipe['wall_seconds'])]
if recipe['compare_rates']:
    command.append('--compare-rates')
if recipe['include_rendering']:
    command.append('--measure-rendering')
environment = dict(os.environ)
if recipe.get('module_path'):
    environment['PYTHONPATH'] = recipe['module_path'] + os.pathsep + environment.get('PYTHONPATH', '')
print('Close other EERSim runs first. Nothing is uploaded.', flush=True)
print('Results: ' + str(output), flush=True)
raise SystemExit(subprocess.call(command, env=environment))
'''

README = '''# EERSim performance diagnostics

On Windows, open PowerShell in this folder and run:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\\run-diagnostics.ps1
```

No administrator privileges, Python installation, CUDA toolkit, or profiler
installation is needed when using the installed EERSim application. The script
uses the executable that exported it; `-Executable "path\\to\\EERSim.exe"` can
select a moved installation. Source installations can also run
`python run-diagnostics.py` with their existing simulator environment.

Close other EERSim runs before capture. This recipe uses the current settings
chosen when exporting, including edits that had not yet been applied in the GUI.
It starts each rate from the configured starting scene, not the current moving
robot state. Existing robot source and asset files are read on this machine;
keep them available. This folder is a local recipe, not a portable asset pack.
Driver modes use neutral controller input; Skills Auto runs the selected
workspace's autonomous program. Compilation uses the normal local build cache.

The script compares 240/120 Hz when selected, otherwise the configured rate.
Each case has its own wall-time budget for physics stepping. It can finish
earlier if the competition period ends. Loading, compilation, rendering, and
cleanup are measured separately and add to elapsed time. Optional rendering is
a separate stationary offscreen probe; no interactive window or images are
created. This is a diagnostic capacity measurement, not GUI playback speed or
an accuracy/score acceptance test. Instrumentation introduces some overhead.

Send back **results-.../EERSim-diagnostics.zip**. It contains a readable summary,
per-stage timings, sampled step/progress data, hardware/runtime information,
NVIDIA utilization/clocks/power/temperature when available, and diagnostic logs.
GPU utilization covers the device, including other applications. The report
records actual physics backend selection and fallback, independently of graphics.

The ZIP excludes raw PROS source, robot/field assets, the launcher recipe, and
private compiler output. Diagnostic logs can contain messages printed by the
robot program. Files under results-.../private stay local and are not archived.
Nothing uploads automatically. Failed captures retain partial evidence.
'''


def export_diagnostics(config: dict, destination: Path, *, mode: str = 'skills_auto',
                       wall_seconds: float = 120, compare_rates: bool = True,
                       include_rendering: bool = True) -> Path:
    """Write a unique child folder; importing/exporting creates no world or process."""
    if mode not in MODES:
        raise ValueError('Select a valid competition mode')
    if not math.isfinite(wall_seconds) or not 1 <= wall_seconds <= 600:
        raise ValueError('Diagnostic duration must be between 1 and 600 wall seconds')
    values = SimConfig.from_dict(config).to_dict()
    destination = Path(destination).expanduser().resolve()
    if not destination.is_dir():
        raise ValueError('Select an existing destination folder')
    folder = Path(tempfile.mkdtemp(prefix='EERSim-diagnostics-', dir=destination))
    save_config(values, folder / 'experiment.json')
    frozen = bool(getattr(sys, 'frozen', False))
    recipe = {
        'schema_version': 1, 'executable': str(Path(sys.executable).resolve()),
        'prefix': [] if frozen else ['-m', 'eersim'],
        'module_path': None if frozen else str(Path(__file__).resolve().parent.parent),
        'mode': mode, 'wall_seconds': float(wall_seconds),
        'compare_rates': bool(compare_rates), 'include_rendering': bool(include_rendering),
    }
    (folder / 'launcher.json').write_text(json.dumps(recipe, indent=2) + '\n', encoding='utf-8')
    # BOM keeps Windows PowerShell5's reading of UTF-8 deterministic.
    (folder / 'run-diagnostics.ps1').write_text(POWERSHELL, encoding='utf-8-sig')
    (folder / 'run-diagnostics.py').write_text(PYTHON_LAUNCHER, encoding='utf-8')
    (folder / 'README.md').write_text(README, encoding='utf-8')
    return folder
