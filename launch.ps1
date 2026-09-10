# Run from any directory. --check diagnoses dependencies without opening the GUI.
$ErrorActionPreference = 'Stop'
$simRoot = $PSScriptRoot
$savedEnvironment = @{}
foreach ($name in @('PATH', 'PYTHONPATH', 'CXX')) {
    $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}
$locationPushed = $false
$exitCode = 1
try {
    $env:PYTHONPATH = $simRoot + [IO.Path]::PathSeparator + $env:PYTHONPATH
    $candidates = @()
    if ($env:EERSIM_PYTHON) {
        $command = Get-Command -Name $env:EERSIM_PYTHON -CommandType Application -ErrorAction SilentlyContinue
        if (-not $command) { throw 'EERSIM_PYTHON does not point to an executable Python. Correct or unset it, then run .\setup.ps1.' }
        $candidates += $command.Source
    } else {
        foreach ($relative in @('.venv/Scripts/python.exe', '.venv/bin/python', '.venv/python.exe', '.sim-tools/physics-env/python.exe', '.sim-tools/physics-env/bin/python')) {
            $candidate = Join-Path $simRoot $relative
            if (Test-Path -LiteralPath $candidate -PathType Leaf) { $candidates += $candidate }
        }
        $command = Get-Command python -CommandType Application -ErrorAction SilentlyContinue
        if ($command -and $command.Source -notmatch '[\\/]Microsoft[\\/]WindowsApps[\\/]') {
            $candidates += $command.Source
        }
    }
    $pythonExecutable = $null
    $diagnostics = @()
    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        # Windows PowerShell represents native stderr as error records. Capture
        # it for diagnosis rather than treating PyBullet's startup banner as fatal.
        try {
            $ErrorActionPreference = 'Continue'
            # Native processes update the global automatic variable. A local
            # sentinel would shadow it when another PowerShell script calls us.
            $global:LASTEXITCODE = 1
            $output = & $candidate (Join-Path $simRoot 'tools/check_environment.py') 2>&1
            $ready = $global:LASTEXITCODE -eq 0
        } catch {
            $output = $_.Exception.Message
            $ready = $false
        } finally {
            $ErrorActionPreference = 'Stop'
        }
        if ($ready) { $pythonExecutable = $candidate; break }
        $diagnostics += ($output | Out-String).Trim()
    }
    if (-not $pythonExecutable) {
        foreach ($diagnostic in $diagnostics) { Write-Host $diagnostic }
        if ($env:EERSIM_PYTHON) { Write-Host 'EERSIM_PYTHON overrides automatic selection. Correct it, or run Remove-Item Env:EERSIM_PYTHON to use the local setup environment.' }
        throw 'No ready EERSim Python environment was found. For a fresh Windows clone, run .\setup.ps1 once, then .\launch.ps1. See README.md for Conda setup on other platforms.'
    }
    Write-Host "Using Python: $pythonExecutable"
    if ($args -contains '--check') {
        Write-Host 'Simulator runtime dependencies are ready.'
        $exitCode = 0
    } else {
        # Respect an explicit compiler or an existing compiler on PATH.
        if (-not $env:CXX -and -not (Get-Command clang++, g++, cl -CommandType Application -ErrorAction SilentlyContinue)) {
            $toolsDirectory = Join-Path $simRoot '.sim-tools'
            if (Test-Path -LiteralPath $toolsDirectory -PathType Container) {
                foreach ($directory in (Get-ChildItem -LiteralPath $toolsDirectory -Directory -Filter 'llvm*' | Sort-Object Name)) {
                    $compiler = Join-Path $directory.FullName 'bin/clang++.exe'
                    if (Test-Path -LiteralPath $compiler -PathType Leaf) {
                        $env:CXX = $compiler
                        $env:PATH = (Split-Path -Parent $compiler) + [IO.Path]::PathSeparator + $env:PATH
                        break
                    }
                }
            }
        }
        Push-Location -LiteralPath $simRoot
        $locationPushed = $true
        & $pythonExecutable -m eersim @args
        $exitCode = $global:LASTEXITCODE
    }
} catch {
    Write-Host ("EERSim: " + $_.Exception.Message) -ForegroundColor Red
} finally {
    if ($locationPushed) { Pop-Location }
    foreach ($name in $savedEnvironment.Keys) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name], 'Process')
    }
}
exit $exitCode
