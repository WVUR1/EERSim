# Run from any directory. Use configured tools, repository-local tools, or PATH.
$ErrorActionPreference = 'Stop'
$simRoot = $PSScriptRoot
$pythonExecutable = 'python'
if ($env:EERSIM_PYTHON) {
    $pythonExecutable = (Get-Command -Name $env:EERSIM_PYTHON -CommandType Application -ErrorAction Stop).Source
} else {
    foreach ($relativePython in @('.venv/Scripts/python.exe', '.venv/bin/python', '.sim-tools/physics-env/python.exe', '.sim-tools/physics-env/bin/python')) {
        $candidatePython = Join-Path $simRoot $relativePython
        if (Test-Path -LiteralPath $candidatePython -PathType Leaf) {
            $pythonExecutable = $candidatePython
            break
        }
    }
}
# Respect an explicit compiler or an existing compiler on PATH.
if (-not $env:CXX -and -not (Get-Command clang++, g++, cl -CommandType Application -ErrorAction SilentlyContinue)) {
    $toolsDirectory = Join-Path $simRoot '.sim-tools'
    if (Test-Path -LiteralPath $toolsDirectory -PathType Container) {
        $compilerDirectories = Get-ChildItem -LiteralPath $toolsDirectory -Directory -Filter 'llvm*' | Sort-Object Name
        foreach ($compilerDirectory in $compilerDirectories) {
            $candidateCompiler = Join-Path $compilerDirectory.FullName 'bin/clang++.exe'
            if (Test-Path -LiteralPath $candidateCompiler -PathType Leaf) {
                $env:CXX = $candidateCompiler
                $env:PATH = (Split-Path -Parent $candidateCompiler) + [IO.Path]::PathSeparator + $env:PATH
                break
            }
        }
    }
}
$env:PYTHONPATH = $simRoot + [IO.Path]::PathSeparator + $env:PYTHONPATH
Push-Location -LiteralPath $simRoot
try {
    & $pythonExecutable -m eersim @args
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
