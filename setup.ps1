# Install a private runtime in this checkout; no Conda/system Python is required.
$ErrorActionPreference = 'Stop'
$simRoot = $PSScriptRoot
$toolsRoot = Join-Path $simRoot '.sim-tools'
$runtimePrefix = Join-Path $toolsRoot 'physics-env'
$mambaRoot = Join-Path $toolsRoot 'mamba'
$mambaDirectory = Join-Path $toolsRoot 'micromamba'
$micromamba = Join-Path $mambaDirectory 'micromamba.exe'
$release = '2.9.0-0'
$expectedHash = 'a6d804394b2418991c4e29562853eaace2f2ce9d9da661a98e74e02e8dbb44b0'
$downloadUrl = "https://github.com/mamba-org/micromamba-releases/releases/download/$release/micromamba-win-64.exe"
$savedPythonPath = $env:PYTHONPATH
$savedProgress = $ProgressPreference
$savedTls = [Net.ServicePointManager]::SecurityProtocol
$exitCode = 1
try {
    if ($env:OS -ne 'Windows_NT' -or -not [Environment]::Is64BitOperatingSystem) {
        throw 'This bootstrap supports 64-bit Windows. Use the Conda commands in README.md on macOS or Linux.'
    }
    New-Item -ItemType Directory -Path $mambaDirectory -Force | Out-Null
    if (-not (Test-Path -LiteralPath $micromamba) -or (Get-FileHash -LiteralPath $micromamba -Algorithm SHA256).Hash -ne $expectedHash) {
        Write-Host "Downloading Micromamba $release from the official mamba-org release..."
        $download = Join-Path $mambaDirectory 'micromamba.download'
        $ProgressPreference = 'SilentlyContinue'
        [Net.ServicePointManager]::SecurityProtocol = $savedTls -bor [Net.SecurityProtocolType]::Tls12
        Invoke-WebRequest -UseBasicParsing -Uri $downloadUrl -OutFile $download
        if ((Get-FileHash -LiteralPath $download -Algorithm SHA256).Hash -ne $expectedHash) {
            throw 'Micromamba download checksum did not match the official release. Rerun setup to download it again.'
        }
        Move-Item -LiteralPath $download -Destination $micromamba -Force
    }
    $environmentFile = Join-Path $simRoot 'environment.yml'
    Write-Host 'Installing Python, PyBullet, PySide6, NumPy, and VTK. The first setup can take several minutes.'
    if (Test-Path -LiteralPath (Join-Path $runtimePrefix 'conda-meta/history')) {
        & $micromamba env update --no-rc --root-prefix $mambaRoot --prefix $runtimePrefix --file $environmentFile --yes
    } else {
        & $micromamba create --no-rc --root-prefix $mambaRoot --prefix $runtimePrefix --file $environmentFile --yes
    }
    if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed (exit $LASTEXITCODE). Check the download/install error above and rerun .\setup.ps1." }
    $python = Join-Path $runtimePrefix 'python.exe'
    & $python -m pip install --no-deps --editable $simRoot
    if ($LASTEXITCODE -ne 0) { throw "Simulator package installation failed (exit $LASTEXITCODE)." }
    $env:PYTHONPATH = $simRoot + [IO.Path]::PathSeparator + $savedPythonPath
    & $python (Join-Path $simRoot 'tools/check_environment.py')
    if ($LASTEXITCODE -ne 0) { throw 'The installed runtime did not pass its dependency check.' }
    Write-Host 'Setup complete. Start the simulator with .\launch.ps1.' -ForegroundColor Green
    Write-Host 'To build PROS workspaces, also install a desktop C++17 compiler and add it to PATH or set CXX (see README.md).'
    if ($env:EERSIM_PYTHON) { Write-Host 'EERSIM_PYTHON is set and overrides this runtime. Unset it to use the environment installed here.' }
    $exitCode = 0
} catch {
    Write-Host ("EERSim setup: " + $_.Exception.Message) -ForegroundColor Red
} finally {
    $env:PYTHONPATH = $savedPythonPath
    $ProgressPreference = $savedProgress
    [Net.ServicePointManager]::SecurityProtocol = $savedTls
}
exit $exitCode
