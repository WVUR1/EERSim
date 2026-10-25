<#
Build a Windows EXE with Python, Qt, Bullet, VTK, assets, and a native compiler.
The default is an unsigned private preview. An existing certificate is optional.
#>
[CmdletBinding()]
param(
    [ValidateSet('OneFile', 'Directory')]
    [string]$Layout = 'OneFile',
    [switch]$SkipArchive,
    [string]$PythonPath,
    [string]$CompilerPath,
    [switch]$WithoutCompiler,
    [string]$CertificateThumbprint,
    [ValidateSet('CurrentUser', 'LocalMachine')]
    [string]$CertificateStore = 'CurrentUser',
    [string]$TimestampServer = 'http://timestamp.digicert.com',
    [string]$SignToolPath
)

$ErrorActionPreference = 'Stop'
$simRoot = $PSScriptRoot
$savedEnvironment = @{}
foreach ($name in @('PATH', 'PYTHONPATH', 'EERSIM_BUNDLE_MODE', 'EERSIM_BUNDLE_COMPILER')) {
    $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}
$locationPushed = $false
$exitCode = 1
try {
    if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or -not [Environment]::Is64BitOperatingSystem) {
        throw 'Build the Windows EXE on 64-bit Windows. Cross-compiling from WSL/Linux is not supported.'
    }
    if ($SkipArchive -and $Layout -ne 'Directory') { throw 'SkipArchive requires -Layout Directory.' }
    if ($WithoutCompiler -and $CompilerPath) { throw 'Use either CompilerPath or WithoutCompiler.' }
    if (-not $PythonPath) { $PythonPath = $env:EERSIM_PYTHON }
    if (-not $PythonPath) {
        foreach ($relative in @('.sim-tools/physics-env/python.exe', '.venv/Scripts/python.exe', '.venv/python.exe')) {
            $candidate = Join-Path $simRoot $relative
            if (Test-Path -LiteralPath $candidate -PathType Leaf) { $PythonPath = $candidate; break }
        }
    }
    if (-not $PythonPath) { $PythonPath = 'python' }
    $python = Get-Command -Name $PythonPath -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if (-not $python) { throw 'Run .\setup.ps1 first, or specify -PythonPath for a complete Windows simulator environment.' }
    $python = $python.Source
    $env:PYTHONPATH = $simRoot + [IO.Path]::PathSeparator + $env:PYTHONPATH
    & $python (Join-Path $simRoot 'tools/check_environment.py')
    if ($LASTEXITCODE -ne 0) { throw 'The build Python is missing simulator dependencies. Run .\setup.ps1, or select the correct -PythonPath.' }

    if (-not $WithoutCompiler) {
        if (-not $CompilerPath) {
            $toolsDirectory = Join-Path $simRoot '.sim-tools'
            if (Test-Path -LiteralPath $toolsDirectory) {
                foreach ($directory in (Get-ChildItem -LiteralPath $toolsDirectory -Directory -Filter 'llvm*' | Sort-Object Name -Descending)) {
                    if (Test-Path -LiteralPath (Join-Path $directory.FullName 'bin/clang++.exe')) { $CompilerPath = $directory.FullName; break }
                }
            }
        }
        if (-not $CompilerPath -or -not (Test-Path -LiteralPath (Join-Path $CompilerPath 'bin/clang++.exe')) -or
            -not (Test-Path -LiteralPath (Join-Path $CompilerPath 'x86_64-w64-mingw32'))) {
            throw 'Supply -CompilerPath for an extracted Windows x86_64 LLVM-MinGW toolchain (see docs/windows-release.md). Use -WithoutCompiler only when target machines provide their own CXX compiler.'
        }
        $CompilerPath = (Resolve-Path -LiteralPath $CompilerPath).Path
    }
    $signing = @{
        CertificateThumbprint = $CertificateThumbprint
        StoreLocation = $CertificateStore
        TimestampServer = $TimestampServer
    }
    if ($SignToolPath) {
        $signCommand = Get-Command -Name $SignToolPath -CommandType Application -ErrorAction Stop | Select-Object -First 1
        $signing.SignToolPath = $signCommand.Source
    }
    $signHelper = Join-Path $simRoot 'packaging/sign-artifact.ps1'
    if ($CertificateThumbprint) {
        & $signHelper @signing -CheckOnly
        if ($LASTEXITCODE -ne 0) { throw 'Signing prerequisites are unavailable. Omit CertificateThumbprint for an unsigned private preview.' }
    } else {
        Write-Host 'Building an UNSIGNED private preview.' -ForegroundColor Yellow
    }

    & $python -m pip install -r (Join-Path $simRoot 'packaging/requirements-build.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Could not install the pinned EXE build tools.' }
    # Isolate pip from checkout egg-info, including the PYTHONPATH set above.
    # Otherwise it may mistake local metadata for the installed distribution
    # and leave an older site-packages dist-info beside the new version.
    Push-Location -LiteralPath ([IO.Path]::GetTempPath())
    try {
        & $python -I -m pip install -e $simRoot --no-deps
        if ($LASTEXITCODE -ne 0) { throw 'Could not refresh application metadata for this checkout.' }
    } finally {
        Pop-Location
    }
    $env:EERSIM_BUNDLE_MODE = if ($Layout -eq 'OneFile') { 'onefile' } else { 'onedir' }
    $env:EERSIM_BUNDLE_COMPILER = if ($WithoutCompiler) { '' } else { $CompilerPath }
    $probeArguments = @('-m', 'eersim.physx_probe_build', '--output', (Join-Path $simRoot 'build/native/physx_scene_probe.dll'),
                        '--cache', (Join-Path $simRoot 'build/native/cache'))
    if ($CompilerPath) { $probeArguments += @('--cxx', (Join-Path $CompilerPath 'bin/clang++.exe')) }
    & $python @probeArguments
    if ($LASTEXITCODE -ne 0) { throw 'Could not build the native GPU verification helper; GPU fallback detection must be included in the release.' }
    $output = Join-Path $simRoot ("dist/windows-" + $env:EERSIM_BUNDLE_MODE)
    $work = Join-Path $simRoot ("build/pyinstaller-" + $env:EERSIM_BUNDLE_MODE)
    Push-Location -LiteralPath $simRoot
    $locationPushed = $true
    & $python -m PyInstaller --noconfirm --distpath $output --workpath $work (Join-Path $simRoot 'packaging/EERSim.spec')
    if ($LASTEXITCODE -ne 0) { throw 'EXE packaging failed. Check the PyInstaller output above.' }
    $executable = if ($Layout -eq 'OneFile') { Join-Path $output 'EERSim.exe' } else { Join-Path $output 'EERSim/EERSim.exe' }
    if (-not (Test-Path -LiteralPath $executable -PathType Leaf)) { throw 'The build did not produce the expected EERSim.exe.' }
    if ($CertificateThumbprint) {
        & $signHelper @signing -FilePath $executable
        if ($LASTEXITCODE -ne 0) { throw 'Signing or signature verification failed. This build is not a signed release.' }
    }
    . (Join-Path $simRoot 'packaging/check-gpu-payload.ps1')
    . (Join-Path $simRoot 'packaging/check-mujoco-payload.ps1')
    $applicationVersion = Get-EERSimSourceVersion -SourceRoot $simRoot
    if ($Layout -eq 'Directory') {
        $gpuPayload = Get-EERSimGpuPayload -PayloadRoot (Join-Path $output 'EERSim/_internal')
        $mujocoPayload = Get-EERSimMuJoCoPayload -PayloadRoot (Join-Path $output 'EERSim/_internal') -ExpectedAppVersion $applicationVersion
    } else {
        # Read the one-file archive directly; never launch an app or GPU probe.
        $gpuJson = & $python (Join-Path $simRoot 'packaging/inspect_gpu_archive.py') $executable
        if ($LASTEXITCODE -ne 0) { throw 'The one-file archive is missing its pinned GPU physics payload.' }
        $gpuPayload = ($gpuJson | Out-String) | ConvertFrom-Json
        $mujocoJson = & $python (Join-Path $simRoot 'packaging/inspect_mujoco_archive.py') $executable $applicationVersion
        if ($LASTEXITCODE -ne 0) { throw 'The one-file archive is missing MuJoCo or does not match this application version.' }
        $mujocoPayload = ($mujocoJson | Out-String) | ConvertFrom-Json
    }
    $artifact = $executable
    if ($Layout -eq 'Directory' -and -not $SkipArchive) {
        $artifact = Join-Path $output 'EERSim-windows-x64.zip'
        for ($archiveAttempt = 1; $archiveAttempt -le 3; $archiveAttempt++) {
            try {
                Compress-Archive -LiteralPath (Join-Path $output 'EERSim') -DestinationPath $artifact -Force -ErrorAction Stop
                break
            } catch {
                if ($archiveAttempt -eq 3) { throw }
                Write-Host "ZIP creation failed (attempt $archiveAttempt of 3): $($_.Exception.Message)"
                Write-Host 'Retrying in two seconds. Close any running app or compiler using the directory bundle.'
                Start-Sleep -Seconds 2
            }
        }
    }
    $hash = (Get-FileHash -LiteralPath $artifact -Algorithm SHA256).Hash.ToLowerInvariant()
    $utf8 = [Text.UTF8Encoding]::new($false)
    [IO.File]::WriteAllText($artifact + '.sha256', ($hash + '  ' + (Split-Path -Leaf $artifact) + "`n"), $utf8)
    $manifest = [ordered]@{
        artifact = if ($Layout -eq 'Directory' -and $SkipArchive) { 'EERSim/EERSim.exe' } else { Split-Path -Leaf $artifact }
        sha256 = $hash
        bytes = (Get-Item -LiteralPath $artifact).Length
        layout = $env:EERSIM_BUNDLE_MODE
        archive = ($Layout -eq 'Directory' -and -not $SkipArchive)
        includes_compiler = -not $WithoutCompiler
        version = $mujocoPayload.application_version
        includes_mujoco = [bool]$mujocoPayload.includes_mujoco
        mujoco_version = $mujocoPayload.mujoco_version
        mujoco_version_source = $mujocoPayload.mujoco_version_source
        mujoco_library_sha256 = $mujocoPayload.mujoco_library_sha256
        includes_gpu_physics = [bool]$gpuPayload.includes_gpu_physics
        gpu_runtime_versions = $gpuPayload.gpu_runtime_versions
        gpu_runtime_versions_source = $gpuPayload.gpu_runtime_versions_source
        gpu_verification_helper_sha256 = $gpuPayload.gpu_verification_helper_sha256
        signed = [bool]$CertificateThumbprint
        signature_status = [string](Get-AuthenticodeSignature -LiteralPath $executable).Status
        built_utc = [DateTime]::UtcNow.ToString('o')
    }
    [IO.File]::WriteAllText((Join-Path $output 'build-manifest.json'), ($manifest | ConvertTo-Json -Depth 5) + "`n", $utf8)
    Write-Host "Build ready: $artifact" -ForegroundColor Green
    Write-Host "SHA256: $hash"
    if (-not $CertificateThumbprint) { Write-Host 'Unsigned preview: Windows does not have a verified publisher identity for this EXE.' }
    $exitCode = 0
} catch {
    Write-Host ("EERSim build: " + $_.Exception.Message) -ForegroundColor Red
} finally {
    if ($locationPushed) { Pop-Location }
    foreach ($name in $savedEnvironment.Keys) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name], 'Process')
    }
}
exit $exitCode
