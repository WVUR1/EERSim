<# Build the per-user Windows installer. The private preview is unsigned by default. #>
[CmdletBinding()]
param(
    [switch]$SkipBundleBuild,
    [string]$BundleDirectory,
    [string]$InnoSetupCompiler,
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
$exitCode = 1

function Get-InnoCompiler {
    $candidates = @()
    if ($InnoSetupCompiler) {
        $explicit = Get-Command -Name $InnoSetupCompiler -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $explicit) { throw 'InnoSetupCompiler must identify ISCC.exe from Inno Setup 7.1 or newer.' }
        $candidates = @($explicit.Source)
    } else {
        $command = Get-Command ISCC.exe -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if ($command) { $candidates += $command.Source }
        $candidates += Join-Path $simRoot '.sim-tools/inno-setup/ISCC.exe'
        foreach ($registryRoot in @('HKCU:\SOFTWARE', 'HKLM:\SOFTWARE', 'HKLM:\SOFTWARE\WOW6432Node')) {
            foreach ($major in @(7, 6)) {
                $key = "$registryRoot\Microsoft\Windows\CurrentVersion\Uninstall\Inno Setup ${major}_is1"
                $installed = Get-ItemProperty -LiteralPath $key -ErrorAction SilentlyContinue
                if ($installed -and $installed.InstallLocation) {
                    $candidates += Join-Path $installed.InstallLocation 'ISCC.exe'
                }
            }
        }
    }
    foreach ($candidate in ($candidates | Select-Object -Unique)) {
        if (-not (Test-Path -LiteralPath $candidate -PathType Leaf)) { continue }
        try {
            $global:LASTEXITCODE = 1
            $versionOutput = (& $candidate --version 2>&1 | Out-String).Trim()
            if ($global:LASTEXITCODE -eq 0 -and $versionOutput -match '(?m)^(\d+\.\d+(?:\.\d+){0,2})\s*$' -and
                [version]$Matches[1] -ge [version]'7.1') {
                return @{ Path = (Resolve-Path -LiteralPath $candidate).Path; Version = $Matches[1] }
            }
        } catch {
            if ($InnoSetupCompiler) { throw "Cannot run the selected Inno Setup compiler: $($_.Exception.Message)" }
        }
        if ($InnoSetupCompiler) { throw 'The selected ISCC.exe must be Inno Setup 7.1 or newer.' }
    }
    throw 'Install Inno Setup 7.1 or newer from https://jrsoftware.org/isdl.php, or supply -InnoSetupCompiler with ISCC.exe. See docs/windows-release.md.'
}

function Quote-InnoSignArgument([string]$Value) {
    if ($Value -match '["\r\n]') { throw 'Signing arguments cannot contain quotation marks or newlines.' }
    # Inno expands $q to a quote and $$ to a literal dollar; $f is already quoted.
    return '$q' + $Value.Replace('$', '$$') + '$q'
}

try {
    if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT -or -not [Environment]::Is64BitOperatingSystem) {
        throw 'Build the installer on 64-bit Windows.'
    }
    if ($BundleDirectory -and -not $SkipBundleBuild) { throw 'BundleDirectory requires -SkipBundleBuild.' }
    if ($WithoutCompiler -and $CompilerPath) { throw 'Use either CompilerPath or WithoutCompiler.' }
    if ($SkipBundleBuild -and ($PythonPath -or $CompilerPath)) { throw 'PythonPath and CompilerPath only apply when building the bundle; omit them with SkipBundleBuild.' }
    $inno = Get-InnoCompiler
    Write-Host ("Using Inno Setup " + $inno.Version)
    $signHelper = Join-Path $simRoot 'packaging/sign-artifact.ps1'
    $signing = @{
        CertificateThumbprint = $CertificateThumbprint
        StoreLocation = $CertificateStore
        TimestampServer = $TimestampServer
    }
    if ($SignToolPath) {
        $signCommand = Get-Command -Name $SignToolPath -CommandType Application -ErrorAction Stop | Select-Object -First 1
        $signing.SignToolPath = $signCommand.Source
    }
    if ($CertificateThumbprint) {
        & $signHelper @signing -CheckOnly
        if ($global:LASTEXITCODE -ne 0) { throw 'Signing prerequisites are unavailable. Omit CertificateThumbprint for an unsigned private preview.' }
    } else {
        Write-Host 'Building an UNSIGNED private installer.' -ForegroundColor Yellow
    }

    if (-not $SkipBundleBuild) {
        $buildOptions = @{ Layout = 'Directory'; SkipArchive = $true; WithoutCompiler = $WithoutCompiler }
        if ($PythonPath) { $buildOptions.PythonPath = $PythonPath }
        if ($CompilerPath) { $buildOptions.CompilerPath = $CompilerPath }
        if ($CertificateThumbprint) {
            $buildOptions.CertificateThumbprint = $CertificateThumbprint
            $buildOptions.CertificateStore = $CertificateStore
            $buildOptions.TimestampServer = $TimestampServer
            if ($signing.SignToolPath) { $buildOptions.SignToolPath = $signing.SignToolPath }
        }
        & (Join-Path $simRoot 'build.ps1') @buildOptions
        if ($global:LASTEXITCODE -ne 0) { throw 'Directory bundle build failed; the installer was not compiled.' }
    }
    if (-not $BundleDirectory) { $BundleDirectory = Join-Path $simRoot 'dist/windows-onedir/EERSim' }
    if (-not (Test-Path -LiteralPath $BundleDirectory -PathType Container)) { throw 'BundleDirectory must contain a completed EERSim directory bundle.' }
    $bundle = (Resolve-Path -LiteralPath $BundleDirectory).Path
    if ($bundle -match '["\r\n]') { throw 'BundleDirectory cannot contain quotation marks or newlines.' }
    foreach ($relative in @('EERSim.exe', '_internal/base_library.zip', '_internal/native/runtime.cpp',
                            '_internal/assets/games/override-2026-27.json', '_internal/assets/robots/xdrive.urdf',
                            '_internal/examples/claw_demo/src/main.cpp')) {
        if (-not (Test-Path -LiteralPath (Join-Path $bundle $relative) -PathType Leaf)) {
            throw "Incomplete directory bundle: missing $relative. Rebuild with .\build.ps1 -Layout Directory -SkipArchive."
        }
    }
    # Also applies to -SkipBundleBuild: an old CPU-only bundle cannot become a GPU release.
    . (Join-Path $simRoot 'packaging/check-gpu-payload.ps1')
    $gpuPayload = Get-EERSimGpuPayload -PayloadRoot (Join-Path $bundle '_internal')
    $includesCompiler = (Test-Path -LiteralPath (Join-Path $bundle '_internal/compiler/bin/clang++.exe') -PathType Leaf) -and
        (Test-Path -LiteralPath (Join-Path $bundle '_internal/compiler/x86_64-w64-mingw32') -PathType Container)
    if (-not $includesCompiler -and -not $WithoutCompiler) { throw 'The bundle is missing the native compiler. Rebuild it, or explicitly use -WithoutCompiler for a package requiring an external CXX compiler.' }
    $application = Join-Path $bundle 'EERSim.exe'
    if ($CertificateThumbprint -and $SkipBundleBuild) {
        & $signHelper @signing -FilePath $application
        if ($global:LASTEXITCODE -ne 0) { throw 'Application signing failed; the installer was not compiled.' }
    }

    $output = Join-Path $simRoot 'dist/windows-installer'
    New-Item -ItemType Directory -Path $output -Force | Out-Null
    $compileArguments = @('--no-ide-signtools', '--quiet-progress', "--define=BundleRoot=$bundle",
                          "--output-dir=$output", '--output-filename=EERSim-Setup-x64')
    if ($CertificateThumbprint) {
        $shellName = if ($PSVersionTable.PSEdition -eq 'Core') { 'pwsh.exe' } else { 'powershell.exe' }
        $callback = (Quote-InnoSignArgument (Join-Path $PSHOME $shellName)) +
            ' -NoProfile -ExecutionPolicy Bypass -File ' + (Quote-InnoSignArgument $signHelper) +
            ' -CertificateThumbprint ' + (Quote-InnoSignArgument $CertificateThumbprint) +
            ' -StoreLocation ' + $CertificateStore +
            ' -TimestampServer ' + (Quote-InnoSignArgument $TimestampServer)
        if ($signing.SignToolPath) { $callback += ' -SignToolPath ' + (Quote-InnoSignArgument $signing.SignToolPath) }
        $callback += ' -FilePath $f'
        $compileArguments += @('--define=SignInstaller=1', "--signtool=eersimrelease=$callback")
    } else {
        $compileArguments += '--no-signing'
    }
    $compileArguments += Join-Path $simRoot 'packaging/EERSim.iss'
    $global:LASTEXITCODE = 1
    & $inno.Path @compileArguments
    if ($global:LASTEXITCODE -ne 0) { throw 'Inno Setup compilation failed. Inspect the compiler output above.' }
    $artifact = Join-Path $output 'EERSim-Setup-x64.exe'
    if (-not (Test-Path -LiteralPath $artifact -PathType Leaf)) { throw 'Inno Setup did not produce EERSim-Setup-x64.exe.' }
    $signature = Get-AuthenticodeSignature -LiteralPath $artifact
    if ($CertificateThumbprint) {
        $expectedThumbprint = $CertificateThumbprint -replace '[\s\u200e\u200f]', ''
        if ($signature.Status -ne 'Valid' -or -not $signature.SignerCertificate -or
            $signature.SignerCertificate.Thumbprint -ne $expectedThumbprint -or -not $signature.TimeStamperCertificate) {
            throw 'The installer signature, selected signer, or timestamp is invalid; this is not a signed release.'
        }
    }
    $hash = (Get-FileHash -LiteralPath $artifact -Algorithm SHA256).Hash.ToLowerInvariant()
    $manifest = [ordered]@{
        artifact = 'EERSim-Setup-x64.exe'
        sha256 = $hash
        bytes = (Get-Item -LiteralPath $artifact).Length
        format = 'windows-installer'
        version = '0.1.9'
        installer_compiler_version = $inno.Version
        install_scope = 'current_user'
        includes_compiler = [bool]$includesCompiler
        includes_gpu_physics = [bool]$gpuPayload.includes_gpu_physics
        gpu_runtime_versions = $gpuPayload.gpu_runtime_versions
        gpu_runtime_versions_source = $gpuPayload.gpu_runtime_versions_source
        gpu_verification_helper_sha256 = $gpuPayload.gpu_verification_helper_sha256
        signed = [bool]$CertificateThumbprint
        signature_status = [string]$signature.Status
        uninstaller_signed = [bool]$CertificateThumbprint
        bundled_app_sha256 = (Get-FileHash -LiteralPath $application -Algorithm SHA256).Hash.ToLowerInvariant()
        bundled_app_signature_status = [string](Get-AuthenticodeSignature -LiteralPath $application).Status
        built_utc = [DateTime]::UtcNow.ToString('o')
    }
    $utf8 = [Text.UTF8Encoding]::new($false)
    [IO.File]::WriteAllText($artifact + '.sha256', ($hash + '  EERSim-Setup-x64.exe' + "`n"), $utf8)
    [IO.File]::WriteAllText((Join-Path $output 'build-manifest.json'), ($manifest | ConvertTo-Json -Depth 5) + "`n", $utf8)
    Write-Host "Installer ready: $artifact" -ForegroundColor Green
    Write-Host "SHA256: $hash"
    if (-not $CertificateThumbprint) { Write-Host 'Unsigned preview: Windows does not have a verified publisher identity for this installer.' }
    $exitCode = 0
} catch {
    Write-Host ("EERSim installer build: " + $_.Exception.Message) -ForegroundColor Red
}
exit $exitCode
