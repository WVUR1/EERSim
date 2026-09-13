<# Same bundled claw scene and PROS program; only solver iterations differ. #>
[CmdletBinding()]
param(
    [string]$Executable,
    [ValidateRange(1, 600)][int]$WallSeconds = 120,
    [string]$OutputDirectory = $PSScriptRoot,
    [ValidateSet(120, 240)][int]$PhysicsHz = 120,
    [ValidateSet('physx_gpu', 'physx_cpu')][string]$PhysicsBackend = 'physx_gpu',
    [ValidateRange(0, 32)][int]$PhysicsDevice = 0,
    [ValidateRange(0, 1000000)][int]$MaxSteps = 0,
    [switch]$PrepareOnly
)
$ErrorActionPreference = 'Stop'

function Write-JsonFile($Value, [string]$Path) {
    $text = $Value | ConvertTo-Json -Depth 40
    [IO.File]::WriteAllText($Path, $text + [Environment]::NewLine, [Text.UTF8Encoding]::new($false))
}
function Resolve-EERSim {
    if ($Executable) {
        return (Resolve-Path -LiteralPath $Executable -ErrorAction Stop).ProviderPath
    }
    $candidates = [Collections.Generic.List[string]]::new()
    $key = 'Software\Microsoft\Windows\CurrentVersion\Uninstall\{95A78754-70D4-4D44-8E83-9241A32DDA25}_is1'
    foreach ($hive in @('HKEY_CURRENT_USER', 'HKEY_LOCAL_MACHINE')) {
        $entry = Get-ItemProperty -LiteralPath ('Registry::' + $hive + '\' + $key) -ErrorAction SilentlyContinue
        if ($entry.InstallLocation) { $candidates.Add((Join-Path $entry.InstallLocation 'EERSim.exe')) }
        if ($entry.'Inno Setup: App Path') { $candidates.Add((Join-Path $entry.'Inno Setup: App Path' 'EERSim.exe')) }
    }
    if ($env:LOCALAPPDATA) { $candidates.Add((Join-Path $env:LOCALAPPDATA 'Programs\EERSim\EERSim.exe')) }
    $candidates.Add((Join-Path $PSScriptRoot 'EERSim.exe'))
    $command = Get-Command EERSim.exe -CommandType Application -ErrorAction SilentlyContinue
    if ($command) { $candidates.Add($command.Source) }
    foreach ($candidate in $candidates | Select-Object -Unique) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            $file = Get-Item -LiteralPath $candidate
            if ($file.VersionInfo.ProductVersion -match '^0\.1\.3(?:\.|$)') { return $file.FullName }
        }
    }
    throw 'EERSim 0.1.3 was not found. Supply -Executable "path\to\EERSim.exe" from the installed app or extracted directory ZIP.'
}
function Quote-NativeArgument([string]$Value) {
    $text = [Text.StringBuilder]::new()
    [void]$text.Append('"')
    $slashes = 0
    foreach ($character in $Value.ToCharArray()) {
        if ($character -eq '\') { $slashes++; continue }
        if ($character -eq '"') { [void]$text.Append(('\' * (2 * $slashes + 1))) }
        else { [void]$text.Append(('\' * $slashes)) }
        [void]$text.Append($character)
        $slashes = 0
    }
    [void]$text.Append(('\' * (2 * $slashes)))
    [void]$text.Append('"')
    return $text.ToString()
}
function Get-SourceHashes([string]$Directory) {
    $list = @()
    foreach ($folder in @('include', 'src')) {
        $base = Join-Path $Directory $folder
        foreach ($file in Get-ChildItem -LiteralPath $base -File -Recurse | Sort-Object FullName) {
            $relative = $file.FullName.Substring($Directory.Length).TrimStart('\', '/').Replace('\', '/')
            $list += [ordered]@{name=$relative; sha256=(Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
        }
    }
    return ,$list
}
function Get-AssetHashes([string]$Directory) {
    $list = @()
    foreach ($file in Get-ChildItem -LiteralPath $Directory -File -Recurse | Sort-Object FullName) {
        $relative = $file.FullName.Substring($Directory.Length).TrimStart('\', '/').Replace('\', '/')
        $list += [ordered]@{name=$relative; sha256=(Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()}
    }
    return ,$list
}
function Same-Hashes($Left, $Right) {
    return (($Left | ConvertTo-Json -Depth 8 -Compress) -ceq ($Right | ConvertTo-Json -Depth 8 -Compress))
}
function Open-ReportStream([string]$Path) {
    for ($attempt = 0; $attempt -lt 40; $attempt++) {
        try {
            return [IO.File]::Open($Path, [IO.FileMode]::Open, [IO.FileAccess]::Read,
                ([IO.FileShare]::ReadWrite -bor [IO.FileShare]::Delete))
        } catch {
            $code = $_.Exception.GetBaseException().HResult -band 0xFFFF
            if ($code -notin @(32, 33) -or $attempt -eq 39) { throw }
            Start-Sleep -Milliseconds 250
        }
    }
}
function Write-ReportArchive([string]$Directory, [string]$Destination) {
    Add-Type -AssemblyName System.IO.Compression
    # CreateNew/Move never replace an existing report. An interrupted archive
    # stays visibly .partial; complete source reports remain untouched.
    $partial = Join-Path ([IO.Path]::GetDirectoryName($Destination)) ('.report-' + [Guid]::NewGuid().ToString('N') + '.partial')
    $outputStream = $zip = $null
    try {
        $outputStream = [IO.File]::Open($partial, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::None)
        $zip = [IO.Compression.ZipArchive]::new($outputStream, [IO.Compression.ZipArchiveMode]::Create, $true)
        foreach ($name in @('comparison.json', 'iterations-100.zip', 'iterations-32.zip')) {
            $path = Join-Path $Directory $name
            if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { continue }
            $inputStream = $entryStream = $null
            try {
                $inputStream = Open-ReportStream $path
                $entry = $zip.CreateEntry($name, [IO.Compression.CompressionLevel]::Optimal)
                $entryStream = $entry.Open()
                $inputStream.CopyTo($entryStream)
            } finally {
                if ($entryStream) { $entryStream.Dispose() }
                if ($inputStream) { $inputStream.Dispose() }
            }
        }
        $zip.Dispose(); $zip = $null
        $outputStream.Dispose(); $outputStream = $null
        for ($attempt = 0; $attempt -lt 40; $attempt++) {
            try { [IO.File]::Move($partial, $Destination); break }
            catch {
                $code = $_.Exception.GetBaseException().HResult -band 0xFFFF
                if ($code -notin @(32, 33) -or $attempt -eq 39) { throw }
                Start-Sleep -Milliseconds 250
            }
        }
    } finally {
        if ($zip) { $zip.Dispose() }
        if ($outputStream) { $outputStream.Dispose() }
    }
}

$program = Resolve-EERSim
$file = Get-Item -LiteralPath $program
if ($file.Name -ine 'EERSim.exe' -or $file.VersionInfo.ProductVersion -notmatch '^0\.1\.3(?:\.|$)') {
    throw 'This recipe requires the EERSim 0.1.3 executable for a matched comparison.'
}
$payload = Join-Path $file.DirectoryName '_internal'
$assets = Join-Path $payload 'assets'
$bundled = Join-Path $payload 'examples\claw_demo'
$compiler = Join-Path $payload 'compiler\bin\clang++.exe'
foreach ($required in @($compiler, (Join-Path $bundled 'src\main.cpp'), (Join-Path $bundled 'include\main.h'),
    (Join-Path $assets 'robots\xdrive_claw.urdf'), (Join-Path $assets 'robots\xdrive_claw.json'),
    (Join-Path $assets 'games\override-2026-27.json'), (Join-Path $assets 'objects\pin_red_yellow.urdf'))) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
        throw 'The selected executable is missing its directory payload, compiler, or bundled claw files. Use the installed app or extract the complete directory ZIP.'
    }
}
[void](New-Item -ItemType Directory -Path $OutputDirectory -Force)
$parentOutput = (Resolve-Path -LiteralPath $OutputDirectory).ProviderPath
$output = Join-Path $parentOutput ('claw-comparison-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss') + '-' + [Guid]::NewGuid().ToString('N').Substring(0, 8))
$private = Join-Path $output 'private'
$share = Join-Path $output 'share'
$workspace = Join-Path $private 'claw_demo'
[void](New-Item -ItemType Directory -Path $workspace, $share)
foreach ($folder in @('src', 'include')) {
    Copy-Item -LiteralPath (Join-Path $bundled $folder) -Destination $workspace -Recurse
}
$sourceHashes = Get-SourceHashes $bundled
if (-not (Same-Hashes $sourceHashes (Get-SourceHashes $workspace))) { throw 'The fresh bundled workspace copy failed its hash check.' }
Write-Host 'Checking bundled asset hashes. No simulation starts during this step.'
$assetHashes = Get-AssetHashes $assets
$exeHash = (Get-FileHash -LiteralPath $program -Algorithm SHA256).Hash.ToLowerInvariant()
$compilerHash = (Get-FileHash -LiteralPath $compiler -Algorithm SHA256).Hash.ToLowerInvariant()
$templateBytes = [IO.File]::ReadAllBytes((Join-Path $PSScriptRoot 'claw-profile.json'))
$templateText = [Text.Encoding]::UTF8.GetString($templateBytes).TrimStart([char]0xFEFF)
$digest = [Security.Cryptography.SHA256]::Create()
try { $templateHash = [BitConverter]::ToString($digest.ComputeHash($templateBytes)).Replace('-', '').ToLowerInvariant() }
finally { $digest.Dispose() }
$scriptHash = (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash.ToLowerInvariant()
$assetKeys = @{robot_urdf='robots/xdrive_claw.urdf'; robot_config='robots/xdrive_claw.json'; game_pack='games/override-2026-27.json'}
$report = [ordered]@{
    schema_version=1; purpose='Matched bundled claw, 100 versus 32 position solver iterations';
    eersim_version=$file.VersionInfo.ProductVersion; executable_sha256=$exeHash; compiler_sha256=$compilerHash;
    profile_template_sha256=$templateHash; recipe_script_sha256=$scriptHash;
    physics_backend=$PhysicsBackend; logical_cuda_device=$PhysicsDevice; physics_hz=$PhysicsHz;
    iteration_order=@(100,32); mode='skills_auto'; competition_seconds=30; wall_budget_seconds_per_case=$WallSeconds;
    max_steps= $(if ($MaxSteps) { $MaxSteps } else { $null }); partial_smoke=($MaxSteps -gt 0);
    rendering_measured=$false; source_hashes=$sourceHashes; asset_hashes=$assetHashes;
    notes='Only solver iterations differ. Wall-budget/step-limit endpoints are incomplete competition runs. Diagnostics do not establish equal physical accuracy or scoring correctness.';
    status='prepared'; comparison_errors=@(); cases=@()
}
$profiles = @{}
foreach ($iterations in @(100, 32)) {
    $config = $templateText | ConvertFrom-Json
    $config.workspace = $workspace
    $config.physics_solver_iterations = $iterations
    $config.physics_hz = $PhysicsHz
    $config.physics_backend = $PhysicsBackend
    $config.physics_device = $PhysicsDevice
    $path = Join-Path $private ('iterations-' + $iterations + '.json')
    Write-JsonFile $config $path
    $profiles[$iterations] = $path
}
Write-JsonFile $report (Join-Path $share 'comparison.json')
Write-Host ('Prepared fresh bundled claw workspace and 100/32 profiles: ' + $output)
if ($PrepareOnly) {
    Write-Host 'PrepareOnly complete. No EERSim executable, simulation, or graphics context was launched.'
    return
}

Write-Host 'Close other EERSim runs before this comparison. This script runs headless cases sequentially.'
Write-Host ('The ' + $WallSeconds + '-second limit applies to physics; loading and compilation add time. Nothing uploads automatically.')
if ($MaxSteps) { Write-Host 'SHORT SMOKE: the step cap prevents full autonomous acceptance.' }
$oldEnvironment = @{}
foreach ($key in @('LOCALAPPDATA', 'APPDATA', 'CXX')) { $oldEnvironment[$key] = [Environment]::GetEnvironmentVariable($key, 'Process') }
$child = $null
$failure = $false
try {
    foreach ($iterations in @(100, 32)) {
        $caseOutput = Join-Path $private ('iterations-' + $iterations)
        $runtime = Join-Path $private ('launcher-runtime-' + $iterations)
        [void](New-Item -ItemType Directory -Path $runtime)
        $env:LOCALAPPDATA = $runtime
        $env:APPDATA = $runtime
        $env:CXX = $compiler
        $arguments = @('--headless', '--diagnostics', '--config', $profiles[$iterations],
            '--diagnostic-output', $caseOutput, '--mode', 'skills_auto', '--diagnostic-wall-seconds', [string]$WallSeconds)
        if ($MaxSteps) { $arguments += @('--diagnostic-max-steps', [string]$MaxSteps) }
        $encoded = ($arguments | ForEach-Object { Quote-NativeArgument ([string]$_) }) -join ' '
        $validation = [Collections.Generic.List[string]]::new()
        $entry = [ordered]@{iterations=$iterations; child_exit_code=$null; watchdog_timeout=$false;
            validation_errors=@(); report_available=$false; competition_complete=$false}
        Write-Host ('Running ' + $iterations + ' iterations at ' + $PhysicsHz + ' Hz...')
        try {
            $child = Start-Process -FilePath $program -ArgumentList $encoded -WorkingDirectory $private -WindowStyle Hidden -PassThru
            $timer = [Diagnostics.Stopwatch]::StartNew()
            $previous = ''
            while (-not $child.WaitForExit(1000)) {
                if ($timer.Elapsed.TotalSeconds -gt $WallSeconds + 540) {
                    $entry.watchdog_timeout = $true
                    & (Join-Path $env:SystemRoot 'System32\taskkill.exe') /PID $child.Id /T /F | Out-Null
                    [void]$child.WaitForExit(10000)
                    break
                }
                $progress = Join-Path $caseOutput 'progress.json'
                if (Test-Path -LiteralPath $progress) {
                    try {
                        $message = (Get-Content -LiteralPath $progress -Raw -Encoding UTF8 | ConvertFrom-Json).message
                        if ($message -ne $previous) { Write-Host $message; $previous = $message }
                    } catch { }
                }
            }
            $child.Refresh()
            $entry.child_exit_code = $child.ExitCode
            if ($child.ExitCode -ne 0 -or $entry.watchdog_timeout) { $validation.Add('The diagnostic process failed or exceeded its watchdog.') }
        } catch {
            [IO.File]::WriteAllText((Join-Path $runtime 'launcher-error.txt'), ($_ | Out-String))
            $validation.Add('The diagnostic process could not complete; launcher details remain private.')
        } finally {
            if ($child -and -not $child.HasExited) {
                & (Join-Path $env:SystemRoot 'System32\taskkill.exe') /PID $child.Id /T /F | Out-Null
                [void]$child.WaitForExit(10000)
            }
            $child = $null
        }
        $casePath = Join-Path $caseOutput ($PhysicsHz.ToString() + 'hz\report\case.json')
        if (Test-Path -LiteralPath $casePath) {
            $case = Get-Content -LiteralPath $casePath -Raw -Encoding UTF8 | ConvertFrom-Json
            if ($case.requested.physics_solver_iterations -ne $iterations -or $case.requested.physics_hz -ne $PhysicsHz) { $validation.Add('Requested iteration count or physics frequency differs.') }
            if ($case.physics.solver_iterations -ne $iterations) { $validation.Add('The physics backend reported a different solver iteration count.') }
            foreach ($key in $assetKeys.Keys) {
                $expectedHash = @($assetHashes | Where-Object { $_.name -eq $assetKeys[$key] })[0].sha256
                if ($case.requested.$key.available -ne $true -or $case.requested.$key.sha256 -ne $expectedHash) {
                    $validation.Add('The reported ' + $key + ' fingerprint differs from the bundled claw profile.')
                }
            }
            if (-not $case.requested.workspace_selected -or $case.requested.extra_field_object_count -ne 1) { $validation.Add('Bundled PROS workspace or the single preload was not selected.') }
            if ($case.physics.backend -ne 'physx' -or [bool]$case.physics.hardware_accelerated -ne ($PhysicsBackend -eq 'physx_gpu')) { $validation.Add('The actual physics backend differs from the requested backend.') }
            if (($case.native_initialization_ticks + $case.native_competition_ticks) -le 0) { $validation.Add('No native PROS initialization or competition calls were recorded.') }
            if ($case.workspace_unchanged -ne $true -or $case.workspace_before.status -ne 'complete' -or $case.workspace_after.status -ne 'complete') { $validation.Add('The diagnostic did not verify unchanged PROS source before and after capture.') }
            $entry.status = $case.status
            $entry.stop_reason = $case.stop_reason
            $entry.competition_complete = ($case.status -eq 'complete' -and $case.stop_reason -eq 'competition_complete')
            $entry.steps = $case.steps
            $entry.simulation_seconds = $case.simulation_seconds
            $entry.step_realtime_factor = $case.step_realtime_factor
            $entry.native_initialization_ticks = $case.native_initialization_ticks
            $entry.native_competition_ticks = $case.native_competition_ticks
            $entry.workspace_before_sha256 = $case.workspace_before.sha256
            $entry.workspace_after_sha256 = $case.workspace_after.sha256
            if ($case.status -ne 'complete') { $validation.Add('The diagnostic case did not complete successfully.') }
        } else { $validation.Add('No case report was produced.') }
        $entry.assets_unchanged = Same-Hashes $assetHashes (Get-AssetHashes $assets)
        $entry.source_unchanged = (Same-Hashes $sourceHashes (Get-SourceHashes $workspace)) -and (Same-Hashes $sourceHashes (Get-SourceHashes $bundled))
        $entry.executable_unchanged = ((Get-FileHash -LiteralPath $program -Algorithm SHA256).Hash.ToLowerInvariant() -eq $exeHash)
        if (-not $entry.assets_unchanged -or -not $entry.source_unchanged -or -not $entry.executable_unchanged) { $validation.Add('Executable, bundled assets, or PROS source changed during capture.') }
        $archive = Join-Path $caseOutput 'EERSim-diagnostics.zip'
        if (Test-Path -LiteralPath $archive) {
            Copy-Item -LiteralPath $archive -Destination (Join-Path $share ('iterations-' + $iterations + '.zip'))
            $entry.report_available = $true
        } else { $validation.Add('The shareable diagnostic ZIP was not produced.') }
        $entry.validation_errors = @($validation.ToArray())
        if ($validation.Count) { $failure = $true }
        $report.cases += $entry
        Write-JsonFile $report (Join-Path $share 'comparison.json')
        if (-not $entry.assets_unchanged -or -not $entry.source_unchanged -or -not $entry.executable_unchanged) { break }
    }
} finally {
    foreach ($key in $oldEnvironment.Keys) { [Environment]::SetEnvironmentVariable($key, $oldEnvironment[$key], 'Process') }
    if ($child -and -not $child.HasExited) {
        & (Join-Path $env:SystemRoot 'System32\taskkill.exe') /PID $child.Id /T /F | Out-Null
        [void]$child.WaitForExit(10000)
    }
    if ($report.cases.Count -eq 2 -and $report.cases[0].workspace_before_sha256 -and
        $report.cases[0].workspace_before_sha256 -ne $report.cases[1].workspace_before_sha256) {
        $failure = $true
        $report.comparison_errors += 'The two cases reported different PROS source fingerprints.'
    }
    $report.status = if ($failure -or $report.cases.Count -ne 2) { 'partial_failure' } elseif ($MaxSteps) { 'partial_smoke_complete' } else { 'capture_complete' }
    $report.competition_completed_both = ($report.cases.Count -eq 2 -and @($report.cases | Where-Object { -not $_.competition_complete }).Count -eq 0)
    Write-JsonFile $report (Join-Path $share 'comparison.json')
    $sendBack = Join-Path $output 'EERSim-claw-iterations-comparison.zip'
    Write-ReportArchive $share $sendBack
    Write-Host ('Report to send back: ' + $sendBack)
    Write-Host 'Only the share folder is zipped. Raw workspace, profiles, caches, and private build output stay local.'
    if (-not $report.competition_completed_both) { Write-Host 'One or both cases ended before the full competition period. This is not autonomous acceptance.' }
}
if ($failure) { throw 'The comparison has validation errors. The ZIP retains available evidence; see comparison.json.' }
