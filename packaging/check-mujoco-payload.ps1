# Read-only inspection: do not launch the application or load a native DLL.
function Get-EERSimSourceVersion {
    [CmdletBinding()]
    param([Parameter(Mandatory = $true)][string]$SourceRoot)
    $source = Get-Content -LiteralPath (Join-Path $SourceRoot 'eersim/__init__.py') -Raw -Encoding UTF8
    $version = [regex]::Match($source, '(?m)^__version__\s*=\s*"([^"]+)"').Groups[1].Value
    if (-not $version) { throw 'Cannot determine the application version from this checkout.' }
    return $version
}

function Get-EERSimMuJoCoPayload {
    [CmdletBinding()]
    param(
        [Parameter(Mandatory = $true)][string]$PayloadRoot,
        [Parameter(Mandatory = $true)][ValidateNotNullOrEmpty()][string]$ExpectedAppVersion
    )
    $library = Join-Path $PayloadRoot 'mujoco/mujoco.dll'
    if (-not (Test-Path -LiteralPath $library -PathType Leaf) -or (Get-Item -LiteralPath $library).Length -eq 0) {
        throw 'Incomplete MuJoCo payload: missing or empty mujoco/mujoco.dll. Rebuild the directory bundle.'
    }
    $packageRoot = Join-Path $PayloadRoot 'mujoco'
    foreach ($module in @('_callbacks', '_constants', '_enums', '_errors', '_functions', '_specs', '_structs')) {
        $bindings = @(Get-ChildItem -LiteralPath $packageRoot -File -Filter "$module*.pyd" |
                      Where-Object { $_.Name -match ('^' + [regex]::Escape($module) + '(\.[^.]+)?\.pyd$') })
        if ($bindings.Count -ne 1 -or $bindings[0].Length -eq 0) {
            throw "Incomplete MuJoCo payload: requires one nonempty $module Python binding. Rebuild the directory bundle."
        }
    }
    $required = [ordered]@{ mujoco = '3.17.0'; 'eerkit-simulator' = $ExpectedAppVersion }
    $versions = [ordered]@{}
    $directories = @(Get-ChildItem -LiteralPath $PayloadRoot -Directory)
    foreach ($package in $required.Keys) {
        $suffix = '-[^/]+\.dist-info$'
        if ($package -eq 'eerkit-simulator') {
            # copy_metadata can select local editable-install egg metadata.
            $suffix = '(?:-[^/]+\.dist-info|(?:-[^/]+)?\.egg-info)$'
        }
        $pattern = '^' + (($package -split '-') -join '[-_.]+') + $suffix
        $candidates = @($directories | Where-Object { $_.Name -match $pattern })
        if ($candidates.Count -ne 1) { throw "MuJoCo payload requires exactly one $package distribution metadata directory." }
        $metadataName = if ($candidates[0].Name.EndsWith('.egg-info')) { 'PKG-INFO' } else { 'METADATA' }
        $metadata = Join-Path $candidates[0].FullName $metadataName
        if (-not (Test-Path -LiteralPath $metadata -PathType Leaf)) { throw "MuJoCo payload is missing $package $metadataName." }
        $content = Get-Content -LiteralPath $metadata -Raw -Encoding UTF8
        $name = [regex]::Match($content, '(?m)^Name:\s*([^\s]+)').Groups[1].Value -replace '[-_.]+', '-'
        $version = [regex]::Match($content, '(?m)^Version:\s*([^\s]+)').Groups[1].Value
        if ($name -ne $package -or $version -ne $required[$package]) {
            throw "MuJoCo payload requires $package==$($required[$package]); bundled metadata reports $name==$version. Rebuild from this checkout."
        }
        $versions[$package] = $version
    }
    return [ordered]@{
        includes_mujoco = $true
        mujoco_version = $versions['mujoco']
        mujoco_version_source = 'bundled-dist-info'
        application_version = $versions['eerkit-simulator']
        mujoco_library_sha256 = (Get-FileHash -LiteralPath $library -Algorithm SHA256).Hash.ToLowerInvariant()
    }
}
