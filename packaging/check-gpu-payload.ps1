# Read-only validation of an extracted Windows application payload. No Python or
# CUDA device is required; the caller gets versions from the actual bundled metadata.
function Get-EERSimGpuPayload {
    [CmdletBinding()]
    param([Parameter(Mandatory = $true)][string]$PayloadRoot)

    foreach ($relative in @('ovphysx/lib/ovphysx.dll', 'ovstage/bin/ovstage.dll',
                            'eersim/_native/physx_scene_probe.dll')) {
        $path = Join-Path $PayloadRoot $relative
        if (-not (Test-Path -LiteralPath $path -PathType Leaf) -or (Get-Item -LiteralPath $path).Length -eq 0) {
            throw "Incomplete GPU physics payload: missing or empty $relative. Rebuild the directory bundle."
        }
    }
    $requiredVersions = [ordered]@{ ovphysx = '0.5.11'; ovstage = '0.1.1.355824' }
    $actualVersions = [ordered]@{}
    foreach ($package in $requiredVersions.Keys) {
        $metadataDirectories = @(Get-ChildItem -LiteralPath $PayloadRoot -Directory -Filter "$package-*.dist-info")
        if ($metadataDirectories.Count -ne 1) {
            throw "GPU physics payload requires exactly one $package distribution metadata directory."
        }
        $metadata = Join-Path $metadataDirectories[0].FullName 'METADATA'
        if (-not (Test-Path -LiteralPath $metadata -PathType Leaf)) { throw "GPU physics payload is missing $package METADATA." }
        $content = Get-Content -LiteralPath $metadata -Raw -Encoding UTF8
        $name = [regex]::Match($content, '(?m)^Name:\s*([^\s]+)').Groups[1].Value
        $version = [regex]::Match($content, '(?m)^Version:\s*([^\s]+)').Groups[1].Value
        if ($name -ne $package -or $version -ne $requiredVersions[$package]) {
            throw "GPU physics payload requires $package==$($requiredVersions[$package]); metadata reports $name==$version."
        }
        $actualVersions[$package] = $version
    }
    return [ordered]@{
        includes_gpu_physics = $true
        gpu_runtime_versions = $actualVersions
        gpu_runtime_versions_source = 'bundled-dist-info'
        gpu_verification_helper_sha256 = (Get-FileHash -LiteralPath (Join-Path $PayloadRoot 'eersim/_native/physx_scene_probe.dll') -Algorithm SHA256).Hash.ToLowerInvariant()
    }
}
