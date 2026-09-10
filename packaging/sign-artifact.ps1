<#
Optional Authenticode signing using an existing Windows certificate and key.
No certificates are created, imported, exported, or added to trust stores.
SignTool reference: https://learn.microsoft.com/windows/win32/seccrypto/signtool
#>
[CmdletBinding()]
param(
    [string]$FilePath,
    [string]$CertificateThumbprint,
    [ValidateSet('CurrentUser', 'LocalMachine')]
    [string]$StoreLocation = 'CurrentUser',
    [string]$TimestampServer = 'http://timestamp.digicert.com',
    [string]$SignToolPath,
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'

function Resolve-SignTool {
    if ($SignToolPath) {
        $command = Get-Command -Name $SignToolPath -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
        if (-not $command) { throw 'SignToolPath must point to signtool.exe from the Windows SDK.' }
        return $command.Source
    }
    $command = Get-Command signtool.exe -CommandType Application -ErrorAction SilentlyContinue | Select-Object -First 1
    if ($command) { return $command.Source }

    $sdkRoots = @()
    if ($env:WindowsSdkDir) { $sdkRoots += $env:WindowsSdkDir }
    foreach ($key in @('HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots',
                        'HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows Kits\Installed Roots')) {
        $installed = Get-ItemProperty -LiteralPath $key -Name KitsRoot10 -ErrorAction SilentlyContinue
        if ($installed -and $installed.KitsRoot10) { $sdkRoots += $installed.KitsRoot10 }
    }
    $binDirectories = @()
    if ($env:WindowsSdkVerBinPath) { $binDirectories += $env:WindowsSdkVerBinPath }
    foreach ($root in ($sdkRoots | Select-Object -Unique)) {
        $bin = Join-Path $root 'bin'
        if (-not (Test-Path -LiteralPath $bin -PathType Container)) { continue }
        $versions = Get-ChildItem -LiteralPath $bin -Directory |
            Where-Object { $_.Name -match '^\d+\.\d+\.\d+\.\d+$' } |
            Sort-Object { [version]$_.Name } -Descending
        $binDirectories += @($versions | ForEach-Object { $_.FullName })
        $binDirectories += $bin
    }
    $architecture = if ($env:PROCESSOR_ARCHITEW6432) { $env:PROCESSOR_ARCHITEW6432 } else { $env:PROCESSOR_ARCHITECTURE }
    $architectures = switch ($architecture) {
        'ARM64' { @('arm64', 'x64', 'x86') }
        'AMD64' { @('x64', 'x86') }
        default { @('x86') }
    }
    foreach ($bin in ($binDirectories | Select-Object -Unique)) {
        foreach ($architecture in $architectures) {
            $candidate = Join-Path $bin "$architecture/signtool.exe"
            if (Test-Path -LiteralPath $candidate -PathType Leaf) { return $candidate }
        }
    }
    throw 'SignTool was not found. Install the Windows SDK Signing Tools, add signtool.exe to PATH, or specify -SignToolPath.'
}

function Invoke-SignToolChecked {
    param([string[]]$Arguments, [string]$Operation)
    try {
        # Native stderr can become PowerShell error records even on success.
        $ErrorActionPreference = 'Continue'
        # A function-local LASTEXITCODE would hide the native process result.
        $global:LASTEXITCODE = 1
        & $script:resolvedSignTool @Arguments
        $code = $global:LASTEXITCODE
    } finally {
        $ErrorActionPreference = 'Stop'
    }
    if ($code -ne 0) { throw "$Operation failed or reported warnings (SignTool exit $code). No signed-release success is claimed." }
}

$exitCode = 1
try {
    if ($env:OS -ne 'Windows_NT') { throw 'Authenticode signing requires Windows.' }
    $thumbprint = ($CertificateThumbprint -replace '[\s\u200e\u200f]', '').ToUpperInvariant()
    if ($thumbprint -notmatch '^[0-9A-F]{40}$') {
        throw 'Specify -CertificateThumbprint for an existing code-signing certificate in the Windows Personal (My) store. Use the build script without signing options for an unsigned private preview.'
    }
    $timestamp = $null
    if (-not [Uri]::TryCreate($TimestampServer, [UriKind]::Absolute, [ref]$timestamp) -or
        $timestamp.Scheme -notin @('http', 'https') -or -not $timestamp.Host -or $timestamp.UserInfo) {
        throw 'TimestampServer must be an HTTP or HTTPS RFC 3161 service URL without embedded credentials.'
    }
    $certificatePath = "Cert:\$StoreLocation\My\$thumbprint"
    $certificate = Get-Item -LiteralPath $certificatePath -ErrorAction SilentlyContinue
    if (-not $certificate) { throw "The selected certificate is absent from $StoreLocation/My. Configure an existing organization certificate and its key provider before requesting signing." }
    if (-not $certificate.HasPrivateKey) { throw 'The selected certificate has no associated private key. Configure its existing key provider; the script never exports or imports keys.' }
    $now = Get-Date
    if ($now -lt $certificate.NotBefore -or $now -gt $certificate.NotAfter) { throw 'The selected code-signing certificate is not currently valid.' }
    $eku = $certificate.Extensions | Where-Object { $_.Oid.Value -eq '2.5.29.37' }
    if ($eku -and -not @($eku.EnhancedKeyUsages | Where-Object { $_.Value -in @('1.3.6.1.5.5.7.3.3', '2.5.29.37.0') }).Count) {
        throw 'The selected certificate does not permit code signing.'
    }
    $script:resolvedSignTool = Resolve-SignTool
    if ($CheckOnly) {
        Write-Host "Signing prerequisites found: $($certificate.Subject) using $script:resolvedSignTool"
        Write-Host 'Signing still requires access to the selected private key, timestamp service, and a trusted certificate chain.'
    } else {
        if (-not $FilePath -or -not (Test-Path -LiteralPath $FilePath -PathType Leaf)) { throw 'FilePath must name an existing final artifact to sign.' }
        $artifact = (Resolve-Path -LiteralPath $FilePath).Path
        $signArguments = @('sign', '/v', '/sha1', $thumbprint, '/s', 'My', '/fd', 'SHA256',
                           '/tr', $timestamp.AbsoluteUri, '/td', 'SHA256')
        if ($StoreLocation -eq 'LocalMachine') { $signArguments += '/sm' }
        $signArguments += $artifact
        Invoke-SignToolChecked -Arguments $signArguments -Operation 'Authenticode signing and timestamping'
        Invoke-SignToolChecked -Arguments @('verify', '/pa', '/all', '/tw', '/v', $artifact) -Operation 'Authenticode verification'
        $signature = Get-AuthenticodeSignature -LiteralPath $artifact
        if ($signature.Status -ne 'Valid' -or -not $signature.SignerCertificate -or
            $signature.SignerCertificate.Thumbprint -ne $thumbprint -or -not $signature.TimeStamperCertificate) {
            throw 'The resulting signature did not verify with the selected signer and a timestamp.'
        }
        Write-Host "Signed and verified: $artifact"
    }
    $exitCode = 0
} catch {
    Write-Host ("EERSim signing: " + $_.Exception.Message) -ForegroundColor Red
}
exit $exitCode
