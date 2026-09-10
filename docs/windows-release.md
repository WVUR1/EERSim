# Windows EXE builds

Run the released `EERSim.exe` directly on 64-bit Windows. It includes Python,
PySide6, PyBullet, VTK, the robot and field assets, and (in the default build) the
desktop LLVM-MinGW compiler needed to compile a selected PROS workspace. The
target machine does not need Python, Conda, PROS tooling, or WSL installed.

Current private previews are **unsigned** because the organization has no
code-signing certificate. Windows may show an unknown-publisher warning. An
unsigned artifact must not be described as a signed or verified-publisher release.

For regular use, download the portable `EERSim-windows-x64.zip`, extract the
entire archive, and run `EERSim/EERSim.exe`. Keep the adjacent `_internal` folder.
The one-file EXE extracts its runtime to a temporary directory at every startup;
the large field assets and compiler can make this take several minutes. Keep
space for the extracted runtime. The directory build avoids repeated extraction.
Choose an external PROS workspace with `src/` and `include/` in the GUI. Bundled
demo code is copied to the user's EERSim application data directory so it remains
writable and survives temporary-bundle cleanup.

## Build an unsigned private preview

Build on Windows; PyInstaller does not cross-compile a Windows executable from
Ubuntu/WSL. First extract the x86_64 UCRT LLVM-MinGW toolchain described below
under `.sim-tools/`; `setup.ps1` installs Python dependencies, not the compiler.
Then run from the repository root:

```powershell
.\setup.ps1
.\build.ps1
```

The build installs pinned packaging tools into the selected local Python
environment. It finds an extracted LLVM-MinGW toolchain under `.sim-tools/llvm*`
or accepts an explicit toolchain directory:

```powershell
.\build.ps1 -CompilerPath $llvmMingwDirectory
```

Obtain the **x86_64 Windows host, UCRT** ZIP from the
[official LLVM-MinGW releases](https://github.com/mstorsjo/llvm-mingw/releases).
Extract it into `.sim-tools/` or set `$llvmMingwDirectory` to its extracted root
(the directory containing `bin/clang++.exe` and `x86_64-w64-mingw32/`). A complete
compiler is bundled so the recipient can build robot code without a separate
compiler install. The locally validated toolchain is LLVM-MinGW 20260616 UCRT.

Output is `dist/windows-onefile/EERSim.exe`, its `.sha256` checksum, and
`build-manifest.json`. These generated artifacts stay out of Git. Share them
through the private repository's Releases page, whose assets can exceed Git's
ordinary file-size limit.

For faster starts, build a directory bundle and distribute its complete ZIP:

```powershell
.\build.ps1 -Layout Directory
```

Extract `dist/windows-onedir/EERSim-windows-x64.zip` and run `EERSim/EERSim.exe`.
The adjacent `_internal` directory is required. `-WithoutCompiler` creates a
smaller package whose users must provide their own desktop compiler through
`CXX` or `PATH`. `-PythonPath` selects another fully installed Windows build
environment.

## Optional signing later

When the organization has a trusted Authenticode certificate and its key provider
configured in the Windows Personal (`My`) certificate store, use:

```powershell
.\build.ps1 -CertificateThumbprint $thumbprint
```

The helper discovers Microsoft's SignTool through `PATH` or the installed
Windows SDK registry settings. It signs with SHA-256, requests an RFC 3161
timestamp, and verifies the resulting signature and signer before reporting
success. `-CertificateStore LocalMachine`, `-SignToolPath`, and `-TimestampServer`
support other existing signing setups. No certificates are generated or imported;
no private key or password is needed in the repository. This signing path has
only prerequisite/error checks until an actual organization certificate exists.

See [Microsoft's SignTool documentation](https://learn.microsoft.com/en-us/windows/win32/seccrypto/signtool)
for certificate-store and verification options.
