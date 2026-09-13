# Windows installer and EXE builds

The private [0.1.3 Windows preview](https://github.com/WVUR1/EERSim/releases/tag/v0.1.3-windows-preview.1)
includes GPU physics. Its `EERSim-Setup-x64.exe` contains Python, PySide6,
PyBullet, VTK, PhysX/OVStage, SciPy, default robots and fields, and the desktop
LLVM-MinGW compiler for selected PROS workspaces. Recipients do not need Python,
Conda, PROS tooling, a separate compiler, or WSL installed.
The frozen application passed headless native compilation, Bullet/PhysX CPU,
automatic fallback, and explicit missing-CUDA checks. Linux A100 execution
validated the integrated GPU backend. A subsequent RTX 4070 Ti diagnostic capture
of 0.1.3 also records Windows CUDA execution of the native PROS claw demo, preload
placement, and completion of the 30-second mode at 120 Hz. See the
[capture analysis](diagnostics-4070ti-followup.md) and [validation](validation.md).

CUDA physics requires an NVIDIA GPU with compute capability 7.0 or newer and a
driver exposing CUDA 12.8 or newer. The x64 PhysX wheel also requires an
AVX-capable CPU. A separate CUDA toolkit or Isaac Sim installation is unnecessary.
**Field → Physics engine** defaults to **Auto**, which reports its reason when
falling back to Bullet. Explicit **PhysX · NVIDIA CUDA** selection fails visibly
if unavailable. **Graphics renderer** is an independent setting; OpenGL support
alone does not imply CUDA physics. See [backend details](gpu-physics.md).

The installer targets 64-bit Windows and installs for the current account under
`%LOCALAPPDATA%\Programs\EERSim` by default. It does not request administrator
privileges. The destination and Start menu folder are editable; the desktop
shortcut is optional. Reinstalling updates that installation. Remove it through
Windows **Installed apps** or its **Uninstall EERSim** Start menu shortcut.
Uninstall removes installed program files and shortcuts, while preserving
`%LOCALAPPDATA%\EERSim` user data and external robot projects, designs, and maps.
Keep custom files in your own folders, outside the installation directory.

Current private previews are **unsigned** because the organization has no
code-signing certificate. Windows may show an unknown-publisher warning. These
artifacts must not be described as signed or verified-publisher releases.

## Import custom robots and fields

1. Select an external PROS workspace containing `src/` and `include/` with
   **Browse…**, then **Build workspace**.
2. Open **Robot → Import URDF…** and select a URDF, with its referenced meshes
   retained beside it. Assign ports, cartridges, and gear ratios in the motor
   setup dialog. Hold the jog buttons to test joints in the movable 3D preview.
3. Open **Field → Import field…** to choose a game-pack JSON or a complete field
   URDF. Built-in field and robot presets are already available.
4. Click **Apply settings & reset scene**. **Save/Load** in **Experiment setup**
   stores and restores experiment profiles.

The application reads external designs in place. Generated import wrappers and
the writable copy of the bundled claw demo live in EERSim's per-user data
directory. There is no need to modify the installed `_internal` assets.

## Build the installer

Build on 64-bit Windows. First run `setup.ps1` and obtain an **x86_64 Windows
host, UCRT** toolchain from the
[official LLVM-MinGW releases](https://github.com/mstorsjo/llvm-mingw/releases).
Extract it under `.sim-tools/llvm*`, or pass `-CompilerPath` with the extracted
directory containing `bin/clang++.exe` and `x86_64-w64-mingw32/`. Setup installs
Python dependencies; it does not install this compiler. The validated toolchain
is LLVM-MinGW 20260616 UCRT.

Install **Inno Setup 7.1 or newer** from its
[official download page](https://jrsoftware.org/isdl.php). The wrapper discovers
`ISCC.exe` through `PATH`, `.sim-tools/inno-setup/`, or Inno Setup's installation
registry entries. Use `-InnoSetupCompiler` to select it explicitly.

```powershell
.\setup.ps1
.\build-installer.ps1
# Optional explicit locations, held in variables selected on this machine:
.\build-installer.ps1 -CompilerPath $llvmMingwDirectory -InnoSetupCompiler $iscc
```

The wrapper runs `build.ps1 -Layout Directory -SkipArchive`, then packages the
complete directory payload with Inno Setup. Output is
`dist/windows-installer/EERSim-Setup-x64.exe`, its `.sha256` checksum, and
`build-manifest.json`. Generated files remain ignored by Git; publish them as
private release assets, which support files larger than Git's normal limit.

`packaging/requirements-build.txt` installs the GPU runtime by default:
`ovphysx==0.5.11`, `ovstage==0.1.1.355824`, and `scipy==1.16.3`. It also pins
`packaging==23.2` and `wheel==0.45.1` to keep the runtime and build-tool dependency
requirements compatible. The spec preserves the complete NVIDIA package trees,
USD schemas, plugin DLLs, metadata, and licenses. Missing or mismatched runtime
packages fail the build. These large binary inputs and generated artifacts stay
under ignored environment/build directories; do not commit them to Git.

To package an already built and validated directory without rebuilding:

```powershell
.\build-installer.ps1 -SkipBundleBuild
.\build-installer.ps1 -SkipBundleBuild -BundleDirectory $validatedBundle
```

`-BundleDirectory` requires `-SkipBundleBuild`. It must point at the folder
containing `EERSim.exe` and `_internal`. The wrapper validates the runtime,
default assets, native adapter, and compiler before packaging. Both fresh builds
and `-SkipBundleBuild` reject missing GPU DLLs, the native scene-verification
helper, and absent or mismatched runtime metadata. The build manifest records
the actual bundled runtime versions and helper checksum. Stop running
copies and their native compiler processes before rebuilding the bundle.
`-PythonPath` selects another complete Windows Python build environment.
`-WithoutCompiler` is an advanced option for recipients who supply their own
desktop compiler through `CXX` or `PATH`.

## Other distribution layouts

These build options remain available in addition to the installer.

```powershell
.\build.ps1 -Layout Directory
.\build.ps1 -Layout Directory -SkipArchive
.\build.ps1
```

The first command produces `dist/windows-onedir/EERSim-windows-x64.zip`; extract
the whole ZIP and run `EERSim/EERSim.exe`, keeping `_internal` beside it. The
second leaves the complete directory ready for installer packaging and records
the EXE checksum in its manifest. The last command produces the one-file
`dist/windows-onefile/EERSim.exe`. That executable extracts the large runtime
and compiler to a temporary folder on every launch, which can take several
minutes. The installer and directory layout avoid repeated extraction.

## Optional signing later

When the organization has a trusted Authenticode certificate and its key provider
configured in the Windows Personal (`My`) certificate store, use:

```powershell
.\build-installer.ps1 -CertificateThumbprint $thumbprint
# The portable formats also support signing:
.\build.ps1 -CertificateThumbprint $thumbprint
```

The existing certificate is checked before building. The application is signed
before packaging; an Inno Setup signing callback signs the embedded uninstaller
and final setup executable. Microsoft's SignTool uses SHA-256, requests an RFC
3161 timestamp, and verifies the signature and selected signer before success.
`-CertificateStore LocalMachine`, `-SignToolPath`, and `-TimestampServer` support
other existing signing setups. No certificates are generated or imported, and
no private key or password belongs in the repository. The actual signing path
remains untested until an organization certificate is available.

See [Microsoft SignTool](https://learn.microsoft.com/en-us/windows/win32/seccrypto/signtool),
[Inno Setup compiler options](https://jrsoftware.org/ishelp/topic_compilercmdline.htm),
and [Inno Setup signing](https://jrsoftware.org/ishelp/topic_setup_signtool.htm).
