# Windows installer and EXE builds

The private [0.1.7 Windows preview](https://github.com/WVUR1/EERSim/releases/tag/v0.1.7-windows-preview.1)
contains Python, PySide6, PyBullet, VTK, PhysX/OVStage, SciPy, default robots and
fields, and the desktop LLVM-MinGW compiler for selected PROS workspaces.
Recipients do not need Python, Conda, a separate compiler, or WSL installed.
Version 0.1.7 adds visible Reset progress, prevents conflicting commands during
the rebuild, strengthens PhysX worker shutdown, and keeps the engine available
after a failed Reset. See the [Reset validation and limits](reset-0.1.7.md). It retains the viewport,
loading, CPU worker, and goal/pin collision improvements from 0.1.6. The
[0.1.6 validation report](performance-0.1.6.md) records those earlier measurements
and limits; it does not establish performance or CUDA reset reliability for 0.1.7.

The installer displays the [academic/noncommercial license](../LICENSE).
Commercial use requires prior written permission from Andrew Sarver; see
[licensing](licensing.md) and the included [Qt/PySide notices](notices/qt-pyside6.md).
Third-party libraries retain their separate licenses.

Previous releases passed headless native compilation, Bullet/PhysX CPU,
automatic fallback and explicit missing-CUDA checks. The supplied RTX 4070 Ti
capture of 0.1.3 records Windows CUDA execution of the PROS claw demo and preload
placement. Those older measurements do not establish 0.1.7 Windows performance.
See the [capture analysis](diagnostics-4070ti-followup.md) and [validation](validation.md).

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
4. Click **Apply settings**. **Save/Load** in **Experiment setup**
   stores and restores experiment profiles.

The application reads external designs in place. Generated import wrappers and
the writable copy of the bundled claw demo live in EERSim's per-user data
directory. There is no need to modify the installed `_internal` assets.

## Scene setup and live performance

**Load claw scoring demo** selects the bundled robot, field, PROS workspace, and
Skills Auto mode. Click **Build workspace** explicitly, then **Start**. Loading
the demo, importing assets, applying settings, and starting a run do not compile
robot source. Rebuild explicitly after editing that source.

Start can reuse an unchanged prepared scene. It still resets a scene that has
already advanced. **Apply settings** reloads physical changes and unchanged
settings; use it after editing external asset files. Applying only viewport FPS
or configured simulation speed changes keeps the current physics state. Camera
movement does not reload the scene. **Reset** stops robot code and restores the
robot, field objects, and seeded noise to the last applied starting state.
It keeps the camera and does not build code or apply pending edits.
**Resetting…** remains visible until the rebuilt scene has rendered; conflicting
actions are disabled during that work. Complex scenes and PhysX initialization
can still take several seconds.

**Field → PhysX CPU workers · advanced** selects host task workers, including
CPU work used by CUDA physics. The default is two; zero runs tasks inline.
The control offers the detected CPU count and preserves a larger saved count.
More workers do not necessarily improve performance. `EERSIM_PHYSX_THREADS`,
when set, overrides this profile/GUI value. Fixed physics steps and PROS callbacks
remain ordered regardless of worker count.

**Telemetry** reports completed live rendering FPS in wall time, independently
of physics rate. This is not a measurement of Qt painting or monitor refresh.
The GUI takes the newest rendered frame when busy. No physics steps are dropped;
individual slow physics steps and expensive rendering can still limit smoothness.

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
