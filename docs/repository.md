# Repository and large assets

EERSim is a standalone repository. Select any PROS source workspace in the GUI;
the robot code does not need to live inside this repository. On 64-bit Windows,
run `.\setup.ps1` once, then `.\launch.ps1` from the repository root. Setup creates
or repairs `.sim-tools/physics-env` using its own downloaded Python and dependencies;
it requires internet access and several gigabytes of free space, but no existing
Python, Conda, or administrator privileges. Rerun setup to repair missing modules
such as PySide6. It ignores global Python and does not install a C++ compiler.
Put your desktop compiler on `PATH` or set `CXX`.

GPU physics is optional when running from source. After normal Windows setup,
install the pinned runtime into that same environment:

```powershell
& .\.sim-tools\physics-env\python.exe -m pip install -e ".[gpu]"
```

For an activated Conda or virtual environment, use `python -m pip install -e
".[gpu]"`. The extra installs `ovphysx==0.5.11`, its matching
`ovstage==0.1.1.355824`, and `scipy==1.16.3`. CUDA physics needs a compatible NVIDIA
GPU with compute capability 7.0 or newer and a driver exposing CUDA 12.8 or newer.
In **Field → Physics engine**, **Auto** selects MuJoCo CPU, included by normal
setup. If MuJoCo initialization fails, Auto uses Bullet and shows the reason
in Field and Telemetry. An
explicit **PhysX · NVIDIA CUDA** selection reports an error instead of silently
switching engines. Save an explicit engine selection for comparisons across
machines. **Graphics renderer** independently controls the viewport.

The Windows installer build includes these GPU runtime packages by default;
installed-app users do not need Python, Conda, or a separate package install.
Vendor runtime DLLs stay in ignored build/distribution folders, not Git.
See [physics backend details](gpu-physics.md) and the
[MuJoCo integration](mujoco.md) for current limits and historical validation.

The launcher discovers environments relative to itself and accepts a local
`.venv` only when its dependencies load. An explicit `EERSIM_PYTHON` overrides the
managed environment; `.\launch.ps1 --check` reports the selected Python and checks.
The [start instructions](../README.md#start-here) include troubleshooting and an
alternative `environment.yml` installation for users who already have Conda or
install Miniforge first. No installation directory or user account is embedded in source.

Saved experiments refer to built-in assets with `asset://` names resolved against
the current installation. Selected workspaces and user assets use profile-relative
paths; generated field packs and prepared URDFs also use relative asset references.
Move the profile and its external asset tree together when sharing a project.
Windows files on different drives or network shares cannot have relative paths;
those user-selected references remain absolute and must be selected again on a
different machine. They are never default installation paths in source.

The runtime field meshes, URDFs, maps, screenshot, and preview video are included
in ordinary Git. The largest current file is about 28 MiB. Raw STEP/BREP files,
CAD download archives, local tool environments, native build products, and wheels
are ignored. Keep CAD downloads under `.sim-tools/` and build wheels under `dist/`.
Wavefront `.obj` files under `assets/` are explicitly included.

GitHub warns above 50 MiB and blocks ordinary Git files above 100 MiB; see
[GitHub's file size documentation](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github).
Before committing new assets, stage the intended files and run:

```sh
git add .
python tools/check_git_sizes.py
```

The same check runs in CI. Current assets do not require Git LFS. If a future
runtime asset must exceed the limit, install Git LFS and track that specific
asset **before committing it**, for example:

```sh
git lfs install
git lfs track "assets/meshes/new-season/large-field.stl"
git add .gitattributes assets/meshes/new-season/large-field.stl
python tools/check_git_sizes.py
```

Git LFS stores a small pointer in Git and downloads the actual asset on checkout.
Contributors and CI will then need Git LFS enabled (`lfs: true` on the checkout
action). Ignored raw CAD is for local regeneration; commit only the runtime assets
needed to load the field. Existing third-party CAD attribution remains in
`assets/README.md` and `assets/cad_manifest.json`.
