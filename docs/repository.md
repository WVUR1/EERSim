# Repository and large assets

EERSim is a standalone repository. Select any PROS source workspace in the GUI;
the robot code does not need to live inside this repository. Install dependencies
from `environment.yml` and put your desktop compiler on `PATH` or set `CXX`.
The PowerShell launcher also accepts `EERSIM_PYTHON` and discovers local environments
relative to itself. No installation directory or user account is embedded in source.

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
