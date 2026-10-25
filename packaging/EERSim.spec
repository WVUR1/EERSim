# -*- mode: python ; coding: utf-8 -*-
"""Windows standalone build; invoke through the repository's build.ps1.

EERSIM_BUNDLE_MODE: onefile (default) or onedir.
EERSIM_BUNDLE_COMPILER: optional complete LLVM-MinGW distribution directory.
"""
import os
from pathlib import Path
import runpy
import sys

from PyInstaller.building.datastruct import Tree
from PyInstaller.compat import is_pure_conda
from PyInstaller.utils.hooks import collect_submodules, copy_metadata, collect_all


if sys.platform != "win32":
    raise SystemExit("Build the Windows EERSim EXE on 64-bit Windows.")
if sys.maxsize <= 2**32:
    raise SystemExit("The EERSim Windows bundle requires 64-bit Python.")

packaging = Path(SPECPATH).resolve()
checkout = packaging.parent
# Hook collection runs before Analysis applies pathex. Always inspect this
# checkout, even if a different EERSim version is installed in the build env.
sys.path.insert(0, str(checkout))
layout = os.environ.get("EERSIM_BUNDLE_MODE", "onefile").lower()
if layout not in {"onefile", "onedir"}:
    raise SystemExit("EERSIM_BUNDLE_MODE must be onefile or onedir.")

# Keep these beside eersim/ inside the bundle. The same __file__-relative
# paths work in a checkout, a directory bundle, and a one-file extraction.
datas = []
scene_probe = checkout / "build/native/physx_scene_probe.dll"
if not scene_probe.is_file():
    raise SystemExit("Build the GPU verification helper through build.ps1 before freezing EERSim.")
datas += [(str(scene_probe), "eersim/_native"),
          (str(checkout / "eersim/_native/physx/LICENSE.md"), "eersim/_native/physx")]
for folder in ("assets", "native", "examples", "docs"):
    source = checkout / folder
    if not source.is_dir():
        raise SystemExit(f"Missing bundle input: {source}")
    for path in sorted(source.rglob("*")):
        relative = path.relative_to(checkout)
        if path.is_file() and not any(part in {".eersim", "__pycache__", ".pytest_cache"}
                                      for part in relative.parts):
            datas.append((str(path), relative.parent.as_posix()))
for name in ("README.md", "environment.yml", "LICENSE", "LICENSE.txt", "LICENSE.md"):
    if (checkout / name).is_file():
        datas.append((str(checkout / name), "."))

# Include package metadata and its license files, also useful for diagnostics.
for distribution in ("eerkit-simulator", "numpy", "mujoco", "pybullet", "PySide6",
                     "PySide6_Essentials", "PySide6_Addons", "shiboken6", "vtk",
                     "ovphysx", "ovstage", "scipy", "packaging",
                     "fsspec", "typing_extensions", "zipp"):
    datas += copy_metadata(distribution, recursive=(distribution == "mujoco"))

# MuJoCo loads its native library and resource files at runtime.
# Collect them explicitly so directory and one-file distributions work offline.
mujoco_datas, mujoco_binaries, mujoco_hiddenimports = collect_all("mujoco")
datas += mujoco_datas
binaries = mujoco_binaries
if is_pure_conda:
    from PyInstaller.utils.hooks import conda_support
    # PyBullet's runtime dependencies live outside its .pyd directory. NumPy's
    # own hook collects BLAS/MKL; traversing it again here duplicates hundreds
    # of megabytes under a second destination. Python DLLs use standard hooks.
    binaries += conda_support.collect_dynamic_libs(
        "pybullet", dest="Library/bin",
        excludes={"numpy", "python", "python_abi", "__win"},
    )
    for distribution in conda_support.walk_dependency_tree("pybullet").values():
        for filename in distribution.files:
            relative = Path(str(filename))
            if relative.parts[:2] == ("Library", "share") and "licenses" in relative.parts:
                source = filename.locate()
                if source.is_file():
                    datas.append((str(source), relative.parent.as_posix()))

# The maintained hooks pinned in requirements-build.txt handle NumPy's BLAS,
# Qt's platform/image plugins, and VTK 9.6's extension-module dependencies.
# OpenGL backend registration and app modules also used by helper invocations
# must survive import analysis even though they are loaded lazily.
gpu_payload, gpu_hiddenimports = runpy.run_path(str(packaging / "gpu_payload.py"))["collect_gpu_payload"]()
hiddenimports = collect_submodules("eersim") + gpu_hiddenimports + mujoco_hiddenimports + [
    "PySide6.QtCore", "PySide6.QtGui", "PySide6.QtWidgets",
    "vtk", "vtkmodules.util.numpy_support", "vtkmodules.vtkRenderingOpenGL2",
    "vtkmodules.vtkInteractionStyle", "pygame",
]

analysis = Analysis(
    [str(packaging / "entrypoint.py")],
    pathex=[str(checkout)],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[str(packaging / "runtime_hook.py")],
    excludes=["PyQt5", "PyQt6", "PySide2", "tkinter", "vtkmodules.tk",
              "vtkmodules.qt", "vtkmodules.wx", "pytest", "cadquery", "OCP"],
    noarchive=False,
)
# Qt's automatic plugin collection includes an unused GPL-only virtual
# keyboard. EERSim uses desktop QtWidgets, not this optional input method.
# Retain normal platform, image, icon, style, and keyboard/input processing.
# Source: https://doc.qt.io/qt-6.8/qtvirtualkeyboard-index.html#licenses-and-attributions

def keep_qt_payload(entry):
    destination = str(entry[0]).replace("\\", "/").casefold()
    omitted = {"qt6virtualkeyboard.dll", "qtvirtualkeyboardplugin.dll"}
    return not (destination.startswith("pyside6/") and destination.rsplit("/", 1)[-1] in omitted)


analysis.binaries = [entry for entry in analysis.binaries if keep_qt_payload(entry)]
analysis.datas = [entry for entry in analysis.datas if keep_qt_payload(entry)]
pyz = PYZ(analysis.pure)

# Add the compiler tree after Analysis so its executables, target libraries,
# symlinks copied as files, resource headers, and licenses retain their layout.
# They are external toolchain payload, not Python extension dependencies.
toolchain = []
compiler_source = os.environ.get("EERSIM_BUNDLE_COMPILER", "").strip()
if compiler_source:
    compiler_root = Path(compiler_source).expanduser().resolve()
    if not (compiler_root / "bin" / "clang++.exe").is_file():
        raise SystemExit("EERSIM_BUNDLE_COMPILER must contain bin/clang++.exe.")
    if not (compiler_root / "x86_64-w64-mingw32").is_dir():
        raise SystemExit("The bundled compiler must be a complete Windows LLVM-MinGW distribution.")
    toolchain = Tree(str(compiler_root), prefix="compiler", typecode="DATA")

options = dict(
    name="EERSim",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    version=str(packaging / "version_info.txt"),
    icon=str(checkout / "assets/logos/happy.png"),
)
if layout == "onedir":
    executable = EXE(pyz, analysis.scripts, [], exclude_binaries=True,
                     contents_directory="_internal", **options)
    COLLECT(executable, analysis.binaries, analysis.datas, gpu_payload, toolchain,
            strip=False, upx=False, name="EERSim")
else:
    EXE(pyz, analysis.scripts, analysis.binaries, analysis.datas, gpu_payload, toolchain, [],
        runtime_tmpdir=None, **options)
