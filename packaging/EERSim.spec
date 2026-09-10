# -*- mode: python ; coding: utf-8 -*-
"""Windows standalone build; invoke through the repository's build.ps1.

EERSIM_BUNDLE_MODE: onefile (default) or onedir.
EERSIM_BUNDLE_COMPILER: optional complete LLVM-MinGW distribution directory.
"""
import os
from pathlib import Path
import sys

from PyInstaller.building.datastruct import Tree
from PyInstaller.compat import is_pure_conda
from PyInstaller.utils.hooks import collect_submodules, copy_metadata


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
for distribution in ("eerkit-simulator", "numpy", "pybullet", "PySide6",
                     "PySide6_Essentials", "PySide6_Addons", "shiboken6", "vtk"):
    datas += copy_metadata(distribution)

binaries = []
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
hiddenimports = collect_submodules("eersim") + [
    "PySide6.QtCore", "PySide6.QtGui", "PySide6.QtWidgets",
    "vtk", "vtk.util.numpy_support", "vtkmodules.vtkRenderingOpenGL2",
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
    icon=None,
)
if layout == "onedir":
    executable = EXE(pyz, analysis.scripts, [], exclude_binaries=True,
                     contents_directory="_internal", **options)
    COLLECT(executable, analysis.binaries, analysis.datas, toolchain,
            strip=False, upx=False, name="EERSim")
else:
    EXE(pyz, analysis.scripts, analysis.binaries, analysis.datas, toolchain, [],
        runtime_tmpdir=None, **options)
