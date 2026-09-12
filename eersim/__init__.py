"""EERKit's desktop robotics simulation laboratory."""

import os
from pathlib import Path
import sys

# Portable conda environments may be launched without `conda activate`.
# Keep their native BLAS/VTK DLL directories available for this process and
# child helpers. Retaining the handles is required by add_dll_directory().
_dll_handles = []
if os.name == "nt":
    _library_dirs = []
    for _prefix in dict.fromkeys((sys.prefix, sys.base_prefix)):
        _directory = Path(_prefix) / "Library" / "bin"
        if _directory.is_dir():
            _library_dirs.append(str(_directory))
            _dll_handles.append(os.add_dll_directory(str(_directory)))
    if _library_dirs:
        os.environ["PATH"] = os.pathsep.join(_library_dirs + [os.environ.get("PATH", "")])

__version__ = "0.1.2"
