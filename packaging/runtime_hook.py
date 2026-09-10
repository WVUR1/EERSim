"""Make bundled conda DLLs available before NumPy/PyBullet/VTK import."""
import os
from pathlib import Path
import sys


if os.name == "nt" and getattr(sys, "frozen", False):
    _bundle = Path(sys._MEIPASS)
    _directories = [path for path in (_bundle, _bundle / "Library" / "bin") if path.is_dir()]
    # Retain handles for the whole process; exiting a handle's context manager
    # would remove the search directory before lazy extension imports happen.
    sys._eersim_dll_handles = [os.add_dll_directory(str(path)) for path in _directories]
    os.environ["PATH"] = os.pathsep.join([str(path) for path in _directories]
                                         + [os.environ.get("PATH", "")])
