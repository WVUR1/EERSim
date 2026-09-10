"""Writable per-user data for installed and frozen desktop applications."""
from __future__ import annotations

import os
from pathlib import Path
import sys


def user_data_dir() -> Path:
    """Return a stable platform directory; querying it creates no files."""
    if sys.platform == "win32":
        configured = os.environ.get("LOCALAPPDATA", "")
        root = Path(configured) if configured and Path(configured).is_absolute() else Path.home() / "AppData" / "Local"
    elif sys.platform == "darwin":
        root = Path.home() / "Library" / "Application Support"
    else:
        configured = os.environ.get("XDG_DATA_HOME", "")
        root = Path(configured) if configured and Path(configured).is_absolute() else Path.home() / ".local" / "share"
    return root / "EERSim"


def ensure_console_streams() -> Path | None:
    """Windowed executables have no console; keep CLI output and errors in a log.

    Existing console streams are preserved. The streams themselves retain the
    open file handle, including when both stdout and stderr share the same log.
    """
    if sys.stdout is not None and sys.stderr is not None:
        return None
    destination = user_data_dir() / "logs" / "eersim.log"
    destination.parent.mkdir(parents=True, exist_ok=True)
    stream = destination.open("a", encoding="utf-8", buffering=1)
    if sys.stdout is None:
        sys.stdout = stream
    if sys.stderr is None:
        sys.stderr = stream
    return destination
