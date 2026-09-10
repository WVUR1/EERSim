"""Probe native OpenGL in an isolated process before using a graphics driver."""
from __future__ import annotations

from functools import lru_cache
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def inspect_context() -> dict:
    import vtkmodules.vtkRenderingOpenGL2  # registers the platform OpenGL backend
    from vtkmodules.vtkRenderingCore import vtkRenderer, vtkRenderWindow

    window = vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(32, 32)
    window.AddRenderer(vtkRenderer())
    try:
        window.Render()
        capabilities = window.ReportCapabilities() or ""
        details = {}
        for line in capabilities.splitlines():
            for label, key in (("OpenGL vendor string:", "vendor"),
                               ("OpenGL renderer string:", "device"),
                               ("OpenGL version string:", "version")):
                if label.lower() in line.lower():
                    details[key] = line.split(":", 1)[1].strip()
        available = bool(window.SupportsOpenGL())
        device = details.get("device", "unknown OpenGL renderer")
        software = any(word in device.lower() for word in
                       ("llvmpipe", "softpipe", "swrast", "software", "gdi generic", "swiftshader", "basic render"))
        return {"available": available, "hardware_accelerated": available and not software and "device" in details,
                **details, "error": "" if available else window.GetOpenGLSupportMessage()}
    finally:
        window.Finalize()


@lru_cache(maxsize=1)
def probe_gpu() -> dict:
    """Driver failures cannot take down the desktop; cache the result per process."""
    environment = os.environ.copy()
    package_root = str(Path(__file__).resolve().parent.parent)
    environment["PYTHONPATH"] = package_root + os.pathsep + environment.get("PYTHONPATH", "")
    try:
        # A PyInstaller executable cannot run Python's -m switch, and a
        # windowed executable has no stdout. Use a private file protocol in
        # that case; its entry point dispatches before importing the GUI.
        with tempfile.TemporaryDirectory(prefix="eersim-gpu-") as temporary:
            output = Path(temporary) / "probe.json"
            frozen = bool(getattr(sys, "frozen", False))
            command = ([sys.executable, "--gpu-probe-output", str(output)] if frozen
                       else [sys.executable, "-m", "eersim.gpu_probe"])
            result = subprocess.run(command, capture_output=True, text=True,
                                    encoding="utf-8", errors="replace", env=environment, timeout=25,
                                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            if result.returncode == 0:
                if frozen and output.is_file():
                    return _validated_result(json.loads(output.read_text(encoding="utf-8")))
                if not frozen:
                    for line in reversed((result.stdout or "").splitlines()):
                        if line.startswith("EERSIM_GPU "):
                            return _validated_result(json.loads(line[len("EERSIM_GPU "):]))
        error = (result.stderr or result.stdout or f"Graphics probe exited with code {result.returncode}").strip()
        if result.returncode == 0:
            error = "Graphics probe produced no valid result. " + error
    except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
        error = str(exc)
    return {"available": False, "hardware_accelerated": False, "error": error[-1500:]}


def _validated_result(info: object) -> dict:
    if not isinstance(info, dict) or any(type(info.get(key)) is not bool for key in ("available", "hardware_accelerated")):
        raise ValueError("Graphics probe returned malformed capability data")
    return info


def _inspect_safely() -> dict:
    try:
        return inspect_context()
    except Exception as exc:
        return {"available": False, "hardware_accelerated": False, "error": str(exc)}


def write_probe_result(path: str | Path) -> int:
    """Private frozen-process entry point; never starts Qt or needs stdout."""
    try:
        Path(path).write_text(json.dumps(_inspect_safely()), encoding="utf-8")
    except (OSError, ValueError):
        return 1
    return 0


if __name__ == "__main__":
    print("EERSIM_GPU " + json.dumps(_inspect_safely()), flush=True)
