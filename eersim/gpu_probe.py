"""Probe native OpenGL in an isolated process before using a graphics driver."""
from __future__ import annotations

from functools import lru_cache
import json
import os
from pathlib import Path
import subprocess
import sys


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
        result = subprocess.run([sys.executable, "-m", "eersim.gpu_probe"],
                                capture_output=True, text=True, encoding="utf-8", errors="replace",
                                env=environment, timeout=25,
                                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        for line in reversed(result.stdout.splitlines()):
            if line.startswith("EERSIM_GPU "):
                info = json.loads(line[len("EERSIM_GPU "):])
                if result.returncode == 0:
                    return info
        error = (result.stderr or result.stdout or f"Graphics probe exited with code {result.returncode}").strip()
    except (OSError, subprocess.TimeoutExpired, ValueError) as exc:
        error = str(exc)
    return {"available": False, "hardware_accelerated": False, "error": error[-1500:]}


if __name__ == "__main__":
    try:
        result = inspect_context()
    except Exception as exc:
        result = {"available": False, "hardware_accelerated": False, "error": str(exc)}
    print("EERSIM_GPU " + json.dumps(result), flush=True)
