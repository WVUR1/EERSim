"""Check GUI/runtime imports without opening a window or installing packages."""
from importlib import import_module
from pathlib import Path
import sys


def main() -> int:
    print(f"Python: {sys.executable}")
    if not (3, 11) <= sys.version_info[:2] < (3, 14):
        print("EERSim requires Python 3.11, 3.12, or 3.13.")
        return 1
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    import eersim  # Configure native DLL search paths before loading dependencies.
    failures = []
    for name, module in (("NumPy", "numpy"), ("MuJoCo", "mujoco"), ("PyBullet", "pybullet"),
                         ("PySide6", "PySide6.QtWidgets"),
                         ("VTK", "vtkmodules.vtkRenderingOpenGL2")):
        try:
            import_module(module)
        except (ImportError, OSError) as exc:
            failures.append(f"{name}: {exc}")
    if failures:
        print("Missing or unloadable simulator dependencies:")
        print("\n".join(f"  {failure}" for failure in failures))
        return 1
    print("Simulator runtime dependencies are ready.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
