"""Collect the complete pinned Windows PhysX runtime without loading its DLLs."""
from importlib.metadata import distribution
from pathlib import Path


GPU_PACKAGES = {"ovphysx": "0.5.11", "ovstage": "0.1.1.355824"}
_REQUIRED = {
    "ovphysx": ("__init__.py", "lib/ovphysx.dll", "lib/config.toml", "plugins/usd", "plugins/gpu"),
    "ovstage": ("__init__.py", "bin/ovstage.dll", "bin/plugins"),
}


def collect_gpu_payload(find_distribution=distribution):
    """Return PyInstaller DATA entries and Python hidden imports.

    Add these entries to EXE/COLLECT after Analysis. Vendor DLLs and their
    plugin/config closure must keep the wheel's complete sibling layout;
    they are loaded by ctypes and the NVIDIA plugin loader at runtime.
    Distribution metadata and license directories are collected separately
    by copy_metadata in EERSim.spec.
    """
    payload, hiddenimports = [], []
    for package, required_version in GPU_PACKAGES.items():
        installed = find_distribution(package)
        if installed.version != required_version:
            raise ValueError(f"The Windows bundle requires {package}=={required_version}; "
                             f"found {installed.version}. Install packaging/requirements-build.txt.")
        root = Path(installed.locate_file(package)).resolve()
        for relative in _REQUIRED[package]:
            if not (root / relative).exists():
                raise ValueError(f"Incomplete {package} Windows runtime: missing {relative}")
        for source in sorted(root.rglob("*")):
            if not source.is_file() or "__pycache__" in source.relative_to(root).parts:
                continue
            relative = source.relative_to(root)
            payload.append(((Path(package) / relative).as_posix(), str(source), "DATA"))
            if source.suffix != ".py":
                continue
            # Discover modules statically so collection does not initialize
            # CUDA, USD, or a native plugin while building the application.
            parents = list(relative.parent.parts)
            if not all((root.joinpath(*parents[:index + 1]) / "__init__.py").is_file()
                       for index in range(len(parents))):
                continue
            parts = [package, *parents]
            if source.stem != "__init__":
                parts.append(source.stem)
            if all(part.isidentifier() for part in parts):
                hiddenimports.append(".".join(parts))
    return payload, sorted(set(hiddenimports))
