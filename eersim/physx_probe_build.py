"""Build the small pinned PhysX scene inspector; no SDK/runtime installation.

Windows uses the Microsoft C++ ABI, even when Clang comes from LLVM-MinGW.
Only standard headers are downloaded, hash checked and extracted into the cache.
Linux uses the installed C++ compiler and the vendored public NVIDIA headers.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile
import urllib.request
import zipfile

NATIVE = Path(__file__).resolve().parent / "_native"
TAG = "ovphysx-0.5.11"
LIBRARY_NAME = "physx_scene_probe.dll" if os.name == "nt" else "physx_scene_probe.so"
# Official Visual Studio 17 release manifest payloads. They are build inputs;
# none of the Microsoft headers or packages are redistributed with EERSim.
_DOWNLOADS = {
    "crt.vsix": (
        "https://download.visualstudio.microsoft.com/download/pr/c610cd8c-801b-44b8-a80a-82cc382aeb43/852382a9aa73502b7849c1bcadfb603ba7175c4e8b60e6aba03c7de711d4ece5/Microsoft.VC.14.44.17.14.CRT.Headers.base.vsix",
        "852382a9aa73502b7849c1bcadfb603ba7175c4e8b60e6aba03c7de711d4ece5"),
    "ucrt.msi": (
        "https://download.visualstudio.microsoft.com/download/pr/fb762014-2a36-4840-8a24-7ff61d1ee019/975874342d151b32b3f4e0eac90db7c8/universal%20crt%20headers%20libraries%20and%20sources-x86_en-us.msi",
        "54448641eef058318b540188245a62982f97ccc462c0c1c1cf3c6d771e26f214"),
    "ucrt.cab": (
        "https://download.visualstudio.microsoft.com/download/pr/4a925113-4714-47d1-8e22-1d149bb57d9f/6d97f0ae0c61820e66c6ac988de5f096/b2f03f34ff83ec013b9e45c7cd8e8a73.cab",
        "d0edd2447f7314f283f61b3db56eb65e01ca2b6f24bd4be52aaa285466fbc1ff"),
}


def _run(args: list[str], *, env: dict[str, str] | None = None) -> str:
    result = subprocess.run(args, capture_output=True, text=True, encoding="utf-8", errors="replace",
                            timeout=120, env=env,
                            creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
    if result.returncode:
        raise RuntimeError(f"PhysX verification helper build failed ({Path(args[0]).name}):\n"
                           + result.stdout[-4000:] + result.stderr[-4000:])
    return result.stdout


def _compiler(configured: str | None) -> Path:
    selected = configured or os.environ.get("EERSIM_PHYSX_CXX") or os.environ.get("CXX")
    if selected:
        found = shutil.which(selected)
        if not found:
            raise RuntimeError(f"PhysX helper compiler is unavailable: {selected!r}; supply an executable without flags")
        return Path(found).resolve()
    candidates: list[Path | str] = []
    if os.name == "nt":
        root = Path(__file__).resolve().parent.parent
        candidates.append(root / "compiler/bin/clang++.exe")
        candidates.extend(sorted((root / ".sim-tools").glob("llvm*/bin/clang++.exe"), reverse=True))
        candidates.extend(["clang++", "clang-cl", "cl"])
    else:
        candidates.extend(["c++", "g++", "clang++"])
    for candidate in candidates:
        found = shutil.which(os.fspath(candidate))
        if found:
            return Path(found).resolve()
    raise RuntimeError("A C++ compiler is needed once to build the PhysX verification helper. "
                       "Run setup.ps1 on Windows, or install/use a Linux C++ compiler; "
                       "EERSIM_PHYSX_CXX can select its executable. The Windows installer includes a prebuilt helper.")


def _download(name: str, directory: Path) -> Path:
    url, digest = _DOWNLOADS[name]
    path = directory / name
    if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest:
        return path
    with urllib.request.urlopen(url, timeout=60) as response:
        data = response.read(8 * 1024 * 1024 + 1)
    if len(data) > 8 * 1024 * 1024 or hashlib.sha256(data).hexdigest() != digest:
        raise RuntimeError(f"Microsoft header download failed SHA256 verification: {name}")
    path.write_bytes(data)
    return path


def _windows_headers(cache: Path) -> tuple[Path, Path]:
    cache.mkdir(parents=True, exist_ok=True)
    msvc = cache / "msvc/Contents/VC/Tools/MSVC/14.44.35207/include"
    ucrt = cache / "ucrt"
    if (msvc / "typeinfo").is_file() and (ucrt / "math.h").is_file():
        return msvc, ucrt
    packages = {name: _download(name, cache) for name in _DOWNLOADS}
    with zipfile.ZipFile(packages["crt.vsix"]) as archive:
        destination = (cache / "msvc").resolve()
        for member in archive.infolist():
            if not (destination / member.filename).resolve().is_relative_to(destination):
                raise RuntimeError("Header archive contains an unsafe path")
        archive.extractall(destination)
    cab = cache / "cab"
    cab.mkdir(exist_ok=True)
    system_root = Path(os.environ.get("SystemRoot", ""))
    expand = system_root / "System32/expand.exe"
    powershell = system_root / "System32/WindowsPowerShell/v1.0/powershell.exe"
    if not expand.is_file() or not powershell.is_file():
        raise RuntimeError("Windows system tools are unavailable; supply --msvc-include and --ucrt-include")
    _run([str(expand), "-F:*", str(packages["ucrt.cab"]), str(cab)])
    # Read the MSI File table without installing or registering the MSI.
    script = cache / "read-header-map.ps1"
    script.write_text('''param([string]$Msi)
$ErrorActionPreference = 'Stop'
$installer = New-Object -ComObject WindowsInstaller.Installer
$db = $installer.OpenDatabase($Msi, 0)
$view = $db.OpenView('SELECT File, FileName, Sequence FROM File')
$view.Execute()
$entries = @()
while ($record = $view.Fetch()) {
    $sequence = $record.IntegerData(3)
    if ($sequence -gt 509 -and $sequence -le 571) {
        $entries += @{ id=$record.StringData(1); name=($record.StringData(2) -split '\\|')[-1] }
    }
}
ConvertTo-Json -InputObject $entries -Compress
''', encoding="utf-8")
    entries = json.loads(_run([str(powershell), "-NoProfile", "-NonInteractive", "-ExecutionPolicy", "Bypass",
                              "-File", str(script), "-Msi", str(packages["ucrt.msi"])]))
    ucrt.mkdir(exist_ok=True)
    for item in entries:
        name, identifier = item["name"], item["id"]
        if Path(name).name != name or Path(identifier).name != identifier:
            raise RuntimeError("Unexpected path in official UCRT header map")
        shutil.copyfile(cab / identifier, ucrt / name)
    if len(entries) != 62 or not (msvc / "typeinfo").is_file() or not (ucrt / "math.h").is_file():
        raise RuntimeError("Incomplete Microsoft standard header extraction")
    return msvc, ucrt


def source_digest() -> str:
    """Verify pinned headers and derive a cache identity from all native inputs."""
    manifest = json.loads((NATIVE / "physx/headers-manifest.json").read_text(encoding="utf-8"))
    if manifest.get("tag") != TAG:
        raise RuntimeError("PhysX verification headers do not match the pinned runtime")
    digest = hashlib.sha256((NATIVE / "physx_scene_probe.cpp").read_bytes())
    for name, expected in sorted(manifest["sha256"].items()):
        path = (NATIVE / "physx/include" / name).resolve()
        if not path.is_relative_to((NATIVE / "physx/include").resolve()):
            raise RuntimeError("Invalid header manifest path")
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError(f"Vendored PhysX header was modified: {name}")
        digest.update(name.encode() + actual.encode())
    return digest.hexdigest()[:20]


def build_probe(output: Path, *, cxx: str | None = None, cache: Path | None = None,
                msvc_include: Path | None = None, ucrt_include: Path | None = None) -> Path:
    if sys.platform not in {"win32", "linux"} or platform.machine().lower() not in {"amd64", "x86_64"}:
        raise RuntimeError("The pinned PhysX verification helper supports Windows/Linux x86-64")
    source_digest()
    output = output.expanduser().resolve()
    if output.suffix.lower() != (".dll" if os.name == "nt" else ".so"):
        raise ValueError("Output must have the platform native-library extension")
    output.parent.mkdir(parents=True, exist_ok=True)
    compiler = _compiler(cxx)
    env = os.environ.copy()
    env["PATH"] = str(compiler.parent) + os.pathsep + env.get("PATH", "")
    with tempfile.TemporaryDirectory(prefix="probe-build-", dir=output.parent) as temporary:
        work = Path(temporary)
        built = work / LIBRARY_NAME
        source, include = NATIVE / "physx_scene_probe.cpp", NATIVE / "physx/include"
        if os.name == "nt":
            if not compiler.name.lower().startswith(("clang", "cl.")):
                raise RuntimeError("Windows PhysX requires Microsoft ABI; use Clang or MSVC, not MinGW GCC")
            if bool(msvc_include) != bool(ucrt_include):
                raise ValueError("Supply both Microsoft and UCRT include paths")
            if msvc_include is None:
                msvc_include, ucrt_include = _windows_headers((cache or output.parent) / "microsoft-headers")
            obj = work / "probe.obj"
            if compiler.name.lower() == "cl.exe":
                _run([str(compiler), "/nologo", "/std:c++17", "/O2", "/DNDEBUG", "/D_WINDOWS", "/GS-", "/c",
                      f"/I{include}", f"/I{msvc_include}", f"/I{ucrt_include}", str(source), f"/Fo{obj}"], env=env)
                linker = shutil.which("link.exe", path=env["PATH"])
                command = [linker] if linker else []
            else:
                _run([str(compiler), "--driver-mode=g++", "--target=x86_64-pc-windows-msvc", "-std=c++17", "-O2",
                      "-DNDEBUG", "-D_WINDOWS", "-Wno-return-type-c-linkage", "-I", str(include),
                      "-isystem", str(msvc_include), "-isystem", str(ucrt_include),
                      "-c", str(source), "-o", str(obj)], env=env)
                linker = compiler.parent / "lld-link.exe"
                fallback = compiler.parent / "ld.lld.exe"
                command = [str(linker)] if linker.is_file() else [str(fallback), "-flavor", "link"] if fallback.is_file() else []
            if not command:
                raise RuntimeError("Microsoft-compatible linker missing: use LLVM with lld-link/ld.lld or MSVC link.exe")
            _run(command + ["/dll", "/noentry", "/nodefaultlib", "/machine:x64", f"/out:{built}", str(obj)], env=env)
        else:
            _run([str(compiler), "-std=c++17", "-O2", "-DNDEBUG", "-fPIC", "-shared", "-fvisibility=hidden",
                  "-I", str(include), str(source), "-o", str(built)], env=env)
        os.replace(built, output)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cxx")
    parser.add_argument("--cache", type=Path)
    parser.add_argument("--msvc-include", type=Path)
    parser.add_argument("--ucrt-include", type=Path)
    args = parser.parse_args()
    print(build_probe(args.output, cxx=args.cxx, cache=args.cache,
                      msvc_include=args.msvc_include, ucrt_include=args.ucrt_include))


if __name__ == "__main__":
    main()
