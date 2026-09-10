"""Compile unchanged PROS C++ sources and exchange simulated hardware I/O.

Missing APIs fail compilation, linking, or raise a surfaced runtime exception.
Firmware ARM libraries and embedded LVGL cannot execute on a desktop host.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import shutil
import subprocess
import sys
import threading
import time
from typing import Callable


class BuildError(RuntimeError):
    """The workspace could not compile against the host PROS adapter."""


class RuntimeErrorNative(RuntimeError):
    """A robot task failed, did not yield, or exited unexpectedly."""


@dataclass(frozen=True)
class BuildResult:
    executable: Path
    workspace: Path
    compiler: str
    source_count: int
    cached: bool


_SUPPORTED_HEADERS = {
    "adi", "abstract_motor", "device", "distance", "error", "imu", "llemu",
    "misc", "motor_group", "motors", "rotation", "rtos", "serial", "version",
}
_DIGITAL = {name: index + 6 for index, name in enumerate(
    ("l1", "l2", "r1", "r2", "up", "down", "left", "right", "x", "b", "y", "a")
)}
_MODES = {
    "initialize": "initialize", "competition_initialize": "competition_initialize",
    "disabled": "disabled", "autonomous": "autonomous", "opcontrol": "opcontrol",
    "skills_auto": "autonomous", "match_auto": "autonomous", "skills_driver": "opcontrol",
    "driver_auto": "opcontrol", "driver": "opcontrol", "match_driver": "opcontrol",
}


def _compiler() -> str:
    configured = os.environ.get("CXX")
    if configured:
        found = shutil.which(configured)
        if found:
            return found
        raise BuildError(f"CXX={configured!r} is not an executable. Set CXX to the compiler path without flags.")
    for name in ("clang++", "g++", "cl"):
        found = shutil.which(name)
        if found:
            return found
    checkout = Path(__file__).resolve().parents[1]
    candidates = []
    if os.name == "nt":
        candidates += sorted((checkout / ".sim-tools").glob("llvm*/bin/clang++.exe"))
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    raise BuildError("No host C++ compiler found. Install LLVM/MinGW (Windows), Xcode command line tools (macOS), "
                     "or g++ (Linux), then add clang++/g++ to PATH. MSVC needs a Developer Command Prompt. "
                     "The PROS ARM compiler cannot build host executables. CXX may specify the compiler executable.")


def build_workspace(workspace: Path, log: Callable[[str], None] | None = None) -> Path:
    """Build src C++ files without changing workspace source.

    Staging makes quoted api.h includes resolve to the simulator overlay.
    Optional eersim.build.json: defines, include_dirs, sources (lists of strings).
    Paths must remain in the workspace. ARM firmware flags/libraries are not reused.
    """
    return build_workspace_result(workspace, log).executable


def build_workspace_result(workspace: Path, log: Callable[[str], None] | None = None) -> BuildResult:
    log = log or (lambda _: None)
    workspace = Path(workspace).expanduser().resolve()
    if not (workspace / "src").is_dir() or not (workspace / "include").is_dir():
        raise BuildError("Select a PROS workspace containing src/ and include/.")
    native = Path(__file__).resolve().parents[1] / "native"
    if not (native / "runtime.cpp").is_file():
        native = Path(sys.prefix) / "share" / "eersim" / "native"
    if not (native / "runtime.cpp").is_file():
        raise BuildError("The EER Sim installation is missing native/runtime.cpp. Reinstall the simulator package.")
    compiler = _compiler()
    config_file = workspace / "eersim.build.json"
    try:
        config = json.loads(config_file.read_text("utf-8")) if config_file.exists() else {}
    except (OSError, ValueError) as exc:
        raise BuildError(f"Cannot read eersim.build.json: {exc}") from exc
    if not isinstance(config, dict) or set(config) - {"defines", "include_dirs", "sources"}:
        raise BuildError("eersim.build.json supports only defines, include_dirs, and sources.")
    for key, value in config.items():
        if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
            raise BuildError(f"eersim.build.json {key} must be a list of strings.")
    sources = sorted(p for p in (workspace / "src").rglob("*") if p.suffix.lower() in {".cpp", ".cc", ".cxx"})
    c_sources = list((workspace / "src").rglob("*.c"))
    if c_sources:
        raise BuildError("C translation units are not supported by this C++ adapter: " + ", ".join(str(p.relative_to(workspace)) for p in c_sources))

    def local_path(value: str) -> Path:
        path = (workspace / value).resolve()
        if not path.is_relative_to(workspace):
            raise BuildError(f"Build path must stay inside the selected workspace: {value}")
        if not path.exists():
            raise BuildError(f"Build path does not exist: {value}")
        return path

    sources += [local_path(p) for p in config.get("sources", [])]
    sources = list(dict.fromkeys(sources))
    extra_includes = [local_path(p) for p in config.get("include_dirs", [])]
    if not sources:
        raise BuildError("No C++ source files found in the selected workspace's src/ directory.")
    digest = hashlib.sha256()
    digest.update((compiler + json.dumps(config, sort_keys=True)).encode())
    inputs = sources + [Path(__file__)] + list(native.glob("*.hpp")) + list(native.glob("*.cpp"))
    for folder in [workspace / "include", workspace / "src", *extra_includes]:
        inputs += sorted(p for p in folder.rglob("*") if p.is_file() and p.suffix.lower() in {".h", ".hpp", ".hh", ".inc", ".tpp"})
    for path in sorted(set(inputs)):
        if path.is_file():
            digest.update(str(path).encode())
            digest.update(path.read_bytes())
    build = workspace / ".eersim" / "build" / digest.hexdigest()[:16]
    executable = build / ("robot.exe" if os.name == "nt" else "robot")
    if executable.is_file():
        log(f"Using cached native robot ({len(sources)} source files).")
        return BuildResult(executable, workspace, compiler, len(sources), True)
    build.mkdir(parents=True, exist_ok=True)
    stage = build / "workspace"
    for folder in [workspace / "include", workspace / "src", *extra_includes]:
        shutil.copytree(folder, stage / folder.relative_to(workspace), dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns(".git", ".eersim", "__pycache__"))
    for source in sources:
        target = stage / source.relative_to(workspace)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    overlay = stage / "include"
    (overlay / "api.h").write_text('#pragma once\n#include "eersim_api.hpp"\n', "utf-8")
    shutil.copy2(native / "eersim_api.hpp", overlay / "eersim_api.hpp")
    pros = overlay / "pros"
    pros.mkdir(exist_ok=True)
    names = {p.stem for p in pros.iterdir() if p.suffix in {".h", ".hpp"}} | _SUPPORTED_HEADERS
    for name in names:
        for extension in ("h", "hpp"):
            body = '#pragma once\n#include "eersim_api.hpp"\n' if name in _SUPPORTED_HEADERS else (
                f'#error "PROS {name} is not supported by EER Sim. Add a modeled adapter before using this API."\n')
            (pros / f"{name}.{extension}").write_text(body, "utf-8")
    msvc = Path(compiler).name.lower() in {"cl", "cl.exe"}
    include_dirs = [overlay, native, *[stage / p.relative_to(workspace) for p in extra_includes]]
    staged_sources = [stage / source.relative_to(workspace) for source in sources]
    defines = ["EERSIM=1", "_USE_MATH_DEFINES", *config.get("defines", [])]
    if msvc:
        args = [compiler, "/nologo", "/std:c++17", "/EHsc", "/O2", "/MD", "/utf-8",
                *[f"/D{x}" for x in defines], *[f"/I{x}" for x in include_dirs],
                *map(str, staged_sources), str(native / "runtime.cpp"), f"/Fe:{executable}"]
    else:
        args = [compiler, "-std=c++17", "-O2", "-pthread", "-Wno-unused-parameter",
                *[f"-D{x}" for x in defines], *[f"-I{x}" for x in include_dirs],
                *map(str, staged_sources), str(native / "runtime.cpp"), "-o", str(executable)]
    log(f"Compiling {len(sources)} unchanged source files with {Path(compiler).name}…")
    (build / "command.json").write_text(json.dumps(args, indent=2), "utf-8")
    env = os.environ.copy()
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
    try:
        result = subprocess.run(args, cwd=build, env=env, capture_output=True, text=True,
                                encoding="utf-8", errors="replace", timeout=180,
                                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise BuildError(f"Native build could not finish: {exc}") from exc
    output = (result.stdout + result.stderr).strip()
    (build / "build.log").write_text(output, "utf-8")
    if output:
        log(output)
    if result.returncode or not executable.exists():
        raise BuildError("Native PROS build failed. Unsupported APIs or ARM libraries need a simulator adapter.\n" + output[-16000:])
    (build / "runtime-path.txt").write_text(str(Path(compiler).parent), "utf-8")
    log(f"Native robot ready: {executable.name}")
    return BuildResult(executable, workspace, compiler, len(sources), False)


def _number(value: object) -> str:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("Hardware snapshots must contain finite numbers.")
    return format(number, ".16g")


class NativeRuntime:
    """One robot process. A step runs all due tasks to their next pros::delay.

    Snapshot.time is the authoritative seconds clock. Restart to reset globals.
    """
    def __init__(self, executable: Path, timeout: float = 5.0):
        self.executable = Path(executable).resolve()
        self.timeout = timeout
        self._lines: queue.Queue[str | None] = queue.Queue()
        self._stderr: deque[str] = deque(maxlen=200)
        self._closed = False
        self._lock = threading.Lock()
        self._time = 0.0
        env = os.environ.copy()
        runtime_path = self.executable.parent / "runtime-path.txt"
        if runtime_path.exists():
            env["PATH"] = runtime_path.read_text("utf-8") + os.pathsep + env.get("PATH", "")
        try:
            self.process = subprocess.Popen([str(self.executable)], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding="utf-8",
                errors="replace", bufsize=1, cwd=self.executable.parent, env=env,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except OSError as exc:
            raise RuntimeErrorNative(f"Cannot start native robot: {exc}") from exc

        def read_stdout() -> None:
            try:
                assert self.process.stdout
                for line in self.process.stdout:
                    self._lines.put(line.rstrip("\r\n"))
            finally:
                self._lines.put(None)

        def read_stderr() -> None:
            assert self.process.stderr
            for line in self.process.stderr:
                self._stderr.append(line.rstrip())

        threading.Thread(target=read_stdout, daemon=True, name="eersim-native-protocol").start()
        threading.Thread(target=read_stderr, daemon=True, name="eersim-native-log").start()

    @property
    def logs(self) -> list[str]:
        return list(self._stderr)

    def step(self, snapshot: dict, mode: str = "disabled", dt: float = 1 / 240,
             controller: dict | None = None) -> dict:
        key = mode.strip().lower().replace(" ", "_")
        if key not in _MODES:
            raise ValueError(f"Unknown competition mode: {mode}")
        mode = _MODES[key]
        controller = controller or {}
        analog = controller.get("analog", controller)
        digital = {str(k).lower(): v for k, v in controller.get("digital", controller.get("buttons", {})).items()}
        axes = [max(-127, min(127, int(analog.get(k, 0)))) for k in ("left_x", "left_y", "right_x", "right_y")]
        buttons = sum(1 << index for name, index in _DIGITAL.items() if digital.get(name, False))
        now = float(snapshot.get("time", self._time))
        lines = [f"STEP {_number(now)} {_number(dt)} {mode}"]
        for port, motor in snapshot.get("motors", {}).items():
            lines.append(f"M {int(port)} {_number(motor.get('position', 0))} {_number(motor.get('velocity', 0))}")
        for port, sensor in snapshot.get("rotation", {}).items():
            lines.append(f"R {int(port)} {_number(sensor.get('position', 0))} {_number(sensor.get('velocity', 0))}")
        imu = snapshot.get("imu", {})
        lines.append(f"I {_number(imu.get('heading', 0))} {_number(imu.get('rotation', imu.get('heading', 0)))}")
        for port, mm in snapshot.get("distance", {}).items():
            lines.append(f"D {int(port)} {_number(mm)}")
        otos = snapshot.get("otos", (0, 0, 0))
        if len(otos) != 3:
            raise ValueError("OTOS snapshot must have x inches, y inches, heading degrees.")
        lines += ["O " + " ".join(_number(v) for v in otos), "A " + " ".join(map(str, axes)), f"B {buttons}", "END"]
        with self._lock:
            if self._closed:
                raise RuntimeErrorNative("Robot runtime is closed.")
            try:
                assert self.process.stdin
                self.process.stdin.write("\n".join(lines) + "\n")
                self.process.stdin.flush()
            except (OSError, BrokenPipeError) as exc:
                raise RuntimeErrorNative("Robot process exited.\n" + "\n".join(self.logs[-20:])) from exc
            result: dict = {"motors": {}, "adi": {}, "lcd": {}, "diagnostics": [], "initialized": False, "callback_done": False, "time": now}
            deadline = time.monotonic() + self.timeout
            while True:
                try:
                    line = self._lines.get(timeout=max(0, deadline - time.monotonic()))
                except queue.Empty as exc:
                    self.close()
                    raise RuntimeErrorNative("Native robot did not yield before the runtime timeout. Simulation stopped.\n" + "\n".join(self.logs[-20:])) from exc
                if line is None:
                    raise RuntimeErrorNative("Native robot exited unexpectedly.\n" + "\n".join(self.logs[-20:]))
                if line == "END":
                    break
                parts = line.split()
                if not parts:
                    continue
                tag = parts[0]
                if tag == "STATE":
                    result.update(initialized=bool(int(parts[1])), callback_done=bool(int(parts[2])), time=float(parts[3]))
                elif tag == "M":
                    result["motors"][parts[1]] = {"mode": ("voltage", "velocity", "position")[int(parts[2])],
                        "value": float(parts[3]), "velocity": float(parts[4]),
                        "brake": ("coast", "brake", "hold")[int(parts[5])],
                        "gearset": (100, 200, 600)[int(parts[6])]}
                elif tag == "P":
                    result["adi"][parts[1]] = bool(int(parts[2]))
                elif tag == "L":
                    result["lcd"][parts[1]] = "" if parts[2] == "-" else bytes.fromhex(parts[2]).decode("utf-8", "replace")
                elif tag in {"LOG", "ERROR"}:
                    message = "" if parts[1] == "-" else bytes.fromhex(parts[1]).decode("utf-8", "replace")
                    if tag == "ERROR":
                        self.close()
                        raise RuntimeErrorNative(message)
                    result["diagnostics"].append(message)
                else:
                    self._stderr.append(line)
            self._time = now + dt
            return result

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self.process.poll() is None:
            try:
                assert self.process.stdin
                self.process.stdin.write("QUIT\n")
                self.process.stdin.flush()
                self.process.wait(timeout=0.3)
            except (OSError, subprocess.TimeoutExpired):
                self.process.kill()
                self.process.wait(timeout=2)
        for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
            if stream:
                stream.close()

    def __enter__(self) -> "NativeRuntime":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
