"""Prepare a passive PROS logger copy and capture its versioned serial telemetry.

No API in this module starts a robot program or commands a motor. Upload is a
separate explicit operation. Stop stops reception, not the program on the Brain.
All command APIs are synchronous and suitable for a GUI worker thread.
"""
from __future__ import annotations

import codecs
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import shutil
import signal
import stat
import subprocess
import sys
import threading
import time
from typing import Callable, Iterable
import uuid


class CaptureError(RuntimeError):
    """An actionable preparation, CLI, or protocol error."""


class CaptureCancelled(CaptureError):
    pass


@dataclass(frozen=True)
class PreparedCapture:
    workspace: Path
    source_workspace: Path
    ports: tuple[int, ...]
    sample_ms: int
    metadata_path: Path


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    cancelled: bool
    timed_out: bool
    output_tail: str
    command: tuple[str, ...]

    @property
    def ok(self) -> bool:
        return self.returncode == 0 and not self.cancelled and not self.timed_out


@dataclass(frozen=True)
class HardwareSample:
    timestamp_ms: int
    port: int
    position_deg: float | None
    rpm: float | None
    voltage_mv: float | None
    current_ma: float | None
    battery_mv: float | None


_EXCLUDED = frozenset({".git", ".eersim", ".sim-tools", ".d", "bin", "build", "dist", "cache", ".cache",
    "__pycache__", ".pytest_cache", ".mypy_cache", ".venv", "venv", "node_modules"})
_METADATA = "eersim-tuning-capture.json"
_LOGGER = "eersim_tuning_logger.c"
_RENAMED = "eersim_tuning_user_initialize"
_MAX_COPY_BYTES = 2 * 1024 ** 3
_MAX_COPY_FILES = 50000
_MAX_LINE = 16384
_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")


def _ports(values: Iterable[int]) -> tuple[int, ...]:
    result = []
    for value in values:
        if type(value) is not int or not 1 <= abs(value) <= 21:
            raise CaptureError("Motor ports must be integers from 1 to 21 (signed ports are accepted).")
        if abs(value) not in result:
            result.append(abs(value))
    if not result:
        raise CaptureError("Select at least one motor port.")
    return tuple(sorted(result))


def _check_cancel(cancel: threading.Event | None) -> None:
    if cancel is not None and cancel.is_set():
        raise CaptureCancelled("Preparing the logger copy was cancelled.")


def _is_link(path: Path) -> bool:
    info = path.lstat()
    return stat.S_ISLNK(info.st_mode) or bool(
        getattr(info, "st_file_attributes", 0) & getattr(stat, "FILE_ATTRIBUTE_REPARSE_POINT", 0x400))


def _regular(path: Path, root: Path) -> None:
    if _is_link(path) or not path.resolve(strict=True).is_relative_to(root):
        raise CaptureError(f"Logger copies cannot follow symlinks or junctions: {path.name}")
    if not path.is_file():
        raise CaptureError(f"Expected a regular workspace file: {path.name}")


def _tokens(source: str) -> list[tuple[str, int, int]]:
    """Token offsets for callback identification; skip comments and literals.

    This is deliberately not a C++ parser. Unsupported macro/conditional or
    qualified callback definitions are rejected instead of rewritten by guess.
    """
    token = re.compile(r'''(?P<directive>^[\t ]*\#(?:\\\r?\n|[^\n])*)|
        (?P<space>[\t \r\f\v]+|\n)|(?P<line>//[^\n]*)|(?P<block>/\*[\s\S]*?\*/)|
        (?P<raw>(?:u8|u|U|L)?R"(?P<delim>[^ ()\\\t\r\n]{0,16})\([\s\S]*?\)(?P=delim)")|
        (?P<string>(?:u8|u|U|L)?"(?:\\[\s\S]|[^"\\])*")|
        (?P<char>(?:u8|u|U|L)?'(?:\\[\s\S]|[^'\\])*')|
        (?P<word>[A-Za-z_][A-Za-z_0-9]*)|(?P<other>.)''', re.VERBOSE | re.MULTILINE)
    result = []
    for match in token.finditer(source):
        if match.lastgroup in {"space", "line", "block", "directive"}:
            continue
        value = match.group()
        if match.lastgroup in {"string", "raw", "char"}:
            value = '"C"' if value == '"C"' else "<literal>"
        result.append((value, match.start(), match.end()))
    return result


def _callback_offsets(source: str) -> list[tuple[int, int]]:
    tokens = _tokens(source)
    scopes: list[bool] = []
    found = []
    for index, (word, begin, end) in enumerate(tokens):
        if word == "{":
            scopes.append(index >= 2 and [v[0] for v in tokens[index - 2:index]] == ["extern", '"C"'])
        elif word == "}":
            if scopes:
                scopes.pop()
        elif word == "initialize" and not any(not linkage for linkage in scopes):
            preceding = [v[0] for v in tokens[max(0, index - 3):index]]
            following = [v[0] for v in tokens[index + 1:index + 5]]
            if (preceding and preceding[-1] == "void" and
                    (following[:3] == ["(", ")", "{"] or following == ["(", "void", ")", "{"])):
                if "static" in preceding or "inline" in preceding:
                    raise CaptureError("initialize must be a normal externally linked PROS callback.")
                found.append((begin, end))
    return found


def _logger_source(ports: tuple[int, ...], sample_ms: int) -> str:
    # Kept in this Python module so wheels/frozen apps need no extra template file.
    # VEX encoder specifications: https://kb.vex.com/hc/en-us/articles/360060929971
    # PROS C telemetry: https://pros.cs.purdue.edu/v5/api/c/motors.html
    return r'''/* Generated EERSim passive logger, protocol EERSIM_TUNE version 1.
 * Motor units, gearing, reversal, tare and outputs are never modified.
 * Start the uploaded program yourself. Closing the terminal does not stop it.
 */
#include "pros/motors.h"
#include "pros/misc.h"
#include "pros/rtos.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

extern void eersim_tuning_user_initialize(void);
static const int8_t ports[] = {PORT_VALUES};
static task_t logger_task = NULL;

static double encoder_degrees(double position, motor_encoder_units_e_t units,
                              motor_gearset_e_t gearing) {
    if (!isfinite(position)) return NAN;
    switch (units) {
        case E_MOTOR_ENCODER_DEGREES: return position;
        case E_MOTOR_ENCODER_ROTATIONS: return position * 360.0;
        case E_MOTOR_ENCODER_COUNTS:
            switch (gearing) {
                case E_MOTOR_GEARSET_36: return position * (360.0 / 1800.0);
                case E_MOTOR_GEARSET_18: return position * (360.0 / 900.0);
                case E_MOTOR_GEARSET_06: return position * (360.0 / 300.0);
                default: return NAN;
            }
        default: return NAN;
    }
}

static const char *number(char *buffer, size_t size, double value) {
    if (!isfinite(value)) return "null";
    snprintf(buffer, size, "%.9g", value);
    return buffer;
}

static double integer_reading(int32_t value) {
    return value == INT32_MAX ? NAN : (double)value;
}

static void log_scan(void) {
    /* One scan timestamp; the API calls are sequential, not simultaneous. */
    uint32_t timestamp = millis();
    double battery = integer_reading(battery_get_voltage());
    for (unsigned i = 0; i < sizeof(ports) / sizeof(ports[0]); ++i) {
        int8_t port = ports[i];
        motor_encoder_units_e_t units = motor_get_encoder_units(port);
        motor_gearset_e_t gearing = motor_get_gearing(port);
        double position = motor_get_position(port);
        double rpm = motor_get_actual_velocity(port);
        double voltage = integer_reading(motor_get_voltage(port));
        double current = integer_reading(motor_get_current_draw(port));
        /* Do not mix conversions if user code changed units/gearing mid-read. */
        if (units != motor_get_encoder_units(port) || gearing != motor_get_gearing(port)) {
            position = NAN;
        }
        position = encoder_degrees(position, units, gearing);
        char p[48], r[48], v[48], c[48], b[48], line[320];
        int length = snprintf(line, sizeof(line), "EERSIM_TUNE,1,%lu,%d,%s,%s,%s,%s,%s\n",
            (unsigned long)timestamp, (int)port,
            number(p, sizeof(p), position), number(r, sizeof(r), rpm),
            number(v, sizeof(v), voltage), number(c, sizeof(c), current),
            number(b, sizeof(b), battery));
        /* A single stdio write per line avoids logger-owned fragment mixing. */
        if (length > 0 && (size_t)length < sizeof(line)) fwrite(line, 1, (size_t)length, stdout);
    }
    fflush(stdout);
}

static void logger_main(void *unused) {
    (void)unused;
    for (;;) {
        uint32_t started = millis();
        log_scan();
        uint32_t elapsed = millis() - started;
        /* Skip missed deadlines instead of producing catch-up bursts. */
        task_delay(elapsed < SAMPLE_INTERVAL ? SAMPLE_INTERVAL - elapsed : SAMPLE_INTERVAL);
    }
}

void initialize(void) {
    eersim_tuning_user_initialize();
    if (logger_task == NULL) {
        logger_task = task_create(logger_main, NULL, TASK_PRIORITY_DEFAULT - 1,
                                  TASK_STACK_DEPTH_DEFAULT, "EERSim telemetry");
        if (logger_task == NULL) fputs("EERSIM_TUNE_ERROR: cannot create logger task\n", stderr);
    }
}
'''.replace("PORT_VALUES", ", ".join(map(str, ports))).replace("SAMPLE_INTERVAL", str(sample_ms) + "u")


def prepare_capture(workspace: str | Path, destination: str | Path,
                    ports: Iterable[int], sample_ms: int = 20, *,
                    cancel: threading.Event | None = None,
                    log: Callable[[str], None] | None = None) -> PreparedCapture:
    """Copy a standard PROS 4 workspace and add read-only telemetry.

    The destination must not exist or overlap the source. Links/junctions are
    rejected; firmware libraries are copied, caches and binaries are excluded.
    The original initialize definition is renamed only in the disposable copy.
    """
    selected = _ports(ports)
    if type(sample_ms) is not int or not 20 <= sample_ms <= 1000:
        raise CaptureError("The logging interval must be 20 to 1000 milliseconds.")
    original = Path(workspace).expanduser()
    if not original.is_dir() or _is_link(original):
        raise CaptureError("Select a real PROS workspace directory, not a symlink or junction.")
    source = original.resolve(strict=True)
    dest_input = Path(destination).expanduser().absolute()
    dest = dest_input.resolve()
    if dest == source or dest.is_relative_to(source) or source.is_relative_to(dest):
        raise CaptureError("The logger destination must be outside the original workspace.")
    if dest_input.exists() or dest_input.is_symlink():
        raise CaptureError("The logger destination already exists; choose a new folder.")
    for required in ("project.pros", "Makefile", "common.mk", "include/pros/motors.h"):
        candidate = source / required
        if not candidate.is_file():
            raise CaptureError(f"A standard PROS workspace is required (missing {required}).")
        _regular(candidate, source)
    try:
        project = json.loads((source / "project.pros").read_text("utf-8-sig"))
        target = project.get("py/state", project).get("target")
    except (ValueError, AttributeError, OSError) as exc:
        raise CaptureError("Cannot read the PROS project metadata.") from exc
    if target != "v5":
        raise CaptureError("Passive capture supports VEX V5 PROS workspaces.")
    motors_header = (source / "include/pros/motors.h").read_text("utf-8-sig")
    if re.search(r"\bmotor_(?:is|set)_reversed\s*\(", motors_header):
        raise CaptureError("This workspace uses legacy PROS motor reversal APIs. Update to PROS 4 before passive capture; reversal cannot be inferred safely.")
    makefile = (source / "Makefile").read_text("utf-8-sig")
    common = (source / "common.mk").read_text("utf-8-sig")
    if "common.mk" not in makefile or "$(CEXTS)" not in common or "$(SRCDIR)" not in common:
        raise CaptureError("The logger requires the standard PROS src/ C-source build rules.")
    files = []
    definitions = []
    total = 0
    for directory, dirs, names in os.walk(source, followlinks=False):
        _check_cancel(cancel)
        root = Path(directory)
        dirs[:] = sorted(name for name in dirs if name.lower() not in _EXCLUDED)
        for name in dirs:
            child = root / name
            if _is_link(child) or not child.resolve(strict=True).is_relative_to(source):
                raise CaptureError(f"Logger copies cannot follow symlinks or junctions: {child.relative_to(source)}")
        for name in sorted(names):
            path = root / name
            if name.lower() in _EXCLUDED:
                continue
            _regular(path, source)
            relative = path.relative_to(source)
            if relative.as_posix() in {_METADATA, "src/" + _LOGGER}:
                raise CaptureError("This workspace is already instrumented for EERSim capture.")
            info = path.stat()
            total += info.st_size
            files.append((relative, info.st_size))
            if len(files) > _MAX_COPY_FILES or total > _MAX_COPY_BYTES:
                raise CaptureError("The logger workspace exceeds the 2 GiB / 50,000 file copy limit.")
            if relative.parts[0] == "src" and path.suffix.lower() in {".c", ".cpp", ".cc", ".c++"}:
                if info.st_size > 16 * 1024 ** 2:
                    raise CaptureError(f"Source file exceeds the 16 MiB callback-inspection limit: {relative}")
                try:
                    text = path.read_text("utf-8-sig")
                except UnicodeError as exc:
                    raise CaptureError(f"Cannot inspect non-UTF-8 source {relative}.") from exc
                if _RENAMED in text:
                    raise CaptureError("The workspace already defines an EERSim logger symbol.")
                for begin, end in _callback_offsets(text):
                    definitions.append((relative, text, begin, end))
    if len(definitions) != 1:
        raise CaptureError("Expected one ordinary global void initialize() definition in src/. Macro-generated, namespaced or conditional alternatives need a standard callback.")
    _check_cancel(cancel)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.mkdir()  # Exclusive creation, including races with another prepare job.
    ownership = dest.stat()
    copied = {}
    try:
        for relative, expected_size in files:
            _check_cancel(cancel)
            path = source / relative
            _regular(path, source)
            target_path = dest / relative
            target_path.parent.mkdir(parents=True, exist_ok=True)
            digest = hashlib.sha256()
            count = 0
            with path.open("rb") as incoming, target_path.open("xb") as outgoing:
                while chunk := incoming.read(1024 * 1024):
                    _check_cancel(cancel)
                    count += len(chunk)
                    if count > expected_size:
                        raise CaptureError(f"Source changed while copying {relative}.")
                    outgoing.write(chunk)
                    digest.update(chunk)
            if count != expected_size:
                raise CaptureError(f"Source changed while copying {relative}.")
            shutil.copymode(path, target_path)
            copied[relative.as_posix()] = digest.hexdigest()
        relative, source_text, begin, end = definitions[0]
        copied_text = (dest / relative).read_text("utf-8-sig")
        if copied_text != source_text:
            raise CaptureError("initialize source changed while preparing the logger.")
        newline = "\r\n" if b"\r\n" in (dest / relative).read_bytes() else "\n"
        declaration = ("#ifdef __cplusplus\nextern \"C\" {\n#endif\n"
                       "void eersim_tuning_user_initialize(void);\n"
                       "#ifdef __cplusplus\n}\n#endif\n")
        instrumented = declaration + source_text[:begin] + _RENAMED + source_text[end:]
        with (dest / relative).open("w", encoding="utf-8", newline=newline) as output:
            output.write(instrumented)
        (dest / "src" / _LOGGER).write_text(_logger_source(selected, sample_ms), encoding="utf-8", newline="\n")
        metadata = {"schema": 1, "protocol": "EERSIM_TUNE,1", "passive": True,
            "source_workspace": str(source), "ports": list(selected), "sample_ms": sample_ms,
            "initialize_source": relative.as_posix(), "source_sha256": copied,
            "modified_files": [relative.as_posix(), "src/" + _LOGGER],
            "notes": ["Original callbacks run normally; logging starts after initialize returns.",
                "Selected positive physical ports are read without changing units or tare.",
                "Readings are sequential within each scan; logging adds CPU/serial overhead.",
                "Upload opens the program screen; the operator starts the program.",
                "Stopping capture does not stop the robot program."]}
        metadata_path = dest / _METADATA
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", "utf-8")
        _check_cancel(cancel)
    except BaseException:
        # Delete only the exclusive directory created by this invocation.
        if (dest.exists() and not _is_link(dest) and dest.resolve() == dest and
                dest.stat().st_ino == ownership.st_ino and not dest.is_relative_to(source)):
            shutil.rmtree(dest)
        raise
    if log:
        log(f"Prepared passive logger copy for ports {', '.join(map(str, selected))} at {sample_ms} ms: {dest}")
    return PreparedCapture(dest, source, selected, sample_ms, metadata_path)


def discover_pros_cli(explicit: str | Path | None = None) -> Path | None:
    """Find an executable without launching it; never invoke batch/shell files."""
    supplied = explicit or os.environ.get("PROS_CLI")
    if supplied:
        candidate = Path(supplied).expanduser()
        if not candidate.is_file():
            found = shutil.which(str(supplied))
            candidate = Path(found) if found else candidate
        return _executable(candidate)
    found = shutil.which("pros")
    if found and (candidate := _executable(Path(found))):
        return candidate
    home = Path.home()
    roots = [home / ".local" / "bin", home / ".pros", Path(sys.prefix) / "Scripts"]
    for variable in ("LOCALAPPDATA", "ProgramFiles", "ProgramFiles(x86)"):
        if os.environ.get(variable):
            roots.append(Path(os.environ[variable]) / "PROS")
    appdata = Path(os.environ.get("APPDATA", home / "AppData" / "Roaming"))
    config = Path(os.environ.get("XDG_CONFIG_HOME", home / ".config"))
    for base in (appdata, config, home / "Library" / "Application Support"):
        for editor in ("Code", "Code - Insiders", "VSCodium"):
            install = base / editor / "User" / "globalStorage" / "sigbots.pros" / "install"
            if install.is_dir():
                roots.extend(sorted(install.glob("pros-cli-*"), reverse=True))
    for root in roots:
        for relative in ("pros.exe", "pros", "bin/pros.exe", "bin/pros", "pros/pros.exe", "pros/pros"):
            if candidate := _executable(root / relative):
                return candidate
    return None


def _executable(path: Path) -> Path | None:
    if path.suffix.lower() in {".bat", ".cmd", ".ps1", ".py"}:
        return None
    if path.is_file() and (os.name == "nt" and path.suffix.lower() == ".exe" or
                           os.name != "nt" and os.access(path, os.X_OK)):
        return path.resolve()
    return None


def parse_telemetry_line(line: str) -> HardwareSample | None:
    """Parse one complete v1 CSV row, ignoring ordinary PROS terminal chatter."""
    if len(line) > _MAX_LINE:
        return None
    clean = _ANSI.sub("", line).strip()
    offset = clean.find("EERSIM_TUNE,")
    if offset < 0:
        return None
    fields = clean[offset:].split(",")
    if len(fields) != 9 or fields[:2] != ["EERSIM_TUNE", "1"]:
        return None
    try:
        timestamp, port = int(fields[2]), int(fields[3])
        if not 0 <= timestamp <= 0xFFFFFFFF or not 1 <= port <= 21:
            return None
        values = []
        for field in fields[4:]:
            if field.strip() == "null":
                values.append(None)
            else:
                value = float(field)
                if not math.isfinite(value):
                    return None
                values.append(value)
        return HardwareSample(timestamp, port, *values)
    except (ValueError, OverflowError):
        return None


class ScanAssembler:
    """Emit only complete, valid scans; retain no unbounded capture history."""
    def __init__(self, ports: Iterable[int], on_sample: Callable[[dict], None] | None,
                 log: Callable[[str], None] | None = None, capture_id: str | None = None):
        self.ports = _ports(ports)
        self.on_sample, self.log = on_sample, log
        self.capture_id = capture_id or uuid.uuid4().hex
        self.timestamp: int | None = None
        self.previous_emitted: int | None = None
        self.elapsed_ms = 0
        self.pending: dict[int, HardwareSample] = {}
        self.emitted = 0
        self.dropped = 0
        self._done = False

    def _drop(self, reason: str) -> None:
        self.dropped += 1
        if self.log and (self.dropped == 1 or self.dropped % 50 == 0):
            self.log(f"Telemetry scan omitted ({reason}); omitted scans: {self.dropped}.")

    def feed(self, sample: HardwareSample) -> None:
        if sample.port not in self.ports:
            return
        if self.timestamp != sample.timestamp_ms:
            if self.pending and not self._done:
                self._drop("missing ports")
            self.timestamp = sample.timestamp_ms
            self.pending = {}
            self._done = False
        if self._done:
            return
        if sample.port in self.pending:
            self._drop("duplicate port")
            self._done = True
            return
        self.pending[sample.port] = sample
        if len(self.pending) != len(self.ports):
            return
        self._done = True
        if any(row.position_deg is None or row.rpm is None for row in self.pending.values()):
            self._drop("invalid motor position or velocity")
            return
        if self.previous_emitted is not None:
            delta = (sample.timestamp_ms - self.previous_emitted) & 0xFFFFFFFF
            if delta > 0x7FFFFFFF or delta == 0:
                raise CaptureError("Robot timestamp restarted or arrived out of order; start a new capture.")
            self.elapsed_ms += delta
        self.previous_emitted = sample.timestamp_ms
        frame = {"time": self.elapsed_ms / 1000.0, "source": "hardware",
            "capture_id": self.capture_id, "timestamp_ms": sample.timestamp_ms,
            "motors": {str(port): {"position_deg": self.pending[port].position_deg,
                "velocity_rpm": self.pending[port].rpm, "voltage_mv": self.pending[port].voltage_mv,
                "current_ma": self.pending[port].current_ma, "battery_mv": self.pending[port].battery_mv}
                for port in self.ports}}
        self.emitted += 1
        if self.on_sample:
            self.on_sample(frame)

    def finish(self) -> None:
        # Complete scans emit immediately, including the final row before cancel.
        if self.pending and not self._done:
            self._drop("capture ended before all ports arrived")
            self._done = True


def _workspace(prepared: PreparedCapture | str | Path) -> tuple[Path, dict]:
    path = prepared.workspace if isinstance(prepared, PreparedCapture) else Path(prepared)
    path = path.expanduser().resolve(strict=True)
    try:
        metadata = json.loads((path / _METADATA).read_text("utf-8"))
        if metadata["schema"] != 1 or metadata["passive"] is not True:
            raise ValueError("unsupported metadata")
        _ports(metadata["ports"])
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise CaptureError("Prepare a passive logger workspace before building, uploading or capturing.") from exc
    return path, metadata


def _cli(explicit: str | Path | None) -> Path:
    path = discover_pros_cli(explicit)
    if path is None:
        raise CaptureError("PROS CLI was not found. Install the PROS CLI/toolchain or select its pros executable.")
    return path


def _command_environment(cli: Path) -> dict[str, str]:
    env = os.environ.copy()
    additions = [str(cli.parent)]
    # VS Code installs its ARM toolchain beside the CLI instead of on PATH.
    for parent in (cli.parent, cli.parent.parent, cli.parent.parent.parent):
        if parent.name == "install":
            for toolchain in sorted(parent.glob("pros-toolchain-*")):
                for candidate in (toolchain / "bin", toolchain / "usr" / "bin"):
                    if candidate.is_dir():
                        additions.append(str(candidate))
    env["PATH"] = os.pathsep.join(additions + [env.get("PATH", "")])
    env["PYTHONUNBUFFERED"] = "1"
    env["NO_COLOR"] = "1"
    return env


def run_build(prepared: PreparedCapture | str | Path, *, cli: str | Path | None = None,
              log: Callable[[str], None] | None = None, cancel: threading.Event | None = None,
              timeout_s: float = 300) -> CommandResult:
    path, _ = _workspace(prepared)
    executable = _cli(cli)
    return run_command([str(executable), "make"], cwd=path, log=log, cancel=cancel,
                       timeout_s=timeout_s, env=_command_environment(executable))


def run_upload(prepared: PreparedCapture | str | Path, *, slot: int = 1,
               port: str | None = None, cli: str | Path | None = None,
               log: Callable[[str], None] | None = None, cancel: threading.Event | None = None,
               timeout_s: float = 120) -> CommandResult:
    if type(slot) is not int or not 1 <= slot <= 8:
        raise CaptureError("The V5 program slot must be between 1 and 8.")
    path, _ = _workspace(prepared)
    executable = _cli(cli)
    args = [str(executable), "upload", "--slot", str(slot), "--after", "screen", str(path)]
    if port:
        args.append(_serial_port(port))
    return run_command(args, cwd=path, log=log, cancel=cancel,
                       timeout_s=timeout_s, env=_command_environment(executable))


def _serial_port(port: str) -> str:
    if not isinstance(port, str) or not port.strip() or port.startswith("-") or any(ord(c) < 32 for c in port):
        raise CaptureError("Enter a serial port such as COM3 or /dev/ttyACM1.")
    return port


def run_terminal(prepared: PreparedCapture | str | Path, *, port: str | None = None,
                 cli: str | Path | None = None, on_sample: Callable[[dict], None] | None = None,
                 log: Callable[[str], None] | None = None, cancel: threading.Event | None = None,
                 timeout_s: float = 3600) -> CommandResult:
    path, metadata = _workspace(prepared)
    executable = _cli(cli)
    assembler = ScanAssembler(metadata["ports"], on_sample, log)
    def line_received(line: str) -> None:
        sample = parse_telemetry_line(line)
        if sample is not None:
            assembler.feed(sample)
        elif "EERSIM_TUNE," in line and log:
            log("Ignored a malformed or unsupported telemetry row.")
        elif log:
            log(line)
    args = [str(executable), "terminal", "--backend", "solo", "--no-banner"]
    if port:
        args.append(_serial_port(port))
    try:
        result = run_command(args, cwd=path, log=line_received, cancel=cancel,
                             timeout_s=timeout_s, env=_command_environment(executable), terminal=True)
    finally:
        assembler.finish()
    if log:
        log(f"Capture finished: {assembler.emitted} complete scans, {assembler.dropped} omitted scans. The robot program was not stopped.")
    return result


def _resume_windows(process: subprocess.Popen) -> None:
    import ctypes
    from ctypes import wintypes
    api = ctypes.WinDLL("ntdll")
    api.NtResumeProcess.argtypes = [wintypes.HANDLE]
    api.NtResumeProcess.restype = ctypes.c_long
    status = api.NtResumeProcess(int(process._handle))
    if status != 0:
        raise CaptureError(f"Cannot resume the PROS helper (Windows status {status}).")


def run_command(command: Iterable[str], *, cwd: str | Path, timeout_s: float,
                log: Callable[[str], None] | None = None,
                cancel: threading.Event | None = None, env: dict[str, str] | None = None,
                terminal: bool = False) -> CommandResult:
    """Run an argv vector with bounded output, deadline and owned child cleanup.

    Windows helpers start suspended and join a kill-on-close job before running.
    POSIX helpers use a private process group. Terminal stdin is a private PTY
    on POSIX, or a hidden console on Windows; no keystrokes are transmitted.
    """
    args = tuple(map(str, command))
    if not args or any("\0" in arg for arg in args) or Path(args[0]).suffix.lower() in {".cmd", ".bat", ".ps1"}:
        raise CaptureError("A direct executable argument list is required.")
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 86400:
        raise CaptureError("Command timeout must be greater than zero and at most 24 hours.")
    if cancel is not None and cancel.is_set():
        return CommandResult(-1, True, False, "", args)
    master = slave = None
    job = process = reader = None
    events: queue.Queue[bytes | BaseException | None] = queue.Queue(maxsize=64)
    stop_reader = threading.Event()
    tail = ""
    pending = ""
    discarding_line = False
    decoder = codecs.getincrementaldecoder("utf-8")("replace")
    cancelled = timed_out = False
    tree_closed = False
    options: dict = {"stdin": subprocess.DEVNULL, "stdout": subprocess.PIPE,
                     "stderr": subprocess.STDOUT, "bufsize": 0, "shell": False,
                     "cwd": str(cwd), "env": env}
    if os.name == "nt":
        from .diagnostic_process import WindowsJob
        job = WindowsJob()
        startup = subprocess.STARTUPINFO()
        startup.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        startup.wShowWindow = 0
        options["startupinfo"] = startup
        options["creationflags"] = 0x00000004 | (subprocess.CREATE_NEW_CONSOLE if terminal else subprocess.CREATE_NO_WINDOW)
    else:
        options["start_new_session"] = True
        if terminal:
            import pty
            master, slave = pty.openpty()
            options["stdin"] = slave
    def publish(value: bytes | BaseException | None) -> None:
        while not stop_reader.is_set():
            try:
                events.put(value, timeout=.1)
                return
            except queue.Full:
                pass
    def read_output() -> None:
        try:
            while chunk := process.stdout.read(4096):
                publish(chunk)
        except BaseException as exc:
            if not stop_reader.is_set():
                publish(exc)
        finally:
            publish(None)
    def consume(chunk: bytes, *, final: bool = False) -> None:
        nonlocal tail, pending, discarding_line
        text = decoder.decode(chunk, final=final)
        tail = (tail + text)[-32768:]
        pending += text
        while "\n" in pending:
            line, pending = pending.split("\n", 1)
            if log and not discarding_line and (not terminal or len(line) <= _MAX_LINE):
                log(line[-_MAX_LINE:].rstrip("\r"))
            discarding_line = False
        if len(pending) > _MAX_LINE:
            # Bound arbitrary program output without buffering megabytes of it.
            if log and not terminal:
                log(pending[:_MAX_LINE])
            pending = ""
            discarding_line = True
        if final and pending:
            # A partial CSV row can look syntactically valid even when its final
            # number was cut off. Only newline-terminated terminal rows count.
            if log and not terminal and not discarding_line:
                log(pending.rstrip("\r"))
            pending = ""
    def terminate_tree() -> None:
        nonlocal tree_closed
        if tree_closed:
            return
        tree_closed = True
        if job is not None:
            job.close()
        elif process is not None and os.name != "nt":
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        if process is not None and process.poll() is None:
            process.kill()
        if process is not None:
            process.wait(timeout=3)
    try:
        process = subprocess.Popen(args, **options)
        if slave is not None:
            os.close(slave)
            slave = None
        if job is not None:
            job.attach(process)
            _resume_windows(process)
        reader = threading.Thread(target=read_output, name="eersim-pros-output", daemon=True)
        reader.start()
        started = time.monotonic()
        ended = False
        while not ended:
            if cancel is not None and cancel.is_set():
                cancelled = True
                break
            if time.monotonic() - started >= timeout_s:
                timed_out = True
                break
            try:
                event = events.get(timeout=.05)
            except queue.Empty:
                if process.poll() is not None:
                    # A surviving child can hold the pipe open: close the group.
                    terminate_tree()
                continue
            if event is None:
                ended = True
            elif isinstance(event, BaseException):
                raise CaptureError(f"Reading PROS output failed: {event}") from event
            else:
                consume(event)
        # EOF need not mean process exit. The same deadline/cancellation applies.
        while not cancelled and not timed_out and process.poll() is None:
            cancelled = cancel is not None and cancel.is_set()
            timed_out = time.monotonic() - started >= timeout_s
            if not cancelled and not timed_out:
                time.sleep(.02)
        terminate_tree()
        # Drain already-read bytes after cancellation so the final complete scan
        # is kept. Bound draining even when an inherited pipe behaves badly.
        deadline = time.monotonic() + 1.0
        while reader.is_alive() or not events.empty():
            if time.monotonic() >= deadline:
                break
            try:
                event = events.get(timeout=.02)
            except queue.Empty:
                continue
            if isinstance(event, bytes):
                consume(event)
        consume(b"", final=True)
        return CommandResult(process.returncode, cancelled, timed_out, tail, args)
    except OSError as exc:
        raise CaptureError(f"Could not run PROS: {exc}") from exc
    finally:
        try:
            terminate_tree()
        finally:
            stop_reader.set()
            for descriptor in (master, slave):
                if descriptor is not None:
                    os.close(descriptor)
            if process is not None and process.stdout is not None:
                process.stdout.close()
            if reader is not None:
                reader.join(timeout=1)
