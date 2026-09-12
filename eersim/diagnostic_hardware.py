"""Bounded, read-only hardware diagnostics; no CUDA context or GUI is created.

NVIDIA queries use the documented nvidia-smi CSV interface. GPU index order is
not assumed to equal CUDA order: UUID masks and explicit PCI_BUS_ID ordering
can be resolved without initializing a driver context.
"""
from __future__ import annotations

import atexit
import csv
import ctypes
from datetime import datetime, timezone
import importlib.metadata
import io
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import threading
import time


QUERY_TIMEOUT_SECONDS = 3.0
MAX_SAMPLES = 1200
_IS_WINDOWS = sys.platform == "win32"
_BASE_FIELDS = ("index", "uuid", "name", "pci.bus_id", "pstate",
                "utilization.gpu", "utilization.memory", "memory.used", "memory.total")
_OPTIONAL_FIELDS = ("power.draw", "power.limit", "temperature.gpu",
                    "clocks.current.graphics", "clocks.current.memory")
_INVENTORY_FIELDS = ("index", "uuid", "name", "pci.bus_id", "driver_version", "memory.total")
_LABELS = {
    "index": "gpu_index", "uuid": "gpu_uuid", "name": "gpu_name",
    "pci.bus_id": "pci_bus_id", "pstate": "pstate", "driver_version": "driver_version",
    "utilization.gpu": "gpu_utilization_percent", "utilization.memory": "memory_utilization_percent",
    "memory.used": "memory_used_mib", "memory.total": "memory_total_mib",
    "power.draw": "power_draw_w", "power.limit": "power_limit_w",
    "temperature.gpu": "temperature_c", "clocks.current.graphics": "graphics_clock_mhz",
    "clocks.current.memory": "memory_clock_mhz",
}
_TEXT_FIELDS = {"uuid", "name", "pci.bus_id", "pstate", "driver_version"}
_CSV_FIELDS = ("timestamp_utc", *(_LABELS[f] for f in _BASE_FIELDS + _OPTIONAL_FIELDS),
               "cuda_visible", "selected_for_logical_device")


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _process_options() -> dict:
    return {"creationflags": getattr(subprocess, "CREATE_NO_WINDOW", 0x08000000)} if _IS_WINDOWS else {}


def _find_nvidia_smi() -> str | None:
    found = shutil.which("nvidia-smi")
    if found:
        return found
    if _IS_WINDOWS:
        candidates = []
        if os.environ.get("SystemRoot"):
            candidates.append(Path(os.environ["SystemRoot"]) / "System32" / "nvidia-smi.exe")
        for key in ("ProgramFiles", "ProgramW6432"):
            if os.environ.get(key):
                candidates.append(Path(os.environ[key]) / "NVIDIA Corporation" / "NVSMI" / "nvidia-smi.exe")
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
    return None


def _command(executable: str, fields: tuple[str, ...]) -> list[str]:
    return [executable, "--query-gpu=" + ",".join(fields), "--format=csv,noheader,nounits"]


def _query_once(executable: str, fields: tuple[str, ...]) -> tuple[str, str]:
    """Static inventory query. run() kills and reaps its child on timeout."""
    try:
        result = subprocess.run(_command(executable, fields), capture_output=True, text=True,
                                encoding="utf-8", errors="replace", timeout=QUERY_TIMEOUT_SECONDS,
                                check=False, **_process_options())
    except subprocess.TimeoutExpired:
        return "timeout", ""
    except OSError:
        return "command_unavailable", ""
    return ("ok", result.stdout) if result.returncode == 0 else ("query_failed", "")


def _parse_rows(raw: str, fields: tuple[str, ...]) -> list[dict]:
    result = []
    for values in csv.reader(io.StringIO(raw)):
        if not values or not any(value.strip() for value in values):
            continue
        if len(values) != len(fields):
            raise ValueError("NVIDIA CSV column count differs from its query")
        row = {}
        for field, raw_value in zip(fields, values):
            value = raw_value.strip()
            if not value or value.upper() in {"N/A", "[N/A]", "NOT SUPPORTED", "[NOT SUPPORTED]"}:
                parsed = None
            elif field in _TEXT_FIELDS:
                parsed = value
            else:
                parsed = float(value)
                if not math.isfinite(parsed) or parsed < 0:
                    raise ValueError("NVIDIA CSV contains invalid numeric data")
                if field == "index":
                    if not parsed.is_integer():
                        raise ValueError("NVIDIA GPU index must be an integer")
                    parsed = int(parsed)
            row[_LABELS[field]] = parsed
        if row.get("gpu_index") is None or not row.get("gpu_uuid"):
            raise ValueError("NVIDIA CSV is missing GPU identity")
        result.append(row)
    identities = [row["gpu_uuid"] for row in result]
    if len(identities) != len(set(identities)):
        raise ValueError("NVIDIA CSV repeats GPU identity")
    return result


def _cuda_environment() -> dict:
    return {"CUDA_VISIBLE_DEVICES": os.environ.get("CUDA_VISIBLE_DEVICES"),
            "CUDA_DEVICE_ORDER": os.environ.get("CUDA_DEVICE_ORDER", "FASTEST_FIRST")}


def _visibility(rows: list[dict], environment: dict, logical_device: int | None) -> dict:
    """Resolve only identities justified by CUDA's documented enumeration rules."""
    mask = environment["CUDA_VISIBLE_DEVICES"]
    order = environment["CUDA_DEVICE_ORDER"]
    uuids = [row["gpu_uuid"] for row in rows]
    visible: list[str] | None = None
    reason = "CUDA default ordering is not inferred from nvidia-smi indices"
    if mask is not None and mask.strip() in {"", "-1"}:
        visible, reason = [], "CUDA visibility is disabled by the mask"
    elif mask is not None and all(token.strip().startswith("GPU-") for token in mask.split(",")):
        reason = "CUDA UUID mask cannot be resolved uniquely from physical GPU inventory"
        resolved = []
        for token in mask.split(","):
            matches = [uuid for uuid in uuids if uuid.startswith(token.strip())]
            if len(matches) != 1 or matches[0] in resolved:
                break
            resolved.append(matches[0])
        else:
            visible, reason = resolved, "Unique CUDA UUID mask"
    elif order == "PCI_BUS_ID":
        try:
            ordered = sorted(rows, key=lambda row: tuple(int(part, 16) for part in
                             row["pci_bus_id"].replace(".", ":").split(":")))
            if any(not row.get("pci_bus_id") for row in rows):
                raise ValueError("Missing PCI identity")
            indices = list(range(len(ordered))) if mask is None else [int(token.strip()) for token in mask.split(",")]
            if len(indices) != len(set(indices)) or any(index < 0 or index >= len(ordered) for index in indices):
                raise ValueError("Unresolvable CUDA index mask")
            visible = [ordered[index]["gpu_uuid"] for index in indices]
            reason = "Explicit CUDA PCI_BUS_ID ordering"
        except (AttributeError, KeyError, TypeError, ValueError):
            reason = "CUDA mask or PCI identity cannot be resolved safely"
    elif mask is None and len(rows) <= 1:
        visible, reason = uuids, "At most one physical GPU is present"
    selected = visible[logical_device] if visible is not None and logical_device is not None and logical_device < len(visible) else None
    for row in rows:
        row["cuda_visible"] = (row["gpu_uuid"] in visible) if visible is not None else (True if mask is None else None)
        row["selected_for_logical_device"] = (row["gpu_uuid"] == selected) if selected is not None else None
    return {"environment": environment, "logical_device": logical_device,
            "resolved_visible_gpu_uuids": visible, "selected_gpu_uuid": selected, "explanation": reason}


def _windows_resources() -> dict:
    result = {"cpu_model": None, "physical_cores": None,
              "memory_total_bytes": None, "memory_available_bytes": None}
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DESCRIPTION\System\CentralProcessor\0") as key:
            result["cpu_model"] = winreg.QueryValueEx(key, "ProcessorNameString")[0].strip()
    except (OSError, ImportError):
        pass
    try:
        from ctypes import wintypes

        class MemoryStatus(ctypes.Structure):
            _fields_ = [("length", wintypes.DWORD), ("load", wintypes.DWORD)] + [
                (name, ctypes.c_ulonglong) for name in
                ("total", "available", "total_page", "available_page", "total_virtual", "available_virtual", "extended")]

        kernel = ctypes.WinDLL("kernel32", use_last_error=True)
        status = MemoryStatus()
        status.length = ctypes.sizeof(status)
        memory_status = kernel.GlobalMemoryStatusEx
        memory_status.argtypes = [ctypes.POINTER(MemoryStatus)]
        memory_status.restype = wintypes.BOOL
        if memory_status(ctypes.byref(status)):
            result.update(memory_total_bytes=status.total, memory_available_bytes=status.available)
        information = kernel.GetLogicalProcessorInformationEx
        information.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.POINTER(wintypes.DWORD)]
        information.restype = wintypes.BOOL
        size = wintypes.DWORD(0)
        information(0, None, ctypes.byref(size))  # RelationProcessorCore
        if 0 < size.value <= 16 * 1024 * 1024:
            buffer = ctypes.create_string_buffer(size.value)
            if information(0, buffer, ctypes.byref(size)):
                offset, count = 0, 0
                while offset + 8 <= size.value:
                    relationship = wintypes.DWORD.from_buffer(buffer, offset).value
                    record_size = wintypes.DWORD.from_buffer(buffer, offset + 4).value
                    if record_size < 8 or offset + record_size > size.value:
                        break
                    count += int(relationship == 0)
                    offset += record_size
                if offset == size.value and count:
                    result["physical_cores"] = count
    except (OSError, AttributeError, ValueError):
        pass
    return result


def _linux_resources() -> dict:
    result = {"cpu_model": None, "physical_cores": None,
              "memory_total_bytes": None, "memory_available_bytes": None}
    try:
        blocks = Path("/proc/cpuinfo").read_text(encoding="utf-8", errors="replace").split("\n\n")
        cores = set()
        for block in blocks:
            values = dict(line.split(":", 1) for line in block.splitlines() if ":" in line)
            values = {key.strip(): value.strip() for key, value in values.items()}
            if values.get("model name"):
                result["cpu_model"] = values["model name"]
            if "physical id" in values and "core id" in values:
                cores.add((values["physical id"], values["core id"]))
        result["physical_cores"] = len(cores) or None
    except (OSError, ValueError):
        pass
    try:
        memory = {}
        for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
            key, value = line.split(":", 1)
            if key in {"MemTotal", "MemAvailable"}:
                memory[key] = int(value.strip().split()[0]) * 1024
        result.update(memory_total_bytes=memory.get("MemTotal"), memory_available_bytes=memory.get("MemAvailable"))
    except (OSError, ValueError):
        pass
    return result


def system_info(*, logical_device: int | None = 0) -> dict:
    """Collect once per diagnostic run; missing optional hardware is not fatal."""
    if logical_device is not None and (isinstance(logical_device, bool) or not isinstance(logical_device, int) or logical_device < 0):
        raise ValueError("logical_device must be a nonnegative integer or None")
    from . import __version__
    resources = _windows_resources() if _IS_WINDOWS else _linux_resources()
    versions = {}
    for name in ("eerkit-simulator", "numpy", "pybullet", "PySide6", "vtk", "ovphysx", "ovstage", "scipy"):
        try:
            versions[name] = importlib.metadata.version(name)
        except (importlib.metadata.PackageNotFoundError, OSError, ValueError):
            versions[name] = None
    nvidia = {"status": "command_unavailable", "gpus": []}
    executable = _find_nvidia_smi()
    if executable:
        status, raw = _query_once(executable, _INVENTORY_FIELDS)
        nvidia["status"] = status
        if status == "ok":
            try:
                nvidia["gpus"] = _parse_rows(raw, _INVENTORY_FIELDS)
            except ValueError:
                nvidia["status"] = "invalid_csv"
    nvidia["cuda_selection"] = _visibility(nvidia["gpus"], _cuda_environment(), logical_device)
    return {"schema_version": 1, "collected_at_utc": _utc_now(),
            "os": {"system": platform.system(), "release": platform.release(),
                   "version": platform.version(), "architecture": platform.machine()},
            "cpu": {"model": resources["cpu_model"], "logical_cores": os.cpu_count(),
                    "physical_cores": resources["physical_cores"]},
            "memory": {"total_bytes": resources["memory_total_bytes"],
                       "available_bytes": resources["memory_available_bytes"]},
            "python": {"version": platform.python_version(), "implementation": platform.python_implementation(),
                       "bits": ctypes.sizeof(ctypes.c_void_p) * 8},
            "eersim_version": __version__, "runtime_versions": versions, "nvidia": nvidia}


class NvidiaSmiSampler:
    """One owned, hidden query at a time; stop kills and reaps an active query.

    ``destination`` is a new case directory (it may already exist). Existing
    output files are never overwritten. Sampling collects no process names.
    """

    def __init__(self, destination: str | Path, *, interval_seconds: float = 1.0,
                 max_samples: int = MAX_SAMPLES, logical_device: int | None = 0):
        if not math.isfinite(interval_seconds) or not 0.01 <= interval_seconds <= 60:
            raise ValueError("interval_seconds must be between 0.01 and 60")
        if isinstance(max_samples, bool) or not isinstance(max_samples, int) or not 1 <= max_samples <= MAX_SAMPLES:
            raise ValueError(f"max_samples must be between 1 and {MAX_SAMPLES}")
        if logical_device is not None and (isinstance(logical_device, bool) or not isinstance(logical_device, int) or logical_device < 0):
            raise ValueError("logical_device must be a nonnegative integer or None")
        self.destination = Path(destination)
        self.interval_seconds = float(interval_seconds)
        self.max_samples = max_samples
        self.logical_device = logical_device
        self._event = threading.Event()
        self._process_lock = threading.Lock()
        self._stop_lock = threading.Lock()
        self._process = None
        self._thread = None
        self._file = None
        self._summary_file = None
        self._stopped = False
        self._started_at = None
        self._environment = _cuda_environment()
        self._summary = {"schema_version": 1, "status": "not_started", "sample_queries": 0,
                         "rows_written": 0, "interval_seconds": self.interval_seconds,
                         "max_samples": self.max_samples, "errors": [], "gpus": {},
                         "measurement_scope": "All physical GPUs; utilization can include unrelated applications",
                         "csv_file": "gpu-samples.csv", "summary_file": "gpu-summary.json"}

    def start(self) -> "NvidiaSmiSampler":
        if self._stopped:
            raise RuntimeError("A stopped sampler cannot be restarted")
        if self._started_at is not None:
            return self
        self.destination.mkdir(parents=True, exist_ok=True)
        if any((self.destination / filename).exists() for filename in ("gpu-summary.json", "gpu-samples.csv")):
            raise FileExistsError("GPU diagnostic output already exists in this case directory")
        self._summary_file = (self.destination / "gpu-summary.json").open("x", encoding="utf-8")
        try:
            self._file = (self.destination / "gpu-samples.csv").open("x", encoding="utf-8", newline="")
        except Exception:
            self._summary_file.close()
            self._summary_file = None
            raise
        self._writer = csv.DictWriter(self._file, fieldnames=_CSV_FIELDS)
        self._writer.writeheader()
        self._file.flush()
        self._started_at = time.monotonic()
        self._summary["started_at_utc"] = _utc_now()
        self._executable = _find_nvidia_smi()
        if self._executable is None:
            self._summary["status"] = "command_unavailable"
            self._file.close()
        else:
            self._summary["status"] = "sampling"
            self._thread = threading.Thread(target=self._sample, name="eersim-nvidia-diagnostics", daemon=True)
            self._thread.start()
        atexit.register(self.stop)
        return self

    def _query(self, fields: tuple[str, ...]) -> tuple[str, str]:
        process = None
        try:
            with self._process_lock:
                if self._event.is_set():
                    return "stopped", ""
                process = subprocess.Popen(_command(self._executable, fields), stdout=subprocess.PIPE,
                                           stderr=subprocess.PIPE, text=True, encoding="utf-8",
                                           errors="replace", **_process_options())
                self._process = process
            try:
                stdout, _ = process.communicate(timeout=QUERY_TIMEOUT_SECONDS)
            except subprocess.TimeoutExpired:
                process.kill()
                process.communicate(timeout=1.0)
                return "timeout", ""
            if self._event.is_set():
                return "stopped", ""
            return ("ok", stdout) if process.returncode == 0 else ("query_failed", "")
        except (OSError, subprocess.SubprocessError):
            return "command_unavailable", ""
        finally:
            if process is not None and process.poll() is None:
                try:
                    process.kill()
                    process.wait(timeout=1.0)
                except (OSError, subprocess.SubprocessError):
                    pass
            with self._process_lock:
                self._process = None

    def _sample(self) -> None:
        fields = _BASE_FIELDS + _OPTIONAL_FIELDS
        try:
            while not self._event.is_set() and self._summary["sample_queries"] < self.max_samples:
                started = time.monotonic()
                status, raw = self._query(fields)
                if status == "query_failed" and fields != _BASE_FIELDS:
                    fields = _BASE_FIELDS
                    self._summary["errors"].append("Optional NVIDIA fields were rejected; collecting basic metrics")
                    status, raw = self._query(fields)
                if status == "stopped":
                    break
                self._summary["sample_queries"] += 1
                if status != "ok":
                    self._summary["status"] = status
                    self._summary["errors"].append("NVIDIA sampling ended: " + status)
                    break
                rows = _parse_rows(raw, fields)
                self._summary["available_fields"] = [_LABELS[field] for field in fields]
                self._summary["cuda_selection"] = _visibility(rows, self._environment, self.logical_device)
                timestamp = _utc_now()
                for row in rows:
                    self._writer.writerow({"timestamp_utc": timestamp, **row})
                    self._update_metrics(row)
                self._summary["rows_written"] += len(rows)
                self._file.flush()
                if not rows:
                    self._summary["status"] = "no_gpus"
                    break
                if self._summary["sample_queries"] < self.max_samples:
                    self._event.wait(max(0.0, self.interval_seconds - (time.monotonic() - started)))
            if self._summary["status"] == "sampling":
                self._summary["status"] = "sample_limit" if self._summary["sample_queries"] >= self.max_samples else "stopped"
        except (ValueError, OSError) as error:
            self._summary["status"] = "invalid_csv" if isinstance(error, ValueError) else "write_error"
            self._summary["errors"].append("NVIDIA sampling ended: " + self._summary["status"])
        finally:
            self._file.close()

    def _update_metrics(self, row: dict) -> None:
        entry = self._summary["gpus"].setdefault(row["gpu_uuid"], {
            "gpu_index": row["gpu_index"], "gpu_name": row["gpu_name"], "samples": 0, "metrics": {}})
        entry["samples"] += 1
        for name in ("gpu_utilization_percent", "memory_utilization_percent", "memory_used_mib", "power_draw_w", "temperature_c"):
            value = row.get(name)
            if value is None:
                continue
            metric = entry["metrics"].setdefault(name, {"count": 0, "min": value, "max": value, "mean": 0.0})
            metric["count"] += 1
            metric["min"], metric["max"] = min(metric["min"], value), max(metric["max"], value)
            metric["mean"] += (value - metric["mean"]) / metric["count"]

    def stop(self) -> dict:
        with self._stop_lock:
            if self._stopped:
                return self._summary
            self._event.set()
            with self._process_lock:
                if self._process is not None and self._process.poll() is None:
                    try:
                        self._process.kill()
                    except OSError:
                        pass
            if self._thread is not None:
                self._thread.join(timeout=5.0)
                if self._thread.is_alive():
                    self._summary["status"] = "stop_timeout"
                    self._summary["errors"].append("Sampling thread did not stop within five seconds")
            self._summary["duration_seconds"] = time.monotonic() - self._started_at if self._started_at is not None else 0.0
            self._summary["stopped_at_utc"] = _utc_now()
            if self._summary_file is not None:
                json.dump(self._summary, self._summary_file, indent=2, allow_nan=False)
                self._summary_file.write("\n")
                self._summary_file.close()
            self._stopped = True
            atexit.unregister(self.stop)
            return self._summary

    def __enter__(self) -> "NvidiaSmiSampler":
        return self.start()

    def __exit__(self, *_args) -> None:
        self.stop()
