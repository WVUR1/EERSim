"""Hardware diagnostics never require an NVIDIA driver, GUI, or live GPU."""
import csv
import io
import json
from pathlib import Path
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from eersim import diagnostic_hardware as hardware


def csv_response(fields, *, utilization=25, optional_na=False):
    output = io.StringIO()
    writer = csv.writer(output)
    for index, uuid, bus in ((0, "GPU-aaaa-0000", "00000000:0B:00.0"),
                             (1, "GPU-bbbb-0000", "00000000:03:00.0")):
        row = {"index": str(index), "uuid": uuid, "name": 'Test GPU, "quoted"',
               "pci.bus_id": bus, "pstate": "P2", "driver_version": "595.71.05",
               "utilization.gpu": str(utilization), "utilization.memory": "10",
               "memory.used": "2048", "memory.total": "16384", "power.draw": "100.5",
               "power.limit": "250", "temperature.gpu": "50", "clocks.current.graphics": "1300",
               "clocks.current.memory": "6000"}
        if optional_na:
            row.update({field: "[N/A]" for field in hardware._OPTIONAL_FIELDS})
        writer.writerow([row[field] for field in fields])
    return output.getvalue()


class FakeProcess:
    def __init__(self, response="", *, returncode=0, blocked=False, timeout=False):
        self.response = response
        self.returncode = None
        self.exit_code = returncode
        self.blocked = blocked
        self.timeout = timeout
        self.killed = False
        self.reaped = False
        self.entered = threading.Event()
        self.released = threading.Event()

    def communicate(self, timeout=None):
        self.entered.set()
        if self.timeout and not self.killed:
            raise subprocess.TimeoutExpired("nvidia-smi", timeout)
        if self.blocked and not self.released.wait(timeout):
            raise subprocess.TimeoutExpired("nvidia-smi", timeout)
        self.returncode = -9 if self.killed else self.exit_code
        self.reaped = True
        return self.response, ""

    def poll(self):
        return self.returncode

    def kill(self):
        self.killed = True
        self.released.set()

    def wait(self, timeout=None):
        self.communicate(timeout)
        return self.returncode


def mocked_queries(monkeypatch, factory):
    calls = []
    processes = []

    def popen(command, **options):
        fields = tuple(command[1].split("=", 1)[1].split(","))
        calls.append((command, options, fields))
        process = factory(fields, len(calls))
        processes.append(process)
        return process

    monkeypatch.setattr(hardware, "_find_nvidia_smi", lambda: "fake-nvidia-smi")
    monkeypatch.setattr(hardware.subprocess, "Popen", popen)
    return calls, processes


def finished(sampler):
    sampler._thread.join(timeout=2)
    assert not sampler._thread.is_alive(), "mock sampler failed to complete"
    return sampler.stop()


def test_no_nvidia_command_still_produces_readable_outputs(monkeypatch, tmp_path):
    monkeypatch.setattr(hardware, "_find_nvidia_smi", lambda: None)
    monkeypatch.setattr(hardware.subprocess, "Popen", lambda *args, **kwargs: pytest.fail("No subprocess expected"))
    sampler = hardware.NvidiaSmiSampler(tmp_path).start()
    summary = sampler.stop()
    assert summary["status"] == "command_unavailable"
    assert summary["rows_written"] == 0
    assert json.loads((tmp_path / "gpu-summary.json").read_text()) == summary
    assert list(csv.DictReader((tmp_path / "gpu-samples.csv").open(newline=""))) == []
    assert sampler.stop() is summary


def test_samples_all_gpus_uuid_mapping_csv_and_aggregate(monkeypatch, tmp_path):
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "GPU-bbbb")
    calls, processes = mocked_queries(monkeypatch, lambda fields, count:
        FakeProcess(csv_response(fields, utilization=20 * count, optional_na=True)))
    sampler = hardware.NvidiaSmiSampler(tmp_path, interval_seconds=0.01, max_samples=2).start()
    summary = finished(sampler)
    assert summary["status"] == "sample_limit"
    assert summary["sample_queries"] == 2 and summary["rows_written"] == 4
    assert len(calls) == 2 and all(process.reaped for process in processes)
    rows = list(csv.DictReader((tmp_path / "gpu-samples.csv").open(newline="")))
    assert rows[0]["gpu_name"] == 'Test GPU, "quoted"'
    assert rows[0]["selected_for_logical_device"] == "False"
    assert rows[1]["selected_for_logical_device"] == "True"
    assert rows[0]["power_draw_w"] == ""
    metric = summary["gpus"]["GPU-bbbb-0000"]["metrics"]["gpu_utilization_percent"]
    assert metric == {"count": 2, "min": 20, "max": 40, "mean": 30}
    assert summary["cuda_selection"]["selected_gpu_uuid"] == "GPU-bbbb-0000"


def test_optional_query_rejection_falls_back_once_with_fixed_csv_header(monkeypatch, tmp_path):
    calls, _ = mocked_queries(monkeypatch, lambda fields, count:
        FakeProcess(returncode=2) if count == 1 else FakeProcess(csv_response(fields)))
    summary = finished(hardware.NvidiaSmiSampler(tmp_path, interval_seconds=0.01, max_samples=2).start())
    assert [call[2] for call in calls] == [hardware._BASE_FIELDS + hardware._OPTIONAL_FIELDS,
                                        hardware._BASE_FIELDS, hardware._BASE_FIELDS]
    assert len(summary["errors"]) == 1
    assert summary["rows_written"] == 4
    rows = list(csv.DictReader((tmp_path / "gpu-samples.csv").open(newline="")))
    assert rows[0]["gpu_utilization_percent"] == "25.0" and rows[0]["power_draw_w"] == ""


def test_stop_kills_and_reaps_only_owned_pending_query(monkeypatch, tmp_path):
    process = FakeProcess(blocked=True)
    mocked_queries(monkeypatch, lambda *args: process)
    sampler = hardware.NvidiaSmiSampler(tmp_path).start()
    assert process.entered.wait(timeout=1)
    started = time.monotonic()
    summary = sampler.stop()
    assert time.monotonic() - started < 1.5
    assert process.killed and process.reaped and not sampler._thread.is_alive()
    assert summary["status"] == "stopped" and summary["rows_written"] == 0


def test_query_timeout_is_reaped_and_does_not_retry(monkeypatch, tmp_path):
    calls, processes = mocked_queries(monkeypatch, lambda *args: FakeProcess(timeout=True))
    summary = finished(hardware.NvidiaSmiSampler(tmp_path).start())
    assert summary["status"] == "timeout" and len(calls) == 1
    assert processes[0].killed and processes[0].reaped


def test_windows_subprocesses_are_hidden_and_never_use_shell(monkeypatch, tmp_path):
    monkeypatch.setattr(hardware, "_IS_WINDOWS", True)
    calls, _ = mocked_queries(monkeypatch, lambda fields, _: FakeProcess(csv_response(fields)))
    finished(hardware.NvidiaSmiSampler(tmp_path, max_samples=1).start())
    options = calls[0][1]
    assert options["creationflags"] & 0x08000000
    assert not options.get("shell")
    static_calls = []

    def run(command, **kwargs):
        static_calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout=csv_response(hardware._INVENTORY_FIELDS))

    monkeypatch.setattr(hardware.subprocess, "run", run)
    assert hardware._query_once("fake-nvidia-smi", hardware._INVENTORY_FIELDS)[0] == "ok"
    assert static_calls[0][1]["creationflags"] & 0x08000000
    assert static_calls[0][1]["timeout"] == 3
    assert "--query-compute-apps" not in " ".join(static_calls[0][0])


@pytest.mark.parametrize("raw", ["malformed,csv\n", csv_response(hardware._BASE_FIELDS + hardware._OPTIONAL_FIELDS).replace("25", "nan", 1)])
def test_invalid_gpu_csv_is_reported_without_partial_numeric_data(monkeypatch, tmp_path, raw):
    mocked_queries(monkeypatch, lambda *args: FakeProcess(raw))
    summary = finished(hardware.NvidiaSmiSampler(tmp_path, max_samples=1).start())
    assert summary["status"] == "invalid_csv" and summary["rows_written"] == 0


@pytest.mark.parametrize("mask,order,logical,expected", [
    ("1,0", "PCI_BUS_ID", 0, "GPU-aaaa-0000"),  # smi indices have the opposite PCI order
    ("1,0", "FASTEST_FIRST", 0, None),
    ("GPU-bbbb,GPU-aaaa", "FASTEST_FIRST", 1, "GPU-aaaa-0000"),
    ("GPU-", "FASTEST_FIRST", 0, None),  # ambiguous prefix
    ("MIG-aaaa", "FASTEST_FIRST", 0, None),
    ("", "FASTEST_FIRST", 0, None),
])
def test_selected_gpu_mapping_does_not_guess_cuda_ordinals(mask, order, logical, expected):
    rows = hardware._parse_rows(csv_response(hardware._BASE_FIELDS), hardware._BASE_FIELDS)
    result = hardware._visibility(rows, {"CUDA_VISIBLE_DEVICES": mask, "CUDA_DEVICE_ORDER": order}, logical)
    assert result["selected_gpu_uuid"] == expected
    if mask == "":
        assert result["resolved_visible_gpu_uuids"] == []
        assert all(row["cuda_visible"] is False for row in rows)


def test_system_info_reports_missing_packages_without_exposing_environment_or_paths(monkeypatch):
    monkeypatch.setenv("DIAGNOSTIC_SECRET_TEST", "never-export-this")
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "GPU-bbbb")
    monkeypatch.setattr(hardware, "_find_nvidia_smi", lambda: None)
    monkeypatch.setattr(hardware, "_IS_WINDOWS", False)
    monkeypatch.setattr(hardware, "_linux_resources", lambda: {
        "cpu_model": "Test CPU", "physical_cores": 12, "memory_total_bytes": 2**35, "memory_available_bytes": 2**34})

    def version(name):
        if name == "ovphysx":
            raise hardware.importlib.metadata.PackageNotFoundError(name)
        return "test-version"

    monkeypatch.setattr(hardware.importlib.metadata, "version", version)
    result = hardware.system_info()
    assert result["cpu"]["physical_cores"] == 12 and result["memory"]["total_bytes"] == 2**35
    assert result["runtime_versions"]["ovphysx"] is None
    assert result["nvidia"]["status"] == "command_unavailable"
    encoded = json.dumps(result, allow_nan=False)
    assert "never-export-this" not in encoded and "DIAGNOSTIC_SECRET_TEST" not in encoded
    assert set(result["nvidia"]["cuda_selection"]["environment"]) == {"CUDA_VISIBLE_DEVICES", "CUDA_DEVICE_ORDER"}
    assert not any(key in result for key in ("hostname", "username", "executable", "environment", "processes"))


def test_linux_resource_parsing_uses_physical_socket_and_core_pair(monkeypatch):
    cpu = "physical id : 0\ncore id : 0\nmodel name : Example CPU\n\nphysical id : 0\ncore id : 0\n\nphysical id : 1\ncore id : 0\n"
    memory = "MemTotal: 1024 kB\nMemAvailable: 512 kB\nMemFree: 100 kB\n"
    monkeypatch.setattr(Path, "read_text", lambda self, **kwargs: cpu if self.name == "cpuinfo" else memory)
    resources = hardware._linux_resources()
    assert resources == {"cpu_model": "Example CPU", "physical_cores": 2,
                         "memory_total_bytes": 1024 * 1024, "memory_available_bytes": 512 * 1024}


@pytest.mark.parametrize("filename", ["gpu-samples.csv", "gpu-summary.json"])
def test_existing_diagnostic_outputs_are_preserved(tmp_path, filename):
    existing = tmp_path / filename
    existing.write_text("existing diagnostic data")
    with pytest.raises(FileExistsError):
        hardware.NvidiaSmiSampler(tmp_path).start()
    assert existing.read_text() == "existing diagnostic data"
    assert sorted(path.name for path in tmp_path.iterdir()) == [filename]


@pytest.mark.parametrize("options", [{"max_samples": 1201}, {"max_samples": 0},
                                      {"interval_seconds": float("nan")}, {"logical_device": -1}])
def test_invalid_sampling_limits_fail_before_start(tmp_path, options):
    with pytest.raises(ValueError):
        hardware.NvidiaSmiSampler(tmp_path, **options)
    assert not list(tmp_path.iterdir())
