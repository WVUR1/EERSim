"""Pure logger/workspace/parser tests; command tests run only Python helpers."""
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import threading
import time

import pytest

from eersim import tuning_hardware as hardware


def workspace(tmp_path, *, source=None):
    root = tmp_path / "Original project"
    (root / "src").mkdir(parents=True)
    (root / "include" / "pros").mkdir(parents=True)
    (root / "firmware").mkdir()
    (root / "Makefile").write_text("CEXTS:=c\nSRCDIR:=./src\n-include ./common.mk\n")
    (root / "common.mk").write_text("# $(CEXTS) $(SRCDIR)\n")
    (root / "project.pros").write_text(json.dumps({"py/state": {"target": "v5"}}))
    (root / "include" / "pros" / "motors.h").write_text("double motor_get_position(int8_t port);\n")
    (root / "firmware" / "libpros.a").write_bytes(b"firmware fixture\x00\x01")
    (root / "src" / "main.cpp").write_text(source or '#include "main.h"\nvoid initialize() { original_call(); }\nvoid opcontrol() { drive(); }\n')
    return root


def snapshot(path):
    return {p.relative_to(path).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in path.rglob("*") if p.is_file()}


def test_prepare_preserves_original_and_other_callbacks_and_avoids_cache(tmp_path):
    original = workspace(tmp_path)
    for name in (".git", ".eersim", ".sim-tools", "bin", "build", "cache"):
        (original / name).mkdir()
        (original / name / "omitted").write_bytes(b"not copied")
    before = snapshot(original)
    messages = []
    result = hardware.prepare_capture(original, tmp_path / "logger copy", [-3, 1, 3], log=messages.append)
    assert snapshot(original) == before
    assert result.ports == (1, 3) and result.sample_ms == 20
    assert result.source_workspace == original.resolve()
    copied = snapshot(result.workspace)
    assert not any(name.split("/")[0] in hardware._EXCLUDED for name in copied)
    assert copied["firmware/libpros.a"] == before["firmware/libpros.a"]
    updated = (result.workspace / "src/main.cpp").read_text()
    assert 'extern "C"' in updated
    assert 'void eersim_tuning_user_initialize() { original_call(); }' in updated
    assert 'void opcontrol() { drive(); }' in updated
    assert (result.workspace / "Makefile").read_bytes() == (original / "Makefile").read_bytes()
    metadata = json.loads(result.metadata_path.read_text())
    assert metadata["passive"] and metadata["ports"] == [1, 3]
    assert metadata["source_sha256"]["src/main.cpp"] == before["src/main.cpp"]
    logger = (result.workspace / "src/eersim_tuning_logger.c").read_text()
    for forbidden in ("motor_move", "motor_set", "motor_tare", "motor_brake"):
        assert forbidden not in logger
    assert "ports[] = {1, 3}" in logger
    assert logger.index("eersim_tuning_user_initialize();") < logger.index("logger_task = task_create")
    assert messages


@pytest.mark.parametrize("source", [
    '/* void initialize() { fake(); } */\nvoid initialize(void) { real(); }',
    'const char* example = "void initialize() { fake(); }";\nvoid initialize() { real(); }',
    'const char* example = R"tag(void initialize() { fake(); })tag";\nvoid initialize() { real(); }',
    'extern "C" { void initialize(void) { real(); } }',
    'extern "C" void initialize() { real(); }',
    'struct Local { void initialize() { ignored(); } };\nvoid initialize() { real(); }',
    'namespace local { void initialize() { ignored(); } }\nvoid initialize() { real(); }',
    'void initialize();\nvoid initialize /* name */ ( /* args */ void ) /* body */ { real(); }',
])
def test_callback_scope_literals_and_comments(source):
    offsets = hardware._callback_offsets(source)
    assert len(offsets) == 1
    start, end = offsets[0]
    assert source[start:end] == "initialize"
    assert "real();" in source[end:]


@pytest.mark.parametrize("source", [
    'namespace local { void initialize() {} }',
    'struct Local { void initialize() {} };',
    'void Namespace::initialize() {}',
    '#define CALLBACK initialize\nvoid CALLBACK() {}',
    '#define MAKE_CALLBACK void initialize() {}\nMAKE_CALLBACK',
    '#define MAKE_CALLBACK \\\nvoid initialize() {}\nMAKE_CALLBACK',
])
def test_callback_unsupported_definitions_not_guessed(tmp_path, source):
    original = workspace(tmp_path, source=source)
    with pytest.raises(hardware.CaptureError, match="one ordinary global"):
        hardware.prepare_capture(original, tmp_path / "copy", [1])
    assert not (tmp_path / "copy").exists()


@pytest.mark.parametrize("source", ["static void initialize() {}", "inline void initialize() {}"])
def test_callback_internal_linkage_rejected(source):
    with pytest.raises(hardware.CaptureError, match="externally linked"):
        hardware._callback_offsets(source)


def test_prepare_conditional_duplicates_rejected(tmp_path):
    original = workspace(tmp_path, source="#if A\nvoid initialize() {}\n#else\nvoid initialize() {}\n#endif\n")
    with pytest.raises(hardware.CaptureError, match="one ordinary global"):
        hardware.prepare_capture(original, tmp_path / "copy", [1])


@pytest.mark.parametrize("ports,interval", [([], 20), ([0], 20), ([22], 20), ([True], 20), ([1.0], 20), ([1], 19), ([1], 1001), ([1], True)])
def test_port_and_rate_limits(tmp_path, ports, interval):
    original = workspace(tmp_path)
    with pytest.raises(hardware.CaptureError):
        hardware.prepare_capture(original, tmp_path / "copy", ports, interval)


def test_prepare_exclusive_destination_and_no_overlap(tmp_path):
    original = workspace(tmp_path)
    existing = tmp_path / "existing"
    existing.mkdir()
    (existing / "keep").write_text("mine")
    for dest in (existing, original, original / "child", tmp_path):
        with pytest.raises(hardware.CaptureError):
            hardware.prepare_capture(original, dest, [1])
    assert (existing / "keep").read_text() == "mine"


def test_prepare_legacy_reversal_fails_explicitly(tmp_path):
    original = workspace(tmp_path)
    (original / "include/pros/motors.h").write_text("int32_t motor_is_reversed(int8_t port);")
    with pytest.raises(hardware.CaptureError, match="legacy PROS"):
        hardware.prepare_capture(original, tmp_path / "copy", [1])


def test_prepare_cancel_before_copy_leaves_source_untouched(tmp_path):
    original = workspace(tmp_path)
    before = snapshot(original)
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(hardware.CaptureCancelled):
        hardware.prepare_capture(original, tmp_path / "copy", [1], cancel=cancel)
    assert snapshot(original) == before and not (tmp_path / "copy").exists()


def test_prepare_copy_failure_cleans_only_owned_destination(tmp_path, monkeypatch):
    original = workspace(tmp_path)
    before = snapshot(original)
    def fail(*args):
        raise OSError("fake copy failure")
    monkeypatch.setattr(hardware.shutil, "copymode", fail)
    with pytest.raises(OSError, match="fake copy"):
        hardware.prepare_capture(original, tmp_path / "copy", [1])
    assert snapshot(original) == before and not (tmp_path / "copy").exists()


def test_prepare_link_never_followed(tmp_path, monkeypatch):
    original = workspace(tmp_path)
    real = hardware._is_link
    monkeypatch.setattr(hardware, "_is_link", lambda p: p.name == "libpros.a" or real(p))
    with pytest.raises(hardware.CaptureError, match="symlinks or junctions"):
        hardware.prepare_capture(original, tmp_path / "copy", [1])


def test_parser_valid_units_null_and_ansi_chatter():
    sample = hardware.parse_telemetry_line("\x1b[31mdebug: EERSIM_TUNE,1,4294967295,3,-360.5,12.5,-12000,null,12400\x1b[0m\r\n")
    assert sample == hardware.HardwareSample(4294967295, 3, -360.5, 12.5, -12000, None, 12400)


@pytest.mark.parametrize("line", [
    "plain PROS output", "EERSIM_TUNE,2,0,1,0,0,0,0,0", "EERSIM_TUNE,1,-1,1,0,0,0,0,0",
    "EERSIM_TUNE,1,0,22,0,0,0,0,0", "EERSIM_TUNE,1,0,1,nan,0,0,0,0",
    "EERSIM_TUNE,1,0,1,0,inf,0,0,0", "EERSIM_TUNE,1,0,1,0,0,0,0,0,trailing",
    "EERSIM_TUNE,1,0,1,0,0,0,0", "EERSIM_TUNE,1,0,1,0,0,0,0,nonsense",
    "x" * 20000 + "EERSIM_TUNE,1,0,1,0,0,0,0,0",
])
def test_parser_rejects_malformed_rows(line):
    assert hardware.parse_telemetry_line(line) is None


def sample(timestamp, port, position=20):
    return hardware.HardwareSample(timestamp, port, position, 60, 2000, None, 12000)


def test_complete_scans_exact_time_wrap_and_final_row_emitted():
    output = []
    reader = hardware.ScanAssembler([-1, 3], output.append, capture_id="capture")
    reader.feed(sample(0xFFFFFFF0, 1))
    assert output == []
    reader.feed(sample(0xFFFFFFF0, 3))
    reader.feed(sample(4, 3, 30))
    reader.feed(sample(4, 1, 40))
    reader.finish()
    assert [row["time"] for row in output] == [0.0, .02]
    assert all(row["source"] == "hardware" and row["capture_id"] == "capture" for row in output)
    assert output[1]["motors"]["1"]["position_deg"] == 40
    assert output[1]["motors"]["3"]["current_ma"] is None
    assert reader.emitted == 2 and reader.dropped == 0


def test_missing_invalid_duplicate_scans_never_zero_filled():
    output, logs = [], []
    reader = hardware.ScanAssembler([1, 2], output.append, logs.append)
    reader.feed(sample(0, 1))
    reader.feed(sample(20, 1, None))
    reader.feed(sample(20, 2))
    reader.feed(sample(40, 1))
    reader.feed(sample(40, 1))
    reader.feed(sample(40, 2))
    reader.feed(sample(60, 1))
    reader.finish()
    reader.finish()
    assert output == [] and reader.dropped == 4 and logs


def test_robot_restart_is_not_silently_joined_to_capture():
    reader = hardware.ScanAssembler([1], lambda row: None)
    reader.feed(sample(40000, 1))
    with pytest.raises(hardware.CaptureError, match="restarted"):
        reader.feed(sample(10, 1))


def test_cli_discovery_paths_spaces_and_batch_rejection(tmp_path, monkeypatch):
    cli = tmp_path / ("pros tool.exe" if os.name == "nt" else "pros tool")
    cli.write_text("fixture")
    cli.chmod(cli.stat().st_mode | stat.S_IXUSR)
    assert hardware.discover_pros_cli(cli) == cli.resolve()
    for suffix in (".cmd", ".bat", ".ps1", ".py"):
        path = tmp_path / ("pros" + suffix)
        path.write_text("echo hello")
        assert hardware.discover_pros_cli(path) is None
    monkeypatch.setenv("PROS_CLI", str(cli))
    assert hardware.discover_pros_cli() == cli.resolve()
    assert hardware.discover_pros_cli(tmp_path / "missing") is None


def test_explicit_build_upload_terminal_argv_and_complete_capture(tmp_path, monkeypatch):
    original = workspace(tmp_path)
    prepared = hardware.prepare_capture(original, tmp_path / "copy", [1, 2])
    cli = Path("/mock cli/pros.exe")
    monkeypatch.setattr(hardware, "_cli", lambda value: cli)
    commands = []
    def run(command, **kwargs):
        commands.append((command, kwargs))
        if "terminal" in command:
            for line in ("connected", "EERSIM_TUNE,1,100,1,10,20,300,null,12000", "EERSIM_TUNE,1,100,2,30,40,500,100,12000"):
                kwargs["log"](line)
        return hardware.CommandResult(0, False, False, "", tuple(command))
    monkeypatch.setattr(hardware, "run_command", run)
    assert hardware.run_build(prepared).ok
    assert commands[-1][0] == [str(cli), "make"]
    hardware.run_upload(prepared, slot=3, port="COM8")
    assert commands[-1][0] == [str(cli), "upload", "--slot", "3", "--after", "screen", str(prepared.workspace), "COM8"]
    frames, logs = [], []
    hardware.run_terminal(prepared, port="COM9", on_sample=frames.append, log=logs.append)
    assert commands[-1][0] == [str(cli), "terminal", "--backend", "solo", "--no-banner", "COM9"]
    assert commands[-1][1]["terminal"] is True
    assert len(frames) == 1 and frames[0]["motors"]["2"]["position_deg"] == 30
    assert "not stopped" in logs[-1]
    assert all("run" not in command and "--raw" not in command for command, _ in commands)


def test_command_cancelled_before_launch_and_rejects_shell(tmp_path, monkeypatch):
    cancel = threading.Event()
    cancel.set()
    monkeypatch.setattr(hardware.subprocess, "Popen", lambda *a, **k: pytest.fail("must not start"))
    result = hardware.run_command(["unused"], cwd=tmp_path, timeout_s=1, cancel=cancel)
    assert result.cancelled
    for command in (["pros.cmd"], [], ["bad\0name"]):
        with pytest.raises(hardware.CaptureError):
            hardware.run_command(command, cwd=tmp_path, timeout_s=1)


def test_command_streams_partial_chunks_unicode_and_bounded_tail(tmp_path):
    lines = []
    code = "import sys; sys.stdout.buffer.write(('hello Ω\\n' + 'x'*40000 + '\\nlast').encode('utf-8')); sys.stdout.flush()"
    result = hardware.run_command([sys.executable, "-c", code], cwd=tmp_path, timeout_s=10, log=lines.append)
    assert result.ok and lines[0] == "hello Ω" and lines[-1] == "last"
    assert len(result.output_tail) == 32768 and result.output_tail.endswith("last")


@pytest.mark.parametrize("cancelled", [False, True])
def test_command_timeout_cancel_owns_descendants(tmp_path, cancelled):
    # Only Python child helpers: no PROS, device, physics, window or GL context.
    child_file = tmp_path / "late-child-output"
    child = "import time; from pathlib import Path; time.sleep(2); Path(%r).write_text('escaped')" % str(child_file)
    parent = "import subprocess,sys,time; subprocess.Popen([sys.executable,'-c',%r]); print('started',flush=True); time.sleep(20)" % child
    cancel = threading.Event()
    def log(line):
        if cancelled and line == "started":
            cancel.set()
    started = time.monotonic()
    result = hardware.run_command([sys.executable, "-c", parent], cwd=tmp_path, timeout_s=.3 if not cancelled else 5,
                                  cancel=cancel, log=log)
    assert time.monotonic() - started < 5
    assert result.cancelled == cancelled and result.timed_out != cancelled
    time.sleep(2.1)
    assert not child_file.exists()


def test_command_callback_failure_cleans_child(tmp_path):
    child_file = tmp_path / "late-output"
    code = "from pathlib import Path; import time; print('hello',flush=True); time.sleep(1); Path(%r).touch()" % str(child_file)
    def fail(line):
        raise ValueError("callback failed")
    with pytest.raises(ValueError, match="callback failed"):
        hardware.run_command([sys.executable, "-c", code], cwd=tmp_path, timeout_s=5, log=fail)
    time.sleep(1.1)
    assert not child_file.exists()


def test_terminal_discards_unterminated_or_overlong_csv_rows(tmp_path):
    lines = []
    code = "import sys; sys.stdout.write('x'*17000 + 'EERSIM_TUNE,1,0,1,0,0,0,0,0\\nEERSIM_TUNE,1,20,1,0,0,0,0,120'); sys.stdout.flush()"
    result = hardware.run_command([sys.executable, "-c", code], cwd=tmp_path, timeout_s=5,
                                  log=lines.append, terminal=True)
    assert result.ok and lines == []


@pytest.mark.skipif(os.name == "nt", reason="POSIX terminal contract")
def test_terminal_has_private_tty_input_without_sending_keys(tmp_path):
    lines = []
    code = "import termios,sys; termios.tcgetattr(0); print('tty ready',flush=True)"
    assert hardware.run_command([sys.executable, "-c", code], cwd=tmp_path, timeout_s=5,
                                terminal=True, log=lines.append).ok
    assert lines == ["tty ready"]
