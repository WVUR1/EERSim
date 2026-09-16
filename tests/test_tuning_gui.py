"""Tuning capture integrity and Qt lifecycle, without starting physics or PROS."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import os
import subprocess
import sys
import textwrap

import pytest

from eersim.tuning_gui import TuningWindow, project_samples, _error_feedback, gain_explanation


def sample(time, position, source="simulation", capture_id="a"):
    return {"time": time, "source": source, "capture_id": capture_id,
            "motors": {"1": {"position_deg": position, "velocity_rpm": 10, "voltage_mv": 1000}}}


def test_plot_projection_respects_reversals_axis_scaling_and_zero_weight_ports():
    target = {"ports": [1, -2, 3], "axes": {"turn": [.5, -.5, 0]}}
    def motor(position):
        return {"position_deg": position, "velocity_rpm": 0, "voltage_mv": None}
    rows = [{"time": 10, "motors": {"1": motor(2), "2": motor(2)}},
            {"time": 11, "motors": {"1": motor(3), "2": motor(3)}}]
    times, values = project_samples(rows, target, "turn")
    assert times == [0, 1]
    assert values == pytest.approx([0, 2])


def test_capture_drops_other_sources_and_never_concatenates_trials():
    window = SimpleNamespace(_closing=False, _capturing=True, _experiment=False,
                             _capture_source="simulation", _capture_id=None,
                             samples=deque(maxlen=3), _plot_dirty=False)
    TuningWindow._sample(window, sample(10, 0))
    TuningWindow._sample(window, sample(11, 10, "hardware"))
    assert len(window.samples) == 1
    TuningWindow._sample(window, sample(11, 10))
    TuningWindow._sample(window, sample(0, 0, capture_id="b"))
    assert list(window.samples) == [sample(0, 0, capture_id="b")]
    for i in range(1, 6):
        TuningWindow._sample(window, sample(i, i, capture_id="b"))
    assert [row["time"] for row in window.samples] == [3, 4, 5]
    window._capturing = False
    TuningWindow._sample(window, sample(6, 6, capture_id="b"))
    assert len(window.samples) == 3 and window.samples[-1]["time"] == 5


def test_cancel_hardware_recording_does_not_control_simulation_or_robot():
    commands = []
    stopped = []
    window = SimpleNamespace(_jobs={SimpleNamespaceHashable(cancel=SimpleNamespace(set=lambda: stopped.append(True)))},
                             engine=SimpleNamespace(tuning_cancel=lambda: commands.append("cancel"),
                                                    tuning_capture=lambda *args: commands.append("capture")),
                             _hardware_active=True, _capturing=True, _experiment=False,
                             status=SimpleNamespace(setText=lambda text: None), _sync_controls=lambda: None)
    TuningWindow.cancel_work(window)
    assert stopped == [True]
    assert commands == [] and not window._capturing


class SimpleNamespaceHashable(SimpleNamespace):
    __hash__ = object.__hash__


def test_hardware_dependency_and_capture_errors_include_recovery_steps():
    title, message, details = _error_feedback("PROS CLI was not found", "Building telemetry workspace")
    assert "PROS" in title and "Live capture" in message and "CLI…" in message
    assert "Building telemetry workspace" in details
    title, message, _ = _error_feedback("Voltage telemetry is unavailable", "Searching candidate PID gains")
    assert "Capture" in title and "fresh continuous trial" in message and "Characterize" in message


def test_gain_units_and_clipping_are_explained_in_selected_controller_units():
    result = {"gains": {"kp": 202, "ki": 0, "kd": 0},
              "options": {"mode": "position", "output_limit_mv": 3000}}
    text = gain_explanation(result)
    assert "mV/degree" in text and "14.9 degree" in text and "proportional-only" in text
    result["options"]["mode"] = "velocity"
    result["gains"]["ki"] = 2
    text = gain_explanation(result)
    assert "mV/RPM" in text and "14.9 RPM" in text and "proportional-only" not in text
    result["simulation_validation"] = {"status": "travel_limit"}
    assert "VALIDATION TRUNCATED" in gain_explanation(result)


def test_detailed_fit_error_preserves_specific_recovery_and_points_to_guide():
    raw = "Response fit is unreliable: R²=0.2.\n\nTry next: select the full X-drive and reduce the pulse duration."
    title, message, details = _error_feedback(raw, "Searching candidate PID gains")
    assert raw in message and "How to use" in message and "X-drive" in details
    assert "different tuning trial" in title


def test_engine_failure_defers_popup_to_parent_workbench():
    seen = []
    window = SimpleNamespace(_hardware_active=False, _reset_pending=True, _experiment=False,
                             _capturing=False, parent=lambda: SimpleNamespace(feedback=object()),
                             _finish_progress=lambda success: seen.append(("progress", success)),
                             status=SimpleNamespace(setText=lambda text: seen.append(("status", text))),
                             console=SimpleNamespace(append=lambda text: seen.append(("log", text))),
                             _error=lambda text: pytest.fail("Parent already owns the engine-error popup"),
                             _sync_controls=lambda: None)
    TuningWindow._engine_failed(window, "Scene reset failed")
    assert not window._reset_pending and window._operation_failed
    assert ("progress", False) in seen and ("log", "Scene reset failed") in seen


@pytest.mark.parametrize("parent_handles", [True, False])
def test_marked_engine_status_then_failed_reports_once_even_when_messages_differ(parent_handles):
    seen = []
    window = SimpleNamespace(_closing=False, _hardware_active=False, _reset_pending=True,
                             _experiment=False, _capturing=False, _last_engine_error=None,
                             parent=lambda: SimpleNamespace(feedback=object()) if parent_handles else None,
                             _finish_progress=lambda value: seen.append(("progress", value)),
                             status=SimpleNamespace(setText=lambda text: seen.append(("status", text))),
                             console=SimpleNamespace(append=lambda text: seen.append(("log", text))),
                             _error=lambda text: seen.append(("popup", text)), _sync_controls=lambda: None)
    TuningWindow._engine_status(window, {"state": "error", "message": "Tuning stopped: fixture failure", "engine_failure": True})
    TuningWindow._engine_failed(window, "fixture failure")
    popups = [row for row in seen if row[0] == "popup"]
    assert len(popups) == (0 if parent_handles else 1)
    assert not window._reset_pending


def test_unmarked_tuning_validation_error_still_reports_with_parent_workbench():
    popups = []
    window = SimpleNamespace(_closing=False, _hardware_active=False, _reset_pending=False,
                             _experiment=True, _capturing=False, _last_engine_error=None,
                             parent=lambda: SimpleNamespace(feedback=object()),
                             _finish_progress=lambda value: None, _error=popups.append,
                             _sync_controls=lambda: None)
    TuningWindow._engine_status(window, {"state": "error", "message": "Requested port is not bound"})
    assert popups == ["Requested port is not bound"]


def test_qt_window_discovery_experiment_reset_worker_cancel_and_reopen(tmp_path):
    """Separate process avoids QCoreApplication fixtures elsewhere in the suite."""
    script = tmp_path / "qt_tuning_check.py"
    script.write_text(textwrap.dedent('''
        import time
        import math
        import sys
        import types
        from types import SimpleNamespace
        from PySide6.QtCore import QObject, Signal
        from PySide6.QtWidgets import QApplication, QToolButton
        from eersim import tuning_gui as gui
        from eersim import tuning_discovery as discovery

        target = discovery.make_target("Lift", "motor", [1])
        discovery.discover_targets = lambda workspace, config: {"targets": [target], "warnings": []}
        class Settings:
            def __init__(self, *args): self.values = {}
            def value(self, key, default): return self.values.get(key, default)
            def setValue(self, key, value): self.values[key] = value
        gui.QSettings = Settings
        class Engine(QObject):
            tuning_sample = Signal(dict)
            tuning_status = Signal(dict)
            reset_completed = Signal(dict)
            failed = Signal(str)
            def __init__(self): super().__init__(); self.calls=[]
            def tuning_capture(self, *args): self.calls.append(("capture", args))
            def tuning_experiment(self, request): self.calls.append(("experiment", request))
            def tuning_cancel(self): self.calls.append(("cancel",))
            def reset(self): self.calls.append(("reset",))
        app = QApplication([])
        # Multiple motors per drivetrain role require no JSON or custom code.
        editor = gui.GroupDialog()
        editor.kind.setCurrentIndex(editor.kind.findData("h_drive"))
        editor.role_fields["Left"].setText("1, -2")
        editor.role_fields["Right"].setText("3, -4, 5")
        editor.role_fields["Lateral"].setText("6, -7")
        assert not editor.advanced.isChecked()
        editor._save()
        assert editor.target["ports"] == [1, -2, 3, -4, 5, 6, -7]
        assert editor.target["axes"]["forward"] == [1, 1, 1, 1, 1, 0, 0]
        assert editor.target["axes"]["turn"] == [1, 1, -1, -1, -1, 0, 0]
        assert editor.target["axes"]["strafe"] == [0, 0, 0, 0, 0, 1, 1]
        reopened = gui.GroupDialog(editor.target)
        assert reopened.role_fields["Right"].text() == "3, -4, 5"
        reopened.role_fields["Lateral"].setText("1")
        reopened._save()
        assert reopened.target is None and "same physical" in reopened.error.text()
        assert reopened.errors._dialog is not None and not reopened.errors._dialog.isModal()
        editor.close(); reopened.close()
        engine = Engine()
        window = gui.TuningWindow("fixture-workspace", {}, engine)
        closed = []
        window.shutdown_finished.connect(lambda: closed.append(True))
        def pump(predicate, timeout=5):
            end = time.monotonic() + timeout
            while not predicate() and time.monotonic() < end:
                app.processEvents()
                time.sleep(.005)
            app.processEvents()
            assert predicate(), "Qt event deadline exceeded"
        window.show()
        pump(lambda: window.target_combo.count() == 1 and not window._jobs)
        window.resize(1120, 700)
        app.processEvents()
        assert window.height() == 700 and window.minimumSizeHint().height() <= 700
        for index in range(window.tabs.count()):
            window.tabs.setCurrentIndex(index)
            app.processEvents()
            page = window.tabs.widget(index)
            assert page.widgetResizable() and page.widget() is not None
            assert window.height() == 700
        window.tabs.setCurrentIndex(0)
        assert window.tabs.tabText(3) == "How to use"
        assert window.guide_topic.count() == 4
        window.tabs.setCurrentIndex(3)
        for topic in range(window.guide_topic.count()):
            window.guide_topic.setCurrentIndex(topic)
            app.processEvents()
            assert window.guide_text.text() and window.guide_text.hasHeightForWidth()
            assert window.height() == 700
        window.guide_topic.setCurrentIndex(1)
        assert "Stopping recording does not stop the physical robot" in window.guide_text.text()
        window.guide_topic.setCurrentIndex(2)
        assert "integral" in window.guide_text.text() and "20 ms" in window.guide_text.text()
        window.tabs.setCurrentIndex(0)
        assert window.console.isHidden()
        assert window.hardware_log is window.console.editor
        for name in ("target_combo", "axis_combo", "mode_combo", "voltage", "duration", "validation_duration", "travel", "period",
                     "step_target", "large_error", "antiwind", "slot", "hardware_port", "cli_path"):
            assert getattr(window, name).property("settingHelp"), name
        assert len([button for button in window.findChildren(QToolButton) if button.objectName() == "SettingHelp"]) >= 13
        window._toggle_console()
        assert window.console.isVisible()
        window.console.setFloating(True)
        assert window.console.isFloating()
        window.console.hide()
        assert window.characterize_button.isEnabled()
        assert not window.short_pulse_button.isEnabled()
        assert not window.isModal()
        window._characterize()
        assert window.busy and not window.characterize_button.isEnabled()
        request = engine.calls[-1][1]
        assert request["kind"] == "characterize" and request["target"]["ports"] == [1]
        assert request["voltage_mv"] == 3000 and request["controller_period_ms"] == 20
        for i in range(4):
            engine.tuning_sample.emit({"time": i*.02, "source": "simulation", "capture_id": "run1",
                                      "motors": {"1": {"position_deg": i, "velocity_rpm": 10, "voltage_mv": 1000}}})
        engine.tuning_status.emit({"state": "completed", "kind": "characterize", "message": "Finished"})
        assert len(window.samples) == 4 and not window.busy
        assert window.tune_button.isEnabled()
        # Pulse duration is separate from the PID search/validation horizon.
        drive = discovery.make_target("X-drive", "x_drive", [1, 2, 3, 4])
        window.targets.append(drive)
        window._populate_targets(drive["id"])
        assert window.short_pulse_button.isEnabled()
        window.short_pulse_button.click()
        assert window.voltage.value() == 2000 and window.duration.value() == .2
        assert window.validation_duration.value() == 2 and window.travel.value() == 180
        assert window._options()["duration_s"] == 2
        window._characterize()
        assert engine.calls[-1][1]["duration_s"] == .2
        assert not window.short_pulse_button.isEnabled()
        engine.tuning_status.emit({"state": "completed", "message": "Short trial"})
        window._populate_targets(target["id"])
        window.voltage.setValue(3000)
        window.duration.setValue(2)
        exact_period = 1000 / 48
        engine.tuning_status.emit({"state": "completed", "controller_period_ms": exact_period})
        assert window._options()["controller_period_ms"] == exact_period
        window.period.setValue(20)
        assert window._options()["controller_period_ms"] == 20
        window._reset()
        assert window.busy and not window.record_button.isEnabled()
        assert not window.cancel_button.isEnabled()
        engine.tuning_status.emit({"state": "cancelled", "message": "Previous tuning stopped"})
        assert window.progress.maximum() == 0 and "Resetting" in window.status.text()
        engine.reset_completed.emit({})
        assert not window.busy and len(window.samples) == 0

        # A failed reset cannot leave the tuning window permanently disabled.
        window._reset()
        engine.failed.emit("fixture reset error")
        assert not window.busy and window.reset_button.isEnabled()
        assert "fixture reset error" in window.status.text()
        assert window.progress.value() == 0
        assert window.errors._dialog is not None and not window.errors._dialog.isModal()
        window.errors.dismiss()

        # Missing external PROS is checked only for a requested hardware action.
        def missing_cli(cancel, progress):
            raise RuntimeError("PROS CLI was not found. Install the PROS CLI/toolchain or select its pros executable.")
        window._launch("Building the telemetry workspace…", missing_cli)
        pump(lambda: not window._jobs)
        assert window.progress.value() == 0
        assert window.errors._dialog and "Live capture" in window.errors._dialog.text()
        assert not window.errors._dialog.isModal() and window.console.isHidden()
        assert "PROS CLI was not found" in window.console.text()
        window.errors.dismiss()
        window._launch("Cancellable background work", lambda cancel, progress: cancel.wait(3))
        window.cancel_work()
        pump(lambda: not window._jobs)
        assert window.progress.value() == 0 and "cancelled" in window.status.text().lower()
        assert window.errors._dialog is None

        # The real fitting backend is called asynchronously and its candidate
        # uses the exact saved units/period when validating in the engine.
        velocity = position = 0.
        rows = []
        for i in range(300):
            voltage = 2000. if .4 <= i*.02 < 2 else (-1200. if 3 <= i*.02 < 4 else 0.)
            rows.append({"time": i*.02, "source": "simulation", "capture_id": "fit",
                         "motors": {"1": {"position_deg": position, "velocity_rpm": velocity, "voltage_mv": voltage}}})
            following = math.exp(-.02/.16)*velocity + .04*(1-math.exp(-.02/.16))*voltage
            position += (velocity+following)*3*.02
            velocity = following
        window._loaded_capture(rows)
        window._tune()
        pump(lambda: not window._jobs, 10)
        assert window.result and all(key in window.result["gains"] for key in ("kp", "ki", "kd"))
        assert "mV/degree" in window.gain_details.text() and "saturates" in window.gain_details.text()
        assert window.progress.value() == 100
        assert window.validate_button.isEnabled()
        saved_options = dict(window.result["options"])
        saved_candidate = window.result
        window._reset()
        engine.reset_completed.emit({})
        assert window.result is saved_candidate and window.validate_button.isEnabled()
        assert not window.samples
        window._validate()
        request = engine.calls[-1][1]
        assert all(request[key] == value for key, value in saved_options.items())
        for i in range(3):
            engine.tuning_sample.emit({"time": i*.02, "source": "simulation", "capture_id": "validate",
                                      "control_mode": "position", "target": 0 if i == 0 else 90,
                                      "motors": {"1": {"position_deg": i, "velocity_rpm": 10, "voltage_mv": 1000}}})
        window._update_plot()
        assert [series[0] for series in window.capture_plot._series] == ["Recorded response", "Controller target"]
        assert window.capture_plot._series[1][1][-1][1] == 90
        engine.tuning_status.emit({"state": "completed", "kind": "validate", "termination": "travel_limit", "message": "Motor 1 travel limit reached"})
        assert window.result["simulation_validation"]["status"] == "travel_limit"
        assert "VALIDATION TRUNCATED" in window.gain_details.text()
        assert window.progress.value() == 0
        assert window.errors._dialog and "Validation step" in window.errors._dialog.text()
        window.errors.dismiss()
        window.period.setValue(30)
        assert window.result is None and not window.validate_button.isEnabled()
        assert not window.gain_details.text()

        # Exercise GUI hardware sequencing with test doubles, without building,
        # uploading or opening a serial port. Stop logging never commands motors.
        hardware_calls = []
        prepared = SimpleNamespace(workspace="logger-copy")
        success = SimpleNamespace(returncode=0, cancelled=False, timed_out=False, output_tail="")
        fake = types.ModuleType("eersim.tuning_hardware")
        def prepare(workspace, destination, ports, **kwargs):
            hardware_calls.append(("prepare", workspace, destination, ports)); return prepared
        def build(value, **kwargs):
            hardware_calls.append(("build", value)); return success
        def upload(value, **kwargs):
            hardware_calls.append(("upload", kwargs["slot"])); return success
        def terminal(value, **kwargs):
            hardware_calls.append(("record", value))
            kwargs["on_sample"]({"time": 1, "source": "hardware", "capture_id": "hardware1",
                                 "motors": {"1": {"position_deg": 0, "velocity_rpm": 0, "voltage_mv": 0}}})
            kwargs["cancel"].wait(3)
            return success
        fake.prepare_capture, fake.run_build, fake.run_upload, fake.run_terminal = prepare, build, upload, terminal
        sys.modules["eersim.tuning_hardware"] = fake
        gui.QFileDialog.getExistingDirectory = lambda *args: "empty-destination"
        window._hardware_prepare()
        pump(lambda: not window._jobs)
        assert window.build_button.isEnabled() and not window.upload_button.isEnabled()
        window._hardware_build()
        pump(lambda: not window._jobs)
        assert window.upload_button.isEnabled()
        fake.run_build = lambda value, **kwargs: SimpleNamespace(returncode=1, cancelled=False, timed_out=False, output_tail="fixture compiler error")
        window._hardware_build()
        pump(lambda: not window._jobs)
        assert not window.upload_button.isEnabled() and window.progress.value() == 0
        assert window.errors._dialog and "compiler error" in window.errors._dialog.detailedText()
        assert "Console" in window.errors._dialog.text()
        window.errors.dismiss()
        fake.run_build = build
        window._hardware_build()
        pump(lambda: not window._jobs)
        assert window.upload_button.isEnabled() and window.progress.value() == 100
        window.slot.setValue(6)
        window._hardware_upload()
        pump(lambda: not window._jobs)
        assert hardware_calls[-1] == ("upload", 6)
        window._hardware_record()
        pump(lambda: len(window.samples) == 1)
        before = len(engine.calls)
        window.cancel_work()
        pump(lambda: not window._jobs)
        assert len(engine.calls) == before and not window._capturing
        assert window.samples[0]["source"] == "hardware"

        # Background work must see cancellation before the window is disposed.
        window._launch("waiting", lambda cancel, progress: cancel.wait(3))
        assert window.busy
        window.close()
        assert window._closing
        pump(lambda: not window._jobs and bool(closed))
        assert not window.isVisible()

        # The main app can reuse the same nonmodal window after closing it.
        assert window.set_workspace("another-workspace", {})
        window.show()
        pump(lambda: not window._jobs)
        window._record()
        assert engine.calls[-1][0] == "capture"
        engine.tuning_sample.emit({"time": 0, "source": "simulation", "motors": {
            "1": {"position_deg": 0, "velocity_rpm": 0, "voltage_mv": 0}}})
        assert len(window.samples) == 1
        window.cancel_work()
        window.close()
        app.processEvents()
        assert len(closed) == 2 and not window._jobs
        print("Qt tuning lifecycle passed")
    '''), encoding="utf-8")
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1]) + os.pathsep + env.get("PYTHONPATH", "")
    result = subprocess.run([sys.executable, str(script)], capture_output=True, text=True,
                            env=env, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Qt tuning lifecycle passed" in result.stdout
