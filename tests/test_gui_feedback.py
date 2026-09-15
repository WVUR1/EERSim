"""Real Qt feedback checks in a subprocess; no physics, GPU or robot required."""
import os
from pathlib import Path
import subprocess
import sys
import textwrap


def test_qt_feedback_console_errors_help_and_progress(tmp_path):
    script = tmp_path / "feedback_check.py"
    script.write_text(textwrap.dedent('''
        import sys
        import threading
        from PySide6.QtCore import QCoreApplication, QEvent, Qt
        from PySide6.QtWidgets import QApplication, QDoubleSpinBox, QMainWindow, QWidget, QToolButton
        from eersim.gui_feedback import (FeedbackConsole, ErrorReporter, HelpFormLayout,
                                         OperationProgress, ExceptionFeedback)
        app = QApplication([])
        window = QMainWindow()
        window.setCentralWidget(QWidget())
        console = FeedbackConsole(window)
        window.addAction(console.toggleViewAction())
        window.show()
        app.processEvents()
        assert not console.isVisible()
        console.append("hidden output")
        assert "hidden output" in console.text()
        assert not console.isVisible()
        for i in range(150): console.append(str(i) + "x" * 19990)
        assert console._pending_chars <= 1_000_000 and console._dropped > 0
        console.toggleViewAction().trigger()
        app.processEvents()
        assert console.isVisible() and "discarded" in console.text()
        console.setFloating(True)
        app.processEvents()
        assert console.isFloating() and console.isWindow()
        console.close()
        app.processEvents()
        console.append("after close")
        assert not console.isVisible() and "after close" in console.text()
        console.toggleViewAction().trigger()
        console.setFloating(False)
        console.clear()
        assert console.text() == ""

        form = HelpFormLayout(window.centralWidget())
        rate = QDoubleSpinBox()
        form.addRow("Physics rate", rate, "Steps per simulated second; higher rates cost more CPU time.")
        icons = window.findChildren(QToolButton, "SettingHelp")
        assert len(icons) == 1 and "Physics rate" in icons[0].accessibleName()
        assert "CPU" in rate.accessibleDescription() and "CPU" in icons[0].toolTip()
        assert icons[0].focusPolicy() == Qt.FocusPolicy.StrongFocus

        progress = OperationProgress(window)
        progress.begin("configure", "Queued scene load")
        assert not progress.isHidden() and progress.bar.maximum() == 0
        progress.update_event(dict(id=1, action="configure", state="running", message="Loading URDF", fraction=None))
        assert len(progress._active) == 1 and progress.bar.maximum() == 0
        progress.update_event(dict(id=2, action="build", state="running", message="Compiling", fraction=.25))
        assert progress.bar.value() == 250
        progress.update_event(dict(id=1, action="configure", state="completed", fraction=1))
        assert not progress.isHidden() and progress.label.text() == "Compiling"
        progress.update_event(dict(id=2, action="build", state="failed"))
        assert progress.isHidden()
        progress.update_event(dict(id=2, action="build", state="running", message="Stale update"))
        assert progress.isHidden()
        progress.begin("reset", "Resetting")
        progress.finish("reset", failed=True)
        assert progress.isHidden()

        logs = []
        reporter = ErrorReporter(window, log=logs.append)
        reporter.error("Build failed", "Install compiler <SDK>", "compiler details")
        first = reporter._dialog
        assert first is not None and first.isVisible()
        assert first.windowModality() == Qt.WindowModality.NonModal
        assert first.textFormat() == Qt.TextFormat.PlainText
        for i in range(20): reporter.error("Build failed", "Install compiler <SDK>", str(i))
        assert reporter._dialog is first and not reporter._queue and len(logs) == 21
        reporter.warning("Assets missing", "Choose a valid robot file")
        assert len(reporter._queue) == 1
        first.accept()
        app.processEvents()
        assert reporter._dialog.windowTitle() == "Assets missing"
        reporter.dismiss()
        assert reporter._dialog is None and not reporter._queue
        # Worker-originated messages are queued onto the GUI thread.
        thread = threading.Thread(target=lambda: reporter.error("Worker stopped", "Try Reset"))
        thread.start(); thread.join()
        assert reporter._dialog is None
        app.processEvents()
        assert reporter._dialog.windowTitle() == "Worker stopped"
        reporter.dismiss()

        previous = sys.excepthook
        recorded = []
        sys.excepthook = lambda *args: recorded.append(args)
        hook = ExceptionFeedback(reporter)
        try:
            try: raise ValueError("callback problem")
            except ValueError: sys.excepthook(*sys.exc_info())
            assert recorded and "callback problem" in reporter._dialog.detailedText()
        finally:
            hook.restore()
            assert sys.excepthook is not hook._hook
            sys.excepthook = previous
        reporter.dismiss()
        console.errors.dismiss()
        # A worker notification queued before close cannot resurrect a window.
        thread = threading.Thread(target=lambda: reporter.error("Late worker", "finished during close"))
        thread.start(); thread.join()
        window.close()
        reporter.dismiss()
        app.processEvents()
        assert reporter._dialog is None and not window.isVisible()
        window.show()
        app.processEvents()
        reporter.error("New attempt", "feedback works after reopen")
        assert reporter._dialog is not None and reporter._dialog.isVisible()
        reporter.dismiss()
        window.close()
        window.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        app.processEvents()
    '''), encoding="utf-8")
    result = subprocess.run([sys.executable, str(script)], cwd=Path(__file__).resolve().parents[1],
                            env={**os.environ, "QT_QPA_PLATFORM": "offscreen"},
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def test_launcher_failure_popup_is_desktop_only(monkeypatch, tmp_path, capsys):
    from eersim import __main__ as launcher
    messages = []
    monkeypatch.setattr(launcher, "_show_startup_error", messages.append)
    missing = str(tmp_path / "missing-profile.json")
    assert launcher.main(["--headless", "--config", missing]) == 1
    assert not messages and "simulator:" in capsys.readouterr().err
    assert launcher.main(["--config", missing]) == 1
    assert len(messages) == 1
