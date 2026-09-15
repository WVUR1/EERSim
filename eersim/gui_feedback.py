"""Shared, bounded desktop feedback; all widgets live on the Qt thread."""
from __future__ import annotations

from collections import OrderedDict, deque
from datetime import datetime
from html import escape
import math
from pathlib import Path
import sys
import threading
import time
import traceback

from PySide6.QtCore import QEvent, QObject, Qt, QTimer, Signal, Slot
from PySide6.QtWidgets import (
    QApplication, QDockWidget, QFileDialog, QFormLayout, QHBoxLayout, QLabel,
    QMessageBox, QPlainTextEdit, QProgressBar, QPushButton, QToolButton,
    QVBoxLayout, QWhatsThis, QWidget,
)


def _tooltip(text):
    return '<div style="max-width: 370px; white-space: normal;">' + escape(str(text)).replace('\n', '<br>') + '</div>'


def apply_help(widget, title, text):
    """Explain a control to pointer users and screen readers alike."""
    widget.setToolTip(_tooltip(text))
    widget.setWhatsThis(_tooltip(text))
    widget.setAccessibleName(str(title))
    widget.setAccessibleDescription(str(text))
    widget.setProperty("settingHelp", str(text))
    return widget


def help_label(text, explanation, parent=None):
    container = QWidget(parent)
    row = QHBoxLayout(container)
    row.setContentsMargins(0, 0, 0, 0)
    row.setSpacing(5)
    label = QLabel(str(text))
    label.setWordWrap(True)
    row.addWidget(label)
    icon = QToolButton()
    icon.setText("?")
    icon.setObjectName("SettingHelp")
    icon.setFixedSize(20, 20)
    icon.setStyleSheet("QToolButton { padding: 0px; border-radius: 10px; color: #edbd50; }")
    icon.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
    apply_help(icon, "Help for " + str(text), explanation)
    icon.clicked.connect(lambda: QWhatsThis.showText(icon.mapToGlobal(icon.rect().bottomLeft()), _tooltip(explanation), icon))
    row.addWidget(icon)
    row.addStretch(1)
    container.setToolTip(_tooltip(explanation))
    container.setAccessibleName(str(text))
    container.setAccessibleDescription(str(explanation))
    return container


class HelpFormLayout(QFormLayout):
    def addRow(self, *args, help_text=None):
        if len(args) == 3:
            args, help_text = args[:2], args[2]
        if len(args) == 2 and isinstance(args[0], str):
            name, field = args
            explanation = help_text or (field.property("settingHelp") if isinstance(field, QWidget) else None)
            explanation = explanation or (field.toolTip() if isinstance(field, QWidget) else None)
            if explanation:
                if isinstance(field, QWidget):
                    apply_help(field, name, explanation)
                return super().addRow(help_label(name, explanation), field)
        return super().addRow(*args)


class ErrorReporter(QObject):
    """Nonblocking, plain-text dialogs with bounded duplicate suppression.

    Repeated identical errors stay in the console; at most one popup is visible.
    A bounded queue preserves other errors without covering the screen in windows.
    Calls from workers are delivered to this object's Qt thread through a signal.
    """
    requested = Signal(int, str, str, str, str)

    def __init__(self, parent, log=None):
        super().__init__(parent)
        self._parent = parent
        self._log = log
        self._dialog = None
        self._queue = deque(maxlen=8)
        self._recent = OrderedDict()
        self._current_key = None
        self._closing = False
        self._generation = 0
        self._suspended = False
        self.requested.connect(self._show)
        parent.destroyed.connect(self._shutdown)
        parent.installEventFilter(self)

    def error(self, title, message, details=""):
        self.requested.emit(self._generation, "error", str(title), str(message), str(details))

    def warning(self, title, message, details=""):
        self.requested.emit(self._generation, "warning", str(title), str(message), str(details))

    def dismiss(self):
        """Close owned dialogs when a window closes, but allow it to reopen."""
        self._generation += 1
        self._queue.clear()
        if self._dialog is not None:
            self._dialog.reject()

    def eventFilter(self, watched, event):
        if watched is self._parent:
            if event.type() == QEvent.Type.Hide:
                self._suspended = True
                self.dismiss()
            elif event.type() == QEvent.Type.Show:
                self._generation += 1
                self._suspended = False
        return super().eventFilter(watched, event)

    @Slot()
    def _shutdown(self):
        self._closing = True
        self._queue.clear()

    @Slot(int, str, str, str, str)
    def _show(self, generation, severity, title, message, details):
        if self._closing:
            return
        if self._log:
            self._log(f"{severity.upper()}: {title}: {message}" + ("\n" + details if details else ""))
        if generation != self._generation or self._suspended:
            return
        item = severity, title, message, details
        key = severity, title, message
        now = time.monotonic()
        if key == self._current_key or any(row[:3] == key for row in self._queue):
            return
        if now - self._recent.get(key, -math.inf) < 5:
            return
        self._recent[key] = now
        self._recent.move_to_end(key)
        while len(self._recent) > 64:
            self._recent.popitem(last=False)
        if self._dialog is not None:
            self._queue.append(item)
        else:
            self._present(item)

    def _present(self, item):
        severity, title, message, details = item
        self._current_key = item[:3]
        box = QMessageBox(self._parent)
        self._dialog = box
        box.setWindowTitle(title)
        box.setIcon(QMessageBox.Icon.Critical if severity == "error" else QMessageBox.Icon.Warning)
        box.setTextFormat(Qt.TextFormat.PlainText)
        box.setText(message[:3000])
        box.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        # Full messages remain in the bounded console; details are also bounded
        # so a compiler spewing output cannot allocate an unbounded Qt document.
        if details or len(message) > 3000:
            box.setDetailedText((details or message)[-60_000:])
        box.setStandardButtons(QMessageBox.StandardButton.Ok)
        box.setDefaultButton(QMessageBox.StandardButton.Ok)
        box.setEscapeButton(QMessageBox.StandardButton.Ok)
        copy_button = box.addButton("Copy details", QMessageBox.ButtonRole.ActionRole)
        copy_button.clicked.connect(lambda: QApplication.clipboard().setText(title + "\n" + message + ("\n" + details if details else "")))
        box.finished.connect(self._finished)
        box.setWindowModality(Qt.WindowModality.NonModal)
        box.show()

    @Slot(int)
    def _finished(self, result):
        old, self._dialog = self._dialog, None
        self._current_key = None
        if old is not None:
            old.deleteLater()
        if self._queue and not self._closing:
            QTimer.singleShot(0, self._next)

    def _next(self):
        if not self._closing and self._dialog is None and self._queue:
            self._present(self._queue.popleft())


class ExceptionFeedback:
    """Report unexpected Python callbacks while preserving terminal tracebacks.

    Installed only for the desktop's event loop, never for headless CLI runs.
    Native crashes cannot be caught by a Python exception hook.
    """
    def __init__(self, reporter):
        self.reporter = reporter
        self._previous = sys.excepthook
        self._previous_thread = threading.excepthook
        self._hook = self._exception
        self._thread_hook = self._thread_exception
        sys.excepthook = self._hook
        threading.excepthook = self._thread_hook

    def _exception(self, kind, value, trace):
        self._previous(kind, value, trace)
        if not issubclass(kind, (SystemExit, KeyboardInterrupt)):
            self._report(kind, value, trace)

    def _thread_exception(self, args):
        self._previous_thread(args)
        if not issubclass(args.exc_type, (SystemExit, KeyboardInterrupt)):
            self._report(args.exc_type, args.exc_value, args.exc_traceback)

    def _report(self, kind, value, trace):
        self.reporter.error("Unexpected application error",
                            "An application operation failed. Open the console or copy the details below. "
                            "If the operation still fails when retried, save the console log and restart EERSim.",
                            "".join(traceback.format_exception(kind, value, trace)))

    def restore(self):
        if sys.excepthook is self._hook:
            sys.excepthook = self._previous
        if threading.excepthook is self._thread_hook:
            threading.excepthook = self._previous_thread


class FeedbackConsole(QDockWidget):
    """Optional detachable console; coalesce logs instead of repainting each line."""
    message = Signal(str)

    def __init__(self, parent, title="Console"):
        super().__init__(title, parent)
        self.setObjectName("FeedbackConsole")
        self.setStyleSheet("QDockWidget { background: #101e2d; color: #edf3f8; } "
                           "QDockWidget::title { background: #1b2c40; padding: 5px; }")
        self.setFeatures(QDockWidget.DockWidgetFeature.DockWidgetClosable |
                         QDockWidget.DockWidgetFeature.DockWidgetMovable |
                         QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        self.setAllowedAreas(Qt.DockWidgetArea.BottomDockWidgetArea | Qt.DockWidgetArea.TopDockWidgetArea)
        self.setMinimumWidth(300)
        contents = QWidget()
        contents.setObjectName("Root")
        layout = QVBoxLayout(contents)
        layout.setContentsMargins(8, 6, 8, 8)
        actions = QHBoxLayout()
        caption = QLabel("Build output, runtime messages and error details")
        caption.setWordWrap(True)
        actions.addWidget(caption, 1)
        self.detach_button = QPushButton("Detach")
        self.detach_button.clicked.connect(lambda: self.setFloating(not self.isFloating()))
        self.topLevelChanged.connect(lambda floating: self.detach_button.setText("Dock" if floating else "Detach"))
        actions.addWidget(self.detach_button)
        for name, callback in (("Copy", self.copy), ("Save…", self.save), ("Clear", self.clear)):
            control = QPushButton(name)
            control.clicked.connect(callback)
            actions.addWidget(control)
        layout.addLayout(actions)
        self.editor = QPlainTextEdit()
        self.editor.setReadOnly(True)
        self.editor.setMaximumBlockCount(4000)
        self.editor.setMinimumHeight(140)
        self.editor.setAccessibleName("Console messages")
        layout.addWidget(self.editor)
        self.setWidget(contents)
        self._pending = deque(maxlen=4000)
        self._pending_chars = 0
        self._dropped = 0
        self._timer = QTimer(self)
        self._timer.setInterval(100)
        self._timer.timeout.connect(self.flush)
        self.visibilityChanged.connect(self._visibility_changed)
        self.message.connect(self._enqueue)
        self.errors = ErrorReporter(parent, log=self.append)
        parent.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self)
        self.hide()  # Every launch starts quiet, including after a floating session.
        action = self.toggleViewAction()
        action.setText("Console")
        action.setShortcut("Ctrl+Shift+C")
        action.setToolTip("Show or hide the detachable build and runtime console")

    def append(self, message):
        self.message.emit(str(message))

    @Slot(str)
    def _enqueue(self, message):
        stamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{stamp}] {message[-20_000:]}"
        while self._pending and (len(self._pending) == self._pending.maxlen or self._pending_chars + len(line) > 1_000_000):
            self._pending_chars -= len(self._pending.popleft())
            self._dropped += 1
        self._pending.append(line)
        self._pending_chars += len(line)
        if self.isVisible() and not self._timer.isActive():
            self._timer.start()

    @Slot(bool)
    def _visibility_changed(self, visible):
        if visible:
            self.flush()
        else:
            self._timer.stop()

    @Slot()
    def flush(self):
        if self._pending or self._dropped:
            follow = self.editor.verticalScrollBar().value() >= self.editor.verticalScrollBar().maximum() - 2
            lines = list(self._pending)
            self._pending.clear()
            self._pending_chars = 0
            if self._dropped:
                lines.insert(0, f"[Console] {self._dropped} older queued messages discarded; save logs sooner for longer captures.")
                self._dropped = 0
            self.editor.appendPlainText("\n".join(lines))
            if follow:
                self.editor.verticalScrollBar().setValue(self.editor.verticalScrollBar().maximum())
        self._timer.stop()

    def text(self):
        self.flush()
        return self.editor.toPlainText()

    @Slot()
    def clear(self):
        self._pending.clear()
        self._pending_chars = 0
        self._dropped = 0
        self.editor.clear()

    @Slot()
    def copy(self):
        QApplication.clipboard().setText(self.text())

    @Slot()
    def save(self):
        filename, _ = QFileDialog.getSaveFileName(self, "Save console log", "eersim-console.txt", "Text files (*.txt);;All files (*)")
        if filename:
            try:
                Path(filename).write_text(self.text(), encoding="utf-8")
            except OSError as exc:
                self.errors.error("Could not save console", "Choose a writable folder and try again.", str(exc))


class OperationProgress(QWidget):
    """Loading feedback distinct from simulation/replay time.

    Native scene work has unknown duration: it uses an indeterminate bar.
    Operation IDs prevent a late completion from hiding a newer operation.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel()
        self.label.setTextFormat(Qt.TextFormat.PlainText)
        self.label.setWordWrap(True)
        self.bar = QProgressBar()
        self.bar.setMinimumWidth(140)
        self.bar.setMaximumWidth(280)
        self.bar.setTextVisible(False)
        self.bar.setAccessibleName("Operation progress")
        layout.addWidget(self.label, 1)
        layout.addWidget(self.bar)
        self._active = OrderedDict()
        self._finished_ids = deque(maxlen=128)
        self.hide()

    def begin(self, key, message):
        self._active[("local", key)] = {"message": str(message), "fraction": None}
        self._refresh()

    def finish(self, key, failed=False):
        self._active.pop(("local", key), None)
        self._refresh()

    @Slot(dict)
    def update_event(self, event):
        key = event.get("id")
        if key is None or key in self._finished_ids:
            return
        self._active.pop(("local", event.get("action")), None)
        if event.get("state") in {"completed", "failed", "cancelled"}:
            self._active.pop(("engine", key), None)
            self._finished_ids.append(key)
        elif event.get("state") == "running":
            self._active[("engine", key)] = dict(event)
        self._refresh()

    def clear(self):
        self._active.clear()
        self.hide()

    def _refresh(self):
        if not self._active:
            self.hide()
            return
        event = next(reversed(self._active.values()))
        message = str(event.get("message", "Working…"))
        self.label.setText(message)
        self.bar.setAccessibleDescription(message)
        fraction = event.get("fraction")
        if isinstance(fraction, (int, float)) and math.isfinite(fraction):
            self.bar.setRange(0, 1000)
            self.bar.setValue(round(max(0, min(1, fraction)) * 1000))
        else:
            self.bar.setRange(0, 0)
        self.show()
