"""Nonmodal joint and drivetrain tuning workbench.

The window samples an existing simulation engine. Discovery, file operations,
identification and hardware subprocesses run outside the Qt presentation thread.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import threading
import uuid
from collections import deque
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, QSettings, Qt, QThread, QTimer, Signal, Slot
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
    QFileDialog, QHBoxLayout, QLabel, QLineEdit, QMainWindow,
    QPlainTextEdit, QProgressBar, QPushButton, QScrollArea, QSpinBox,
    QTabWidget, QVBoxLayout, QWidget,
)

from .gui_styles import GOLD, STYLESHEET
from .gui_feedback import FeedbackConsole, ErrorReporter, HelpFormLayout, help_label, apply_help


MAX_SAMPLES = 60_000
ROLE_AXES = {
    "tank": {"Left": {"forward": 1, "turn": 1}, "Right": {"forward": 1, "turn": -1}},
    "h_drive": {"Left": {"forward": 1, "turn": 1, "strafe": 0},
                "Right": {"forward": 1, "turn": -1, "strafe": 0},
                "Lateral": {"forward": 0, "turn": 0, "strafe": 1}},
    "x_drive": {"Front left": {"forward": 1, "turn": 1, "strafe": 1},
                "Front right": {"forward": 1, "turn": -1, "strafe": -1},
                "Rear left": {"forward": 1, "turn": 1, "strafe": -1},
                "Rear right": {"forward": 1, "turn": -1, "strafe": 1}},
}


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (ValueError, TypeError):
        return False


def _error_feedback(message, operation):
    """Keep user recovery steps beside the failure; raw output stays available."""
    text = str(message).strip() or "The operation did not complete."
    lower, stage = text.lower(), str(operation).lower()
    if ("pros" in lower or "arm-none-eabi" in lower) and any(word in lower for word in ("not found", "not installed", "no such file", "does not exist")):
        title = "PROS tools are unavailable"
        recovery = "Install the PROS CLI and its ARM toolchain. In Live capture, use CLI… to select the installed executable, then retry the hardware action."
    elif "upload" in stage:
        title = "Upload did not complete"
        recovery = "Check the Brain's USB connection, serial port and selected program slot. Build the logger workspace successfully before retrying Upload. The console contains the CLI output."
    elif "build" in stage:
        title = "Logger build did not complete"
        recovery = "Check the first compiler error in Console and confirm the PROS CLI and ARM toolchain are installed. Correct the logger workspace or prepare a fresh copy, then retry Build."
    elif "prepar" in stage:
        title = "Logger copy could not be prepared"
        recovery = "Choose a writable parent folder outside the selected PROS workspace, confirm the source workspace contains a PROS project, and retry Prepare logger copy."
    elif "record" in stage or "connected logger" in stage:
        title = "Recording did not complete"
        recovery = "Check the serial port and USB connection. The Brain must be running a program built from the prepared logger workspace. Reconnect and record a fresh trial. Stopping recording does not stop the robot program."
    elif any(word in stage for word in ("analyz", "candidate", "capture", "reading")) and any(word in lower for word in ("voltage", "capture", "encoder", "sample", "timestamp", "excitation")):
        title = "Capture cannot be used for tuning"
        recovery = "Select the matching target and axis, then record a fresh continuous trial with a baseline, a voltage pulse and its coast response. Characterize in simulation provides this pulse; position/velocity commands may not expose the voltage needed to identify a model."
    else:
        title = "Tuning operation did not complete"
        recovery = "Review the message and Console details, correct the indicated setting, then retry the action. Export any useful capture before restarting the simulator."
    return title, text + "\n\n" + recovery, "Operation: " + str(operation) + "\n\n" + text


def project_samples(samples, target, axis, mode="position"):
    """Small presentation-only projection; analysis owns the fitted model."""
    from .tuning import project_sample
    field = {"position": "position_deg", "velocity": "velocity_rpm", "voltage": "voltage_mv"}[mode]
    points = []
    origin = None
    for sample in samples:
        if origin is None and _finite(sample.get("time")):
            origin = float(sample["time"])
        try:
            projected = project_sample(sample, target, axis)
        except (ValueError, TypeError, KeyError):
            continue
        if not _finite(projected.get(field)):
            continue
        points.append((float(projected["time"]), float(projected[field])))
    if not points:
        return [], []
    start, zero = points[0]
    start = origin if origin is not None else start
    return ([t - start for t, _ in points],
            [value - zero if mode == "position" else value for _, value in points])


class ResponsePlot(QWidget):
    """A bounded QPainter trace without matplotlib, VTK or GPU dependencies."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 150)
        self.setAccessibleName("Tuning response chart")
        self._series = []
        self.unit = "degrees"
        self.empty_message = "Record or import a run to inspect the response"

    def set_series(self, series, unit="degrees"):
        # Keep no more than about one point per horizontal pixel per trace.
        maximum = max(300, min(1600, self.width() * 2))
        self._series = []
        for name, times, values, color in series:
            length = min(len(times), len(values))
            stride = max(1, math.ceil(length / maximum))
            points = [(float(times[i]), float(values[i])) for i in range(0, length, stride)
                      if _finite(times[i]) and _finite(values[i])]
            if length and (length - 1) % stride and _finite(times[length - 1]) and _finite(values[length - 1]):
                points.append((float(times[length - 1]), float(values[length - 1])))
            self._series.append((name, points, color))
        self.unit = unit
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#091522"))
        box = QRectF(66, 35, max(1, self.width() - 85), max(1, self.height() - 78))
        all_points = [point for _, points, _ in self._series for point in points]
        if not all_points:
            painter.setPen(QColor("#91a4ba"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                             self.empty_message)
            return
        xmin, xmax = min(p[0] for p in all_points), max(p[0] for p in all_points)
        ymin, ymax = min(p[1] for p in all_points), max(p[1] for p in all_points)
        xmax = max(xmax, xmin + .01)
        pad = max((ymax - ymin) * .10, 1.)
        ymin, ymax = ymin - pad, ymax + pad
        divisions = max(1, min(4, int(box.height() / 23)))
        for i in range(divisions + 1):
            y = box.bottom() - box.height() * i / divisions
            painter.setPen(QColor("#223348"))
            painter.drawLine(QPointF(box.left(), y), QPointF(box.right(), y))
            painter.setPen(QColor("#91a4ba"))
            painter.drawText(QRectF(2, y - 10, 57, 20), Qt.AlignmentFlag.AlignRight,
                             f"{ymin + (ymax - ymin) * i / divisions:.1f}")
        painter.drawText(QRectF(6, 6, 200, 20), self.unit)
        painter.drawText(QRectF(box.left(), box.bottom() + 9, box.width(), 22),
                         Qt.AlignmentFlag.AlignCenter, f"{xmin:.2f} to {xmax:.2f} seconds")
        painter.save()
        painter.setClipRect(box)
        for name, points, color in self._series:
            path = QPainterPath()
            for i, (t, value) in enumerate(points):
                point = QPointF(box.left() + (t - xmin) / (xmax - xmin) * box.width(),
                                box.bottom() - (value - ymin) / (ymax - ymin) * box.height())
                path.moveTo(point) if i == 0 else path.lineTo(point)
            painter.setPen(QPen(QColor(color), 1.8))
            painter.drawPath(path)
        painter.restore()
        x = 170
        for name, _, color in self._series:
            painter.setPen(QColor(color))
            painter.drawText(QRectF(x, 6, 170, 20), name)
            x += 165


class _Job(QThread):
    result = Signal(object)
    error = Signal(str)
    update = Signal(object)

    def __init__(self, work, parent=None):
        super().__init__(parent)
        self.cancel = threading.Event()
        self.work = work
        self.outcome = "running"

    def run(self):
        try:
            result = self.work(self.cancel, self.update.emit)
            self.outcome = "cancelled" if self.cancel.is_set() else "succeeded"
            self.result.emit(result)
        except Exception as exc:
            if self.cancel.is_set() or isinstance(exc, InterruptedError):
                self.outcome = "cancelled"
            else:
                self.outcome = "failed"
                self.error.emit(str(exc))


class GroupDialog(QDialog):
    """Explicit topology and signs when static C++ discovery is ambiguous."""
    def __init__(self, target=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Motor group mapping")
        self.setStyleSheet(STYLESHEET + "QDialog { background: #101e2d; }")
        self.resize(600, 500)
        self.original = target or {}
        self.target = None
        self.errors = ErrorReporter(self, log=getattr(getattr(parent, "console", None), "append", lambda message: None))
        layout = QVBoxLayout(self)
        form = HelpFormLayout()
        self.name = QLineEdit(self.original.get("name", "My mechanism"))
        self.kind = QComboBox()
        for label, kind in (("Single joint", "motor"), ("Shared motor group", "motor_group"),
                            ("Tank drivetrain", "tank"), ("H-drive", "h_drive"), ("X-drive", "x_drive")):
            self.kind.addItem(label, kind)
        index = self.kind.findData(self.original.get("kind", "motor"))
        self.kind.setCurrentIndex(max(0, index))
        self.ports = QLineEdit(", ".join(str(port) for port in self.original.get("ports", [1])))
        self.ports.setPlaceholderText("Signed PROS ports, for example: 1, -2, 3, -4")
        form.addRow("Name", self.name, "A display name for this target. Naming a group does not change your PROS source code.")
        form.addRow("Topology", self.kind, "Choose a joint, shared mechanism, Tank, H-drive or X-drive. Drivetrain side/corner assignments generate forward, turn and strafe axes automatically.")
        form.addRow("Ports", self.ports, "Signed PROS motor ports from 1 to 21. A negative number reverses that motor's logical direction. Each physical port appears once; drivetrain assignments fill this list for you.")
        layout.addLayout(form)
        self.role_panel = QWidget()
        role_form = HelpFormLayout(self.role_panel)
        role_form.setContentsMargins(0, 0, 0, 0)
        self.role_fields = {}
        self.role_labels = {}
        for name in dict.fromkeys(name for roles in ROLE_AXES.values() for name in roles):
            label, field = name + " motor ports", QLineEdit()
            field.setPlaceholderText("For example: 1, -2, 3")
            field.textEdited.connect(self._roles_edited)
            role_form.addRow(label, field, "List every motor in this side or corner, separated by commas. Use negative ports for reversed motors, for example 1, -2. All motors here share the same axis coefficients.")
            self.role_fields[name], self.role_labels[name] = field, role_form.labelForField(field)
        layout.addWidget(self.role_panel)
        self.hint = QLabel()
        self.hint.setWordWrap(True)
        layout.addWidget(self.hint)
        self.axes = QPlainTextEdit()
        self.axes.setPlaceholderText('{"position": [1.0]}')
        self.advanced = QCheckBox("Advanced: edit axis coefficients directly")
        self.advanced.toggled.connect(self.axes.setVisible)
        self.advanced.toggled.connect(self._advanced_changed)
        advanced_help = "Optional custom coefficients, one number per listed port in the same order. Values range from -1 to 1 and each axis must include a nonzero coefficient. Normal grouped Tank, H and X drives use the role fields above."
        apply_help(self.advanced, "Advanced axis coefficients", advanced_help)
        advanced_row = QHBoxLayout()
        advanced_row.addWidget(self.advanced, 1)
        advanced_row.addWidget(help_label("Coefficients", advanced_help, self))
        layout.addLayout(advanced_row)
        apply_help(self.axes, "Custom axis coefficients", advanced_help)
        layout.addWidget(self.axes)
        self.axes.hide()
        default = QPushButton("Generate weights from motor assignments")
        default.clicked.connect(lambda: self._defaults(show_message=True))
        apply_help(default, "Generate axis weights", "Rebuild the axis coefficients from the selected topology and motor assignments. This replaces any advanced coefficient edits.")
        layout.addWidget(default)
        self.error = QLabel()
        self.error.setWordWrap(True)
        self.error.setStyleSheet("color: #f0a4a4;")
        layout.addWidget(self.error)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self._save)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self.kind.currentIndexChanged.connect(self._hint)
        self.kind.currentIndexChanged.connect(self._configure_roles)
        self._hint()
        self._configure_roles()
        if self.original.get("axes"):
            self.axes.setPlainText(json.dumps(self.original["axes"], indent=2))
        else:
            self._defaults()

    def _configure_roles(self, *args):
        kind = self.kind.currentData()
        roles = ROLE_AXES.get(kind, {})
        self.role_panel.setVisible(bool(roles))
        for name, field in self.role_fields.items():
            field.setVisible(name in roles)
            self.role_labels[name].setVisible(name in roles)
        self.ports.setReadOnly(bool(roles) and not self.advanced.isChecked())
        if not roles:
            existing = self.original.get("axes", {})
            if self.original.get("kind") == kind and existing and (
                    len(existing) != 1 or any(float(weight) != 1 for weights in existing.values() for weight in weights)):
                self.advanced.setChecked(True)
            return
        try:
            ports = self._parsed_ports()
        except ValueError:
            ports = []
        assigned = {name: [] for name in roles}
        original_axes = self.original.get("axes", {}) if self.original.get("kind") == kind else {}
        if original_axes and all(len(values) == len(ports) for values in original_axes.values()):
            for index, port in enumerate(ports):
                matches = [name for name, weights in roles.items()
                           if all(axis in original_axes and abs(float(original_axes[axis][index]) - weight) < 1e-9
                                  for axis, weight in weights.items())]
                if len(matches) == 1:
                    assigned[matches[0]].append(port)
            if sum(map(len, assigned.values())) != len(ports):
                self.advanced.setChecked(True)
        elif len(ports) <= len(roles):
            for role, port in zip(roles, ports):
                assigned[role].append(port)
        for name, values in assigned.items():
            self.role_fields[name].setText(", ".join(map(str, values)))

    def _advanced_changed(self, enabled):
        self.ports.setReadOnly(self.kind.currentData() in ROLE_AXES and not enabled)
        self.role_panel.setEnabled(not enabled)

    def _role_mapping(self):
        roles = ROLE_AXES[self.kind.currentData()]
        ports, axes = [], {axis: [] for axis in next(iter(roles.values()))}
        for role, weights in roles.items():
            text = self.role_fields[role].text()
            values = [int(item.strip()) for item in text.split(",") if item.strip()]
            if not values:
                raise ValueError(f"Assign at least one motor port to {role.lower()}.")
            ports.extend(values)
            for axis, weight in weights.items():
                axes[axis].extend([weight] * len(values))
        return ports, axes

    def _roles_edited(self, *args):
        if self.advanced.isChecked() or self.kind.currentData() not in ROLE_AXES:
            return
        try:
            ports, axes = self._role_mapping()
            self.ports.setText(", ".join(map(str, ports)))
            self.axes.setPlainText(json.dumps(axes, indent=2))
            self.error.clear()
        except ValueError:
            pass

    def _hint(self):
        self.hint.setText({
            "motor": "One joint. Negative ports reverse a motor. Confirm its direction before recording.",
            "motor_group": "All listed motors drive one mechanism in the same logical direction.",
            "tank": "Assign each side's motors below. Use a negative port to reverse its logical direction.",
            "h_drive": "Assign left, right and lateral motor groups. Each field accepts multiple signed ports.",
            "x_drive": "Assign the motors at each corner. Each field accepts multiple signed ports; check roller direction.",
        }[self.kind.currentData()])

    def _parsed_ports(self):
        ports = [int(item.strip()) for item in self.ports.text().split(",") if item.strip()]
        if not ports or any(not 1 <= abs(port) <= 21 for port in ports):
            raise ValueError("Use signed motor ports from 1 to 21.")
        if len({abs(port) for port in ports}) != len(ports):
            raise ValueError("Each physical motor port may appear only once.")
        return ports

    def _defaults(self, show_message=False):
        try:
            kind = self.kind.currentData()
            if kind in ROLE_AXES:
                ports, axes = self._role_mapping()
                self.ports.setText(", ".join(map(str, ports)))
            else:
                ports, axes = self._parsed_ports(), None
            from .tuning_discovery import make_target
            target = make_target(self.name.text(), kind, ports, axes=axes)
            self.axes.setPlainText(json.dumps(target["axes"], indent=2))
            self.error.clear()
        except (ValueError, TypeError) as exc:
            self.error.setText(str(exc))
            if show_message:
                self.errors.warning("Check motor assignments", str(exc), "Fill in the motor ports for each required role, then generate the weights again.")

    def _save(self):
        try:
            if not self.advanced.isChecked():
                kind = self.kind.currentData()
                if kind in ROLE_AXES:
                    ports, axes = self._role_mapping()
                    self.ports.setText(", ".join(map(str, ports)))
                else:
                    ports, axes = self._parsed_ports(), {"shaft": [1] * len(self._parsed_ports())}
            else:
                ports = self._parsed_ports()
                axes = json.loads(self.axes.toPlainText())
            if not isinstance(axes, dict) or not axes:
                raise ValueError("Provide at least one named axis.")
            for name, weights in axes.items():
                if not str(name).strip() or not isinstance(weights, list) or len(weights) != len(ports):
                    raise ValueError("Each axis needs one weight for every port.")
                if not all(_finite(weight) and abs(float(weight)) <= 1 for weight in weights) or not any(float(w) for w in weights):
                    raise ValueError("Weights must be finite, between -1 and 1, with at least one nonzero value.")
            from .tuning_discovery import make_target
            self.target = make_target(self.name.text().strip() or "Manual group", self.kind.currentData(),
                                      ports, axes=axes, source="manual", verified=False)
            self.target["id"] = self.original.get("id", self.target["id"])
            self.target["notes"] = ["User-provided mapping; check motor direction and mechanism geometry."]
            self.accept()
        except (ValueError, TypeError) as exc:
            self.error.setText(str(exc))
            self.errors.warning("Check motor mapping", str(exc), "Correct the highlighted mapping fields and choose Save again. Each physical motor port can belong to only one role.")

    def done(self, result):
        self.errors.dismiss()
        super().done(result)


class TuningWindow(QMainWindow):
    shutdown_finished = Signal()

    def __init__(self, workspace: str, config: dict, engine=None, parent=None):
        super().__init__(parent)
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.setWindowTitle("EERSim · PID tuning lab")
        available = self.screen().availableGeometry()
        self.resize(min(1120, available.width()), min(820, max(1, available.height() - 60)))
        self.setStyleSheet(STYLESHEET)
        self.workspace = str(workspace or "")
        self.config = copy.deepcopy(config)
        self.engine = engine
        self.targets = []
        self.manual_targets = []
        self._settings = QSettings("EERSim", "PID Tuning")
        self._settings_key = "targets/" + hashlib.sha256(self.workspace.encode("utf-8")).hexdigest()
        try:
            saved = json.loads(str(self._settings.value(self._settings_key, "[]")))
            if isinstance(saved, list):
                self.manual_targets = [target for target in saved if isinstance(target, dict) and
                                       all(key in target for key in ("id", "name", "ports", "axes"))]
        except (ValueError, TypeError):
            pass
        self.samples = deque(maxlen=MAX_SAMPLES)
        self.result = None
        self._jobs = set()
        self._closing = False
        self._capturing = False
        self._experiment = False
        self._reset_pending = False
        self._capture_id = None
        self._plot_dirty = False
        self._prepared_workspace = None
        self._hardware_built = False
        self._hardware_active = False
        self._capture_source = "simulation"
        self._actual_period_ms = None
        self._connections = []
        self._operation_title = "Tuning operation"
        self._operation_failed = False
        self._last_engine_error = None
        self._engine_failure_reported = False
        self.console = FeedbackConsole(self)
        self.console.hide()
        self.hardware_log = self.console.editor
        self.errors = ErrorReporter(self, log=self.console.append)
        self._build_ui()
        if engine is not None:
            for name, callback in (("tuning_sample", self._sample), ("tuning_status", self._engine_status),
                                   ("reset_completed", self._reset_complete), ("failed", self._engine_failed)):
                signal = getattr(engine, name, None)
                if signal is not None:
                    signal.connect(callback)
                    self._connections.append((signal, callback))
        self._plot_timer = QTimer(self)
        self._plot_timer.setInterval(100)
        self._plot_timer.timeout.connect(self._update_plot)
        self._plot_timer.start()
        QTimer.singleShot(0, self.refresh_targets)

    @property
    def busy(self):
        return bool(self._jobs or self._experiment or self._reset_pending)

    def set_workspace(self, workspace, config):
        """Refresh a reused window after its source selection changes."""
        if self.busy or self._capturing:
            self.status.setText("Stop the current capture or experiment before changing workspaces.")
            return False
        workspace = str(workspace or "")
        changed = workspace != self.workspace
        self.workspace = workspace
        self.config = copy.deepcopy(config)
        self._closing = False
        self.workspace_label.setText(workspace or "No PROS workspace selected; robot bindings remain available.")
        if changed:
            self._prepared_workspace = None
            self._hardware_built = False
            self._clear_capture()
            self._settings_key = "targets/" + hashlib.sha256(workspace.encode("utf-8")).hexdigest()
            try:
                saved = json.loads(str(self._settings.value(self._settings_key, "[]")))
                self.manual_targets = [target for target in saved if isinstance(target, dict) and
                                       all(key in target for key in ("id", "name", "ports", "axes"))] if isinstance(saved, list) else []
            except (ValueError, TypeError):
                self.manual_targets = []
        if self.engine is not None and not self._connections:
            for name, callback in (("tuning_sample", self._sample), ("tuning_status", self._engine_status),
                                   ("reset_completed", self._reset_complete), ("failed", self._engine_failed)):
                signal = getattr(self.engine, name, None)
                if signal is not None:
                    signal.connect(callback)
                    self._connections.append((signal, callback))
        self._plot_timer.start()
        self.refresh_targets()
        return True

    def _button(self, label, callback, row=None, primary=False):
        button = QPushButton(label)
        button.clicked.connect(callback)
        if primary:
            button.setObjectName("Primary")
        if row is not None:
            row.addWidget(button)
        return button

    def _spin(self, value, lower, upper, suffix="", decimals=1):
        box = QDoubleSpinBox()
        box.setRange(lower, upper)
        box.setDecimals(decimals)
        box.setValue(value)
        box.setSuffix(suffix)
        return box

    def _build_ui(self):
        root = QWidget()
        root.setObjectName("Root")
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(20, 18, 20, 16)
        title = QLabel("PID TUNING LAB")
        title.setObjectName("Brand")
        layout.addWidget(title)
        subtitle = QLabel("Choose a mechanism · capture its response · compare candidate gains")
        subtitle.setObjectName("Muted")
        layout.addWidget(subtitle)
        self.workspace_label = QLabel(self.workspace or "No PROS workspace selected; robot bindings remain available.")
        self.workspace_label.setWordWrap(True)
        self.workspace_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.workspace_label)
        row = QHBoxLayout()
        self.target_combo = QComboBox()
        self.target_combo.setMinimumWidth(260)
        self.target_combo.currentIndexChanged.connect(self._target_changed)
        self.axis_combo = QComboBox()
        self.axis_combo.currentIndexChanged.connect(self._selection_changed)
        target_help = "Select a joint or motor group discovered in the current PROS workspace or robot mapping. Use Add group or Edit mapping when the source code cannot describe its layout. Discovery does not build or run your code."
        axis_help = "Select shaft movement, forward, turn or strafe. Drivetrain feedback is projected from motor encoders; these are not odometry distance or IMU heading PID values."
        apply_help(self.target_combo, "Tuning target", target_help)
        apply_help(self.axis_combo, "Control axis", axis_help)
        row.addWidget(help_label("Target", target_help, self))
        row.addWidget(self.target_combo, 2)
        row.addWidget(help_label("Axis", axis_help, self))
        row.addWidget(self.axis_combo, 1)
        self.refresh_button = self._button("Refresh workspace", self.refresh_targets, row)
        self.add_button = self._button("Add group", self._add_group, row)
        self.edit_button = self._button("Edit mapping", self._edit_group, row)
        apply_help(self.refresh_button, "Refresh workspace", "Rescan the selected workspace and applied robot mapping for motor groups. This does not compile the workspace.")
        apply_help(self.add_button, "Add motor group", "Create a saved manual joint or drivetrain mapping. Assign multiple motors to each side or corner using signed PROS ports.")
        apply_help(self.edit_button, "Edit target mapping", "Adjust this target's name, ports, reversal signs or drivetrain roles. The override is saved for this workspace without editing its source code.")
        layout.addLayout(row)
        self.mapping = QLabel()
        self.mapping.setWordWrap(True)
        layout.addWidget(self.mapping)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs, 1)
        self._setup_tab()
        self._capture_tab()
        self._results_tab()
        self.progress = QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        layout.addWidget(self.progress)
        footer = QHBoxLayout()
        self.status = QLabel("Ready")
        self.status.setWordWrap(True)
        footer.addWidget(self.status, 1)
        self.console_button = self._button("Console", self._toggle_console, footer)
        apply_help(self.console_button, "Diagnostic console", "Show or hide the bounded operation log. Drag the console's title bar to detach it into a separate window.")
        self.cancel_button = self._button("Stop / cancel", self.cancel_work, footer)
        self.cancel_button.setObjectName("Danger")
        apply_help(self.cancel_button, "Stop capture or experiment", "Stop the current simulated tuning experiment or cancel its background task. For a physical robot this ends logging only; the robot keeps following its program and controls. Scene reset finishes its current rebuild.")
        layout.addLayout(footer)
        self.menuBar().addMenu("View").addAction(self.console.toggleViewAction())
        self._sync_controls()

    def _setup_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        note = QLabel("Tune your outer PID using the same units, loop period and output interface as your PROS code. "
                      "VEX cartridge limits are modeled; candidate gains require testing on the physical mechanism.")
        note.setWordWrap(True)
        layout.addWidget(note)
        form = HelpFormLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("Position (motor degrees)", "position")
        self.mode_combo.addItem("Velocity (motor RPM)", "velocity")
        self.mode_combo.currentIndexChanged.connect(self._mode_changed)
        self.voltage = self._spin(3000, 100, 12000, " mV", 0)
        self.duration = self._spin(2, .2, 30, " s", 2)
        self.travel = self._spin(180, 1, 100000, " motor deg", 1)
        self.period = self._spin(20, 1, 1000, " ms", 3)
        self.period.setToolTip("Simulation aligns this interval with physics steps. The exact actual interval is retained in candidate exports.")
        self.step_target = self._spin(90, -10000, 10000, "", 2)
        self.large_error = self._spin(100000, .01, 1000000, "", 2)
        self.antiwind = self._spin(1000, 0, 1000000, " error·s", 2)
        for widget in (self.voltage, self.duration, self.travel, self.step_target, self.large_error, self.antiwind):
            widget.valueChanged.connect(self._selection_changed)
        self.period.valueChanged.connect(self._period_changed)
        for label, widget, explanation in (
                ("Controller", self.mode_combo, "Position controls relative motor-shaft degrees; velocity controls motor RPM. Use the same feedback units and PID formula in your robot code. Changing the controller invalidates the current candidate."),
                ("Characterization / output limit", self.voltage, "Voltage in millivolts for the characterization pulse and the candidate PID output limit. 3000 mV is 3 V; the maximum is 12000 mV. A pulse too weak to move the mechanism cannot identify its response."),
                ("Powered interval", self.duration, "Duration of the powered pulse or validation interval in simulation seconds. A short baseline and coast interval are captured too. Wall-clock rendering speed does not change this duration."),
                ("Maximum travel from start", self.travel, "Stop the experiment if any active motor exceeds this displacement from its initial encoder position. Units are motor-shaft degrees, before any external gearing. Choose a bound within the mechanism's usable travel."),
                ("Controller period", self.period, "Requested milliseconds between PID updates. Simulation aligns the interval with physics steps; the exact actual interval is retained in candidate exports. Match this interval in your robot code."),
                ("Validation step (deg or RPM)", self.step_target, "A nonzero signed target. Position mode uses motor degrees relative to the initial position and must stay inside the travel bound. Velocity mode uses motor RPM."),
                ("EERKit integral activation error", self.large_error, "Accumulate integral only while absolute error is below this threshold and nonzero, matching EERKit PID behavior. Outside that range the accumulated error resets. Units match the selected controller's error."),
                ("EERKit integral limit", self.antiwind, "Limit the accumulated-error contribution before multiplying by Ki, in error units times seconds. The accumulator itself is not clipped, matching EERKit's controller. Larger values can increase windup.")):
            form.addRow(label, widget, explanation)
        layout.addLayout(form)
        actions = QHBoxLayout()
        self.characterize_button = self._button("Characterize in simulation", self._characterize, actions, True)
        self.reset_button = self._button("Reset simulation", self._reset, actions)
        apply_help(self.characterize_button, "Characterize mechanism", "Stop simulated user code and apply a bounded voltage pulse through the selected motor bindings. This begins a fresh capture without rebuilding the PROS workspace.")
        apply_help(self.reset_button, "Reset simulation", "Restore the applied robot and field starting state before repeating a trial. The current candidate gains are preserved so you can reset and then validate them.")
        layout.addLayout(actions)
        explanation = QLabel("Characterization captures a short voltage pulse and its return to coast. It stops at the travel bound. "
                             "Simulation reset rebuilds the applied starting scene. Return a physical robot to its start by hand before a repeat test.")
        explanation.setWordWrap(True)
        explanation.setObjectName("Muted")
        layout.addWidget(explanation)
        self.discovery_notes = QPlainTextEdit()
        self.discovery_notes.setReadOnly(True)
        self.discovery_notes.setMaximumHeight(130)
        self.discovery_notes.hide()
        layout.addWidget(self.discovery_notes)
        layout.addStretch()
        self._add_scrollable_tab(page, "Setup")

    def _add_scrollable_tab(self, page, title):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        scroll.setAccessibleName(title + " settings")
        self.tabs.addTab(scroll, title)

    def _capture_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        row = QHBoxLayout()
        self.record_button = self._button("Record live simulation", self._record, row, True)
        self.import_button = self._button("Import capture…", self._import, row)
        self.save_capture_button = self._button("Export capture…", self._save_capture, row)
        self.clear_button = self._button("Clear capture", self._clear_capture, row)
        for button, title, explanation in (
                (self.record_button, "Record live simulation", "Record the selected motors while you run the existing program in the main simulator. This observes feedback; it does not build or start code."),
                (self.import_button, "Import a capture", "Open a single continuous JSON, JSONL, CSV or logger trial. Select the matching signed motor ports and axis after import."),
                (self.save_capture_button, "Export recorded trial", "Save the current motor samples as JSONL or CSV, including source, time, capture identity and available telemetry."),
                (self.clear_button, "Clear current capture", "Discard this window's current samples and candidate. Export them first if you want to keep the trial.")):
            apply_help(button, title, explanation)
        layout.addLayout(row)
        self.capture_label = QLabel("0 samples · one trial per capture")
        layout.addWidget(self.capture_label)
        self.capture_plot = ResponsePlot()
        layout.addWidget(self.capture_plot, 2)
        self.voltage_plot = ResponsePlot()
        self.voltage_plot.setMinimumHeight(125)
        self.voltage_plot.setMaximumHeight(170)
        self.voltage_plot.empty_message = "Applied / measured voltage will appear here"
        layout.addWidget(self.voltage_plot, 1)
        hardware = QLabel("PHYSICAL ROBOT")
        hardware.setObjectName("Section")
        layout.addWidget(hardware)
        note = QLabel("Prepare creates a separate PROS workspace with passive telemetry logging. Build and upload it explicitly, "
                      "then run your normal robot callbacks while recording. Existing firmware cannot gain logging without an upload. "
                      "Stop ends logging; the physical robot continues following its program and controls.")
        note.setWordWrap(True)
        layout.addWidget(note)
        row = QHBoxLayout()
        self.prepare_button = self._button("1 · Prepare logger copy…", self._hardware_prepare, row)
        self.build_button = self._button("2 · Build", self._hardware_build, row)
        self.slot = QSpinBox()
        self.slot.setRange(1, 8)
        self.slot.setValue(8)
        slot_help = "Choose which V5 Brain program slot to replace when you press Upload. Upload is a separate explicit action and does not start the program automatically."
        apply_help(self.slot, "V5 program slot", slot_help)
        slot_caption = help_label("Program slot", slot_help, self)
        slot_caption.setMaximumWidth(115)
        row.addWidget(slot_caption)
        self.slot.setMaximumWidth(80)
        row.addWidget(self.slot)
        self.upload_button = self._button("3 · Upload", self._hardware_upload, row)
        apply_help(self.prepare_button, "Prepare telemetry workspace", "Choose a parent folder for a new logger workspace copy. The original callbacks and workspace are preserved. Logger sampling is at least 20 ms; the exact interval is shown in the console.")
        apply_help(self.build_button, "Build logger program", "Compile the prepared workspace with an installed PROS CLI and ARM toolchain. If the CLI is not discovered, use the PROS CLI field below to select it.")
        apply_help(self.upload_button, "Upload logger program", "Replace the selected V5 program slot with the successfully built logger program. Connect the Brain and verify the slot before pressing this button.")
        layout.addLayout(row)
        row = QHBoxLayout()
        self.hardware_port = QLineEdit()
        self.hardware_port.setPlaceholderText("Serial port (for example COM4 or /dev/ttyACM1)")
        port_help = "Optional Brain serial port, such as COM4 on Windows or /dev/ttyACM1 on Linux. Leave empty for PROS discovery. Connect/record reads logger output; stopping recording does not stop the physical robot program."
        apply_help(self.hardware_port, "Brain serial port", port_help)
        row.addWidget(help_label("Serial", port_help, self))
        row.addWidget(self.hardware_port, 1)
        self.cli_path = QLineEdit(str(self._settings.value("pros_cli", "")))
        self.cli_path.setPlaceholderText("PROS CLI: automatic discovery")
        cli_help = "Optional path to an installed PROS CLI executable, such as pros.exe. Leave empty for discovery from PATH or the PROS VS Code extension. Hardware build, upload and recording need PROS; the physical robot build also needs its ARM toolchain."
        apply_help(self.cli_path, "PROS CLI executable", cli_help)
        row.addWidget(help_label("PROS CLI", cli_help, self))
        row.addWidget(self.cli_path, 1)
        self.cli_button = self._button("CLI…", self._choose_cli, row)
        self.hardware_record_button = self._button("4 · Connect / record", self._hardware_record, row)
        apply_help(self.cli_button, "Select PROS executable", "Browse to an existing PROS CLI executable. Selecting it saves this preference; no install, build or upload starts automatically.")
        apply_help(self.hardware_record_button, "Record physical robot", "Read complete telemetry scans from the uploaded passive logger while you operate the robot normally. This button does not start motion; Stop / cancel only ends recording.")
        layout.addLayout(row)
        self._add_scrollable_tab(page, "Live capture")

    def _results_tab(self):
        page = QWidget()
        layout = QVBoxLayout(page)
        row = QHBoxLayout()
        self.analyze_button = self._button("Analyze response", self._analyze, row)
        self.tune_button = self._button("Find candidate gains", self._tune, row, True)
        self.validate_button = self._button("Validate in simulation", self._validate, row)
        self.export_button = self._button("Export gains / report…", self._export, row)
        for button, title, explanation in (
                (self.analyze_button, "Analyze recorded response", "Inspect the recorded motor response without calculating gains. Missing voltage is acceptable for inspection but prevents model identification."),
                (self.tune_button, "Find candidate gains", "Fit the selected axis response from a continuous voltage-driven capture, then compare PID candidates across model uncertainty. Keep a usable pulse and its response in the capture."),
                (self.validate_button, "Validate candidate in simulation", "Run the saved candidate and its exact controller settings through the current physics scene. Reset first to repeat the same starting condition; simulation validation does not validate the physical robot."),
                (self.export_button, "Export PID candidate", "Save gains, units, exact controller period, model quality, settings and predictions in a JSON report. Use matching PID behavior and units when transferring gains.")):
            apply_help(button, title, explanation)
        layout.addLayout(row)
        self.gain_label = QLabel("No candidate yet")
        self.gain_label.setObjectName("Title")
        self.gain_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.gain_label)
        self.result_plot = ResponsePlot()
        layout.addWidget(self.result_plot, 1)
        self.report = QPlainTextEdit()
        self.report.setReadOnly(True)
        layout.addWidget(self.report, 1)
        note = QLabel("Candidate gains use motor degrees or RPM and millivolt output; time is measured in seconds. "
                      "Copy gains only into a controller with matching units, loop period and integral behavior. "
                      "Fitted-response predictions are separate from full-scene validation.")
        note.setWordWrap(True)
        note.setObjectName("Muted")
        layout.addWidget(note)
        self._add_scrollable_tab(page, "Tune / results")

    def _target(self):
        return self.target_combo.currentData()

    def _options(self):
        return {"mode": self.mode_combo.currentData(), "controller_period_ms": self._actual_period_ms if self._actual_period_ms is not None else self.period.value(),
                "step_target": self.step_target.value(), "voltage_mv": self.voltage.value(),
                "output_limit_mv": self.voltage.value(), "duration_s": self.duration.value(),
                "max_travel_deg": self.travel.value(), "large_error": self.large_error.value(),
                "antiwind": self.antiwind.value()}

    def _sync_controls(self, *args):
        if not hasattr(self, "cancel_button"):
            return
        busy = bool(self._jobs or self._experiment or self._reset_pending or self._capturing or self._closing)
        target = bool(self._target())
        for widget in (self.target_combo, self.axis_combo, self.refresh_button, self.add_button, self.edit_button,
                       self.mode_combo, self.voltage, self.duration, self.travel, self.period, self.step_target,
                       self.large_error, self.antiwind, self.import_button, self.clear_button):
            widget.setEnabled(not busy)
        self.characterize_button.setEnabled(not busy and target and self.engine is not None)
        self.validate_button.setEnabled(not busy and target and self.engine is not None and self.result is not None)
        self.reset_button.setEnabled(not busy and self.engine is not None)
        self.record_button.setEnabled(not busy and target and self.engine is not None)
        self.analyze_button.setEnabled(not busy and target and len(self.samples) >= 3)
        self.tune_button.setEnabled(not busy and target and len(self.samples) >= 3)
        self.save_capture_button.setEnabled(not busy and bool(self.samples))
        self.export_button.setEnabled(not busy and self.result is not None)
        self.prepare_button.setEnabled(not busy and target and bool(self.workspace))
        self.build_button.setEnabled(not busy and bool(self._prepared_workspace))
        self.upload_button.setEnabled(not busy and bool(self._prepared_workspace) and self._hardware_built)
        self.hardware_record_button.setEnabled(not busy and target and bool(self._prepared_workspace or self.workspace))
        self.slot.setEnabled(not busy)
        self.hardware_port.setEnabled(not busy)
        self.cli_path.setEnabled(not busy)
        self.cli_button.setEnabled(not busy)
        self.cancel_button.setEnabled(bool(self._jobs or self._capturing or self._experiment) and not self._closing)

    def _launch(self, title, work, done=None):
        if self._jobs or self._closing:
            return
        job = _Job(work, self)
        self._jobs.add(job)
        self._begin_operation(title)
        job.update.connect(self._job_update)
        job.error.connect(self._error)
        job.result.connect(lambda result: self._job_result(job, result, done))
        job.finished.connect(lambda: self._job_finished(job))
        self._sync_controls()
        job.start()

    def _begin_operation(self, title):
        self._operation_title = str(title)
        self._operation_failed = False
        self._last_engine_error = None
        self._engine_failure_reported = False
        self.status.setText(self._operation_title)
        self.console.append(self._operation_title)
        self.progress.setRange(0, 0)

    def _finish_progress(self, success):
        self.progress.setRange(0, 100)
        self.progress.setValue(100 if success else 0)

    def _job_result(self, job, result, done):
        if self._closing or job.cancel.is_set() or job.outcome == "cancelled":
            return
        if done is not None:
            try:
                done(result)
            except Exception as exc:
                job.outcome = "failed"
                self._error(str(exc))

    def _job_finished(self, job):
        self._jobs.discard(job)
        hardware_recording = self._hardware_active
        if self._hardware_active:
            self._hardware_active = False
            self._capturing = False
        job.deleteLater()
        cancelled = job.outcome == "cancelled" or job.cancel.is_set()
        success = job.outcome == "succeeded" and not cancelled and not self._operation_failed
        self._finish_progress(success)
        if cancelled:
            message = ("Recording stopped. The physical robot continues following its program."
                       if hardware_recording else "Operation cancelled.")
            self.status.setText(message)
            self.console.append(message)
        elif success:
            self.console.append("Completed: " + self._operation_title)
        self._sync_controls()
        if self._closing and not self._jobs:
            QTimer.singleShot(0, self.close)

    @Slot(object)
    def _job_update(self, update):
        if self._closing:
            return
        if isinstance(update, dict) and "sample" in update:
            self._sample(update["sample"])
        elif isinstance(update, dict):
            if "message" in update:
                self.status.setText(str(update["message"]))
                self.console.append(str(update["message"]))
            if "progress" in update or "fraction" in update:
                raw = update.get("fraction", update.get("progress", 0))
                if _finite(raw):
                    self.progress.setRange(0, 100)
                    value = float(raw)
                    self.progress.setValue(max(0, min(99, round(value * 100 if value <= 1 else value))))
        else:
            self.console.append(str(update))

    @Slot(str)
    def _error(self, message):
        self._operation_failed = True
        self._finish_progress(False)
        if not self._closing:
            title, explanation, details = _error_feedback(message, self._operation_title)
            self.status.setText(title + ": " + (str(message).splitlines() or ["No additional details"])[0][:220])
            self.errors.error(title, explanation, details)

    def _toggle_console(self):
        visible = not self.console.isVisible()
        self.console.setVisible(visible)
        if visible:
            self.console.raise_()

    def refresh_targets(self):
        def work(cancel, progress):
            from .tuning_discovery import discover_targets
            return discover_targets(self.workspace, self.config)
        self._launch("Discovering configured joints and PROS motor groups…", work, self._discovered)

    def _discovered(self, result):
        self.targets = result.get("targets", [])
        warnings = [str(warning) for warning in result.get("warnings", [])]
        self.discovery_notes.setPlainText("\n".join(warnings))
        self.discovery_notes.setVisible(bool(warnings))
        for warning in warnings:
            self.console.append("Discovery: " + warning)
        self._populate_targets()
        self.status.setText(f"Found {len(self.targets)} targets. Select a mechanism and axis to begin.")

    def _populate_targets(self, selected=None):
        selected = selected or (self._target() or {}).get("id")
        self.target_combo.blockSignals(True)
        self.target_combo.clear()
        combined = {target["id"]: target for target in self.targets}
        combined.update({target["id"]: target for target in self.manual_targets})
        for target in combined.values():
            source = str(target.get("source", "binding"))
            label = "manual" if source == "manual" else ("robot" if source.startswith(("binding", "robot", "config", "simulation")) else "code")
            self.target_combo.addItem(f"{target['name']}  ·  {label}", target)
        for i in range(self.target_combo.count()):
            if self.target_combo.itemData(i).get("id") == selected:
                self.target_combo.setCurrentIndex(i)
                break
        self.target_combo.blockSignals(False)
        self._target_changed()

    def _target_changed(self, *args):
        target = self._target() or {}
        self.axis_combo.blockSignals(True)
        self.axis_combo.clear()
        self.axis_combo.addItems(list(target.get("axes", {})))
        self.axis_combo.blockSignals(False)
        notes = target.get("notes", [])
        if isinstance(notes, str):
            notes = [notes]
        self.mapping.setText(f"Ports: {target.get('ports', [])} · {target.get('kind', 'No target')} · "
                             f"{target.get('source', '')}\n" + " ".join(str(note) for note in notes))
        self._selection_changed()

    def _selection_changed(self, *args):
        self._plot_dirty = True
        self.result = None
        if hasattr(self, "gain_label"):
            self.gain_label.setText("No candidate for this selection")
            self.report.clear()
            self.result_plot.set_series([])
        self._sync_controls()

    def _mode_changed(self, *args):
        if hasattr(self, "step_target"):
            if self.mode_combo.currentData() == "velocity" and self.step_target.value() == 90:
                self.step_target.setValue(60)
            elif self.mode_combo.currentData() == "position" and self.step_target.value() == 60:
                self.step_target.setValue(90)
        self._selection_changed()

    def _period_changed(self, *args):
        self._actual_period_ms = None
        self._selection_changed()

    def _add_group(self):
        dialog = GroupDialog(parent=self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self.manual_targets.append(dialog.target)
            self._settings.setValue(self._settings_key, json.dumps(self.manual_targets))
            self._populate_targets(dialog.target["id"])

    def _edit_group(self):
        if not self._target():
            return
        dialog = GroupDialog(copy.deepcopy(self._target()), self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self.manual_targets = [target for target in self.manual_targets if target["id"] != dialog.target["id"]]
            self.manual_targets.append(dialog.target)
            self._settings.setValue(self._settings_key, json.dumps(self.manual_targets))
            self._populate_targets(dialog.target["id"])

    def _clear_capture(self, preserve_result=False):
        self.samples.clear()
        self._capture_id = None
        self._plot_dirty = True
        self.capture_label.setText("0 samples · one trial per capture")
        if not preserve_result:
            self._selection_changed()
        self._sync_controls()

    @Slot(dict)
    def _sample(self, sample):
        if self._closing or not (self._capturing or self._experiment):
            return
        if not isinstance(sample, dict) or not _finite(sample.get("time")):
            return
        if sample.get("source", "simulation") != self._capture_source:
            return
        capture_id = sample.get("capture_id")
        # A reset/new producer cannot silently concatenate time histories.
        if self.samples and (float(sample["time"]) < float(self.samples[-1]["time"]) or
                             (capture_id is not None and self._capture_id not in (None, capture_id))):
            self.samples.clear()
        self._capture_id = capture_id
        self.samples.append(copy.deepcopy(sample))
        self._capture_source = sample.get("source", self._capture_source)
        self._plot_dirty = True

    def _update_plot(self):
        if not self._plot_dirty:
            return
        self._plot_dirty = False
        rows = list(self.samples)
        stride = max(1, math.ceil(len(rows) / 1600))
        display = rows[::stride]
        if rows and display[-1] is not rows[-1]:
            display.append(rows[-1])
        target, axis = self._target() or {}, self.axis_combo.currentText()
        mode = self.mode_combo.currentData()
        times, values = project_samples(display, target, axis, mode)
        series = [("Recorded response", times, values, GOLD)]
        origin = float(rows[0]["time"]) if rows else 0.
        setpoints = [(float(row["time"]) - origin, float(row["target"])) for row in display
                     if row.get("control_mode") == mode and _finite(row.get("target"))]
        if setpoints:
            series.append(("Controller target", [item[0] for item in setpoints], [item[1] for item in setpoints], "#80d7c0"))
        self.capture_plot.set_series(series, "motor degrees" if mode == "position" else "motor RPM")
        voltage_times, voltages = project_samples(display, target, axis, "voltage")
        self.voltage_plot.empty_message = ("Voltage unavailable; record raw voltage commands or a physical-robot logger trial"
                                           if rows else "Applied / measured voltage will appear here")
        self.voltage_plot.set_series([("Applied / measured voltage", voltage_times, voltages, "#80d7c0")], "millivolts")
        self.capture_label.setText(f"{len(self.samples):,} samples · {self._capture_source} · "
                                  f"{times[-1] if times else 0:.2f} s" +
                                  (" · buffer full; oldest samples discarded" if len(self.samples) == MAX_SAMPLES else ""))

    def _record(self):
        self._clear_capture()
        self._capture_source = "simulation"
        self._capturing = True
        self._begin_operation("Recording simulation feedback. Run your code in the main window, then Stop / cancel here.")
        self.engine.tuning_capture(True, copy.deepcopy(self._target()))
        self._sync_controls()

    def _experiment_request(self, kind):
        request = copy.deepcopy(self.result["options"]) if kind == "validate" else self._options()
        request.setdefault("voltage_mv", request.get("output_limit_mv", self.voltage.value()))
        request.update({"target": copy.deepcopy(self._target()), "axis": self.axis_combo.currentText(), "kind": kind})
        if kind == "validate":
            request["gains"] = copy.deepcopy(self.result["gains"])
        self._clear_capture(preserve_result=kind == "validate")
        self._capture_source = "simulation"
        self._experiment = True
        self._begin_operation("Starting " + kind + " experiment…")
        self.tabs.setCurrentIndex(1)
        self._sync_controls()
        self.engine.tuning_experiment(request)

    def _characterize(self):
        self._experiment_request("characterize")

    def _validate(self):
        self._experiment_request("validate")

    @Slot(dict)
    def _engine_status(self, status):
        if self._closing or self._hardware_active:
            return
        state = status.get("state")
        engine_failure = bool(status.get("engine_failure"))
        # Reset also stops an old observer; that acknowledgement is not reset
        # completion and must not replace the active scene-rebuild feedback.
        if self._reset_pending and state in ("cancelled", "idle") and not engine_failure:
            self.console.append(str(status.get("message", "Previous tuning observer stopped.")))
            return
        if _finite(status.get("controller_period_ms")):
            self._actual_period_ms = float(status["controller_period_ms"])
            previous = self.period.blockSignals(True)
            self.period.setValue(self._actual_period_ms)
            self.period.blockSignals(previous)
        if state in ("completed", "cancelled", "error", "idle"):
            self._experiment = False
            self._capturing = False
            self._finish_progress(state == "completed")
        message = str(status.get("message", state or "Ready"))
        if state == "error" and engine_failure:
            self._engine_failure_reported = True
            self._last_engine_error = message
            if getattr(self.parent(), "feedback", None) is not None:
                self._operation_failed = True
                self.status.setText(message)
                self.console.append(message)
            else:
                self._error(message)
        elif state == "error" and message != self._last_engine_error:
            self._error(message)
        else:
            self.status.setText(message)
            self.console.append(message)
        self._sync_controls()

    def _reset(self):
        self._reset_pending = True
        self._begin_operation("Resetting simulation scene…")
        self._sync_controls()
        self.engine.reset()

    @Slot(dict)
    def _reset_complete(self, state):
        if self._reset_pending:
            self._reset_pending = False
            self._clear_capture(preserve_result=True)
            self.status.setText("Simulation restored to its applied starting state.")
            self.console.append("Simulation restored to its applied starting state.")
            self._finish_progress(True)
            self._sync_controls()

    @Slot(str)
    def _engine_failed(self, message):
        if self._hardware_active:
            return
        if getattr(self, "_engine_failure_reported", False):
            self._reset_pending = self._experiment = self._capturing = False
            self._sync_controls()
            return
        if self._reset_pending or self._experiment or self._capturing:
            self._reset_pending = self._experiment = self._capturing = False
            self._last_engine_error = str(message)
            if getattr(self.parent(), "feedback", None) is not None:
                self._operation_failed = True
                self._finish_progress(False)
                self.status.setText("Could not complete: " + (str(message).splitlines() or ["No additional details"])[0][:220])
                self.console.append(str(message))
            else:
                self._error(message)
            self._sync_controls()

    def _analyze(self):
        samples, target, axis, mode = list(self.samples), copy.deepcopy(self._target()), self.axis_combo.currentText(), self.mode_combo.currentData()
        def work(cancel, progress):
            from .tuning import analyze_response
            return analyze_response(samples, target, axis, mode=mode)
        self._launch("Analyzing recorded response…", work, self._show_analysis)

    def _show_analysis(self, result):
        summary = {key: value for key, value in result.items() if key not in ("time", "response", "position_deg", "velocity_rpm", "voltage_mv")}
        self.report.setPlainText(json.dumps(summary, indent=2, default=str))
        self.result_plot.set_series([("Recorded response", result.get("time", []), result.get("response", []), GOLD)], result.get("units", ""))
        self.tabs.setCurrentIndex(2)
        self.status.setText("Response analysis complete.")

    def _tune(self):
        samples, target, axis, options = list(self.samples), copy.deepcopy(self._target()), self.axis_combo.currentText(), self._options()
        def work(cancel, progress):
            from .tuning import autotune
            return autotune(samples, target, axis, options, cancel=cancel, progress=progress)
        self._launch("Searching candidate PID gains…", work, self._show_result)

    def _show_result(self, result):
        self.result = result
        gains = result.get("gains", {})
        self.gain_label.setText("Candidate · " + "   ".join(f"{key.upper()} {float(gains[key]):.5g}" for key in ("kp", "ki", "kd") if key in gains))
        self.report.setPlainText(json.dumps({key: value for key, value in result.items() if key != "validation"}, indent=2, default=str))
        validation = result.get("validation", {})
        times = validation.get("time", [])
        self.result_plot.set_series([
            ("Fitted model", times, validation.get("response", []), GOLD),
            ("Requested step", times, validation.get("target", []), "#80d7c0")],
            "motor degrees" if self.mode_combo.currentData() == "position" else "motor RPM")
        self.tabs.setCurrentIndex(2)
        self.status.setText("Candidate ready. Validate in the complete scene and compare with a physical-robot capture.")
        self._sync_controls()

    def _file_start(self, name):
        return str(Path(self.workspace) / name) if self.workspace else name

    def _import(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import a recorded trial", self._file_start(""), "Capture (*.jsonl *.json *.csv);;All files (*)")
        if not path:
            return
        def work(cancel, progress):
            from .tuning import read_capture
            return read_capture(path)
        self._launch("Reading capture…", work, self._loaded_capture)

    def _loaded_capture(self, samples):
        self._clear_capture()
        self.samples.extend(samples[-MAX_SAMPLES:])
        self._capture_source = self.samples[0].get("source", "imported") if self.samples else "imported"
        self._plot_dirty = True
        self.tabs.setCurrentIndex(1)
        self.status.setText(f"Imported {len(self.samples):,} samples. Select the matching target ports and axis.")
        self._sync_controls()

    def _save_capture(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export recorded trial", self._file_start("tuning-capture.jsonl"), "JSON lines (*.jsonl);;CSV (*.csv)")
        if not path:
            return
        samples = list(self.samples)
        def work(cancel, progress):
            from .tuning import export_capture
            return export_capture(path, samples)
        self._launch("Saving capture…", work, lambda result: self.status.setText("Capture saved: " + path))

    def _export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export gains and report", self._file_start("pid-candidate.json"), "JSON report (*.json)")
        if not path:
            return
        result = copy.deepcopy(self.result)
        def work(cancel, progress):
            from .tuning import export_result
            return export_result(path, result)
        self._launch("Saving candidate report…", work, lambda result: self.status.setText("Candidate report saved: " + path))

    def _hardware_prepare(self):
        parent = QFileDialog.getExistingDirectory(self, "Choose where to create a new logger workspace", str(Path(self.workspace).parent) if self.workspace else "")
        if not parent:
            return
        workspace, ports = self.workspace, list(self._target()["ports"])
        destination = Path(parent) / f"{Path(workspace).name or 'pros'}-pid-logger-{uuid.uuid4().hex[:8]}"
        period = max(20, round(self.period.value()))
        self.console.append(f"Logger copy: {destination}\nMotor ports: {ports}; telemetry interval: {period} ms")
        def work(cancel, progress):
            from .tuning_hardware import prepare_capture
            return prepare_capture(workspace, destination, ports, sample_ms=period, cancel=cancel, log=progress)
        def complete(prepared):
            self._prepared_workspace = prepared
            self._hardware_built = False
            self.console.append("Prepared logger workspace: " + str(prepared.workspace))
            self.status.setText("Logger copy prepared. Build it before uploading to your selected program slot.")
        self._launch("Preparing a separate telemetry workspace…", work, complete)

    def _hardware_build(self):
        prepared = self._prepared_workspace
        cli = self.cli_path.text().strip() or None
        self._hardware_built = False
        def work(cancel, progress):
            from .tuning_hardware import run_build
            return run_build(prepared, cli=cli, cancel=cancel, log=progress)
        def complete(result):
            self._hardware_built = result.returncode == 0 and not result.cancelled and not result.timed_out
            self._hardware_command_complete(result, "Build")
        self._launch("Building the telemetry workspace…", work, complete)

    def _hardware_upload(self):
        prepared, slot, port = self._prepared_workspace, self.slot.value(), self.hardware_port.text().strip() or None
        cli = self.cli_path.text().strip() or None
        def work(cancel, progress):
            from .tuning_hardware import run_upload
            return run_upload(prepared, slot=slot, port=port, cli=cli, cancel=cancel, log=progress)
        self._launch(f"Uploading logger program to V5 slot {slot}…", work,
                     lambda result: self._hardware_command_complete(result, "Upload"))

    def _hardware_record(self):
        prepared = self._prepared_workspace or self.workspace
        port = self.hardware_port.text().strip() or None
        cli = self.cli_path.text().strip() or None
        self._clear_capture()
        self._capture_source = "hardware"
        self._capturing = self._hardware_active = True
        def work(cancel, progress):
            from .tuning_hardware import run_terminal
            return run_terminal(prepared, port=port, cli=cli, cancel=cancel, log=progress,
                                on_sample=lambda sample: progress({"sample": sample}))
        self._launch("Connected logger captures your robot program; use the robot controls to start and stop motion.", work,
                     lambda result: self._hardware_command_complete(result, "Recording"))

    def _hardware_command_complete(self, result, action):
        if result.cancelled:
            message = action + " cancelled."
            self._operation_failed = True
        elif result.timed_out:
            message = action + " timed out."
            self._operation_failed = True
            title, explanation, details = _error_feedback(message, action)
            self.errors.warning(title, explanation, details + "\n\n" + str(result.output_tail))
        elif result.returncode:
            message = f"{action} failed (exit {result.returncode}). Review the terminal output."
            self._operation_failed = True
            title, explanation, details = _error_feedback(message, action)
            self.errors.error(title, explanation, details + "\n\n" + str(result.output_tail))
        else:
            message = action + " complete."
        self.status.setText(message)
        self._finish_progress(not self._operation_failed)
        if result.output_tail:
            self.console.append(str(result.output_tail))

    def _choose_cli(self):
        path, _ = QFileDialog.getOpenFileName(self, "Select the PROS CLI executable", self.cli_path.text(), "All files (*)")
        if path:
            self.cli_path.setText(path)
            self._settings.setValue("pros_cli", path)

    def cancel_work(self):
        for job in tuple(self._jobs):
            job.cancel.set()
        if self.engine is not None and not self._hardware_active and (self._capturing or self._experiment):
            self.engine.tuning_cancel()
            self.engine.tuning_capture(False)
        self._capturing = False
        self._experiment = False
        self.status.setText("Stopping…" if self._jobs else "Capture / experiment stopped.")
        if not self._jobs and not self._reset_pending:
            self._finish_progress(False)
        self._sync_controls()

    def closeEvent(self, event):
        self._closing = True
        self.errors.dismiss()
        self.console.errors.dismiss()
        self.cancel_work()
        if self._jobs:
            self.status.setText("Closing after background work stops…")
            event.ignore()
            return
        self._plot_timer.stop()
        self.console.hide()
        for signal, callback in self._connections:
            try:
                signal.disconnect(callback)
            except (RuntimeError, TypeError):
                pass
        self._connections.clear()
        self.shutdown_finished.emit()
        event.accept()
