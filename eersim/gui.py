"""Desktop autonomy workbench. Physics and native robot code run in the engine.

The GUI never advances physics or builds C++ itself. It sends commands to the
engine, and displays copied camera frames and state delivered through Qt signals.
"""

from __future__ import annotations

import copy
import importlib
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from PySide6.QtCore import QEvent, QPointF, QRectF, QStandardPaths, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QIcon, QImage, QKeyEvent, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDoubleSpinBox, QDialog, QDialogButtonBox,
    QFileDialog, QFrame, QGridLayout, QHBoxLayout, QHeaderView,
    QLabel, QLineEdit, QMainWindow, QPlainTextEdit, QProgressBar,
    QPushButton, QScrollArea, QSizePolicy, QSlider, QSpinBox, QSplitter, QStyle, QStyleOptionSlider,
    QTabWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from .config import ASSETS, default_config
from .engine import SimulationEngine
from .gui_styles import GOLD, MUTED, STYLESHEET
from .gui_feedback import (ErrorReporter, ExceptionFeedback, FeedbackConsole, HelpFormLayout,
                           OperationProgress, apply_help, help_label)
from .motor import CARTRIDGES as PHYSICAL_CARTRIDGES


def _dialog_default(filename, category, previous=None, workspace=None):
    """Choose writable frozen-app defaults without replacing user choices."""
    if previous:
        return str(previous)
    if workspace:
        return str(Path(workspace).expanduser().resolve() / filename)
    if getattr(sys, "frozen", False):
        from .runtime_paths import user_data_dir
        directory = user_data_dir() / category
        directory.mkdir(parents=True, exist_ok=True)
        return str(directory / filename)
    return filename


MODES = (
    ("skills_driver", "Skills Driver", "opcontrol", 60),
    ("skills_auto", "Skills Auto", "autonomous", 60),
    ("match_auto", "Match Auto", "autonomous", 30),
    ("driver_auto", "Driver Auto", "opcontrol", 90),
)
ANALOG_NAMES = ("left_x", "left_y", "right_x", "right_y")
DIGITAL_NAMES = ("l1", "l2", "r1", "r2", "up", "down", "left", "right", "x", "b", "y", "a")
CARTRIDGES = {name: (preset.label, preset.rpm, preset.stall_torque)
              for name, preset in PHYSICAL_CARTRIDGES.items()}
APPLY_STATUS = "Applying experiment settings…"
RESET_STATUS = "Resetting scene…"

SETTING_HELP = {
    "workspace": "Choose the PROS project folder containing src/ and include/. Build workspace compiles its source for this desktop simulator. Rebuild explicitly after code changes; the V5 PROS CLI is only needed for physical-robot firmware.",
    "robot_urdf": "A standard URDF supplies links, mass, inertia, joints, and geometry. Keep referenced meshes beside it. Assign V5 motors in the import preview, then Apply settings to load the robot into the field.",
    "bindings": "Optional JSON for motor ports, joint assignments, external gearing, sensor mounts, and pneumatic joints. Motor assignments can also be edited below. Apply settings after changes.",
    "field": "Choose a game-pack JSON or field URDF. A URDF becomes static collision geometry; a game pack can also define movable pieces. Import selects the asset; Apply settings loads it.",
    "field_preset": "Full dynamics allow game pieces to move, collide, and be grasped. Fixed-piece route planning keeps pieces stationary and is faster, but cannot test scoring interactions. Apply settings after choosing a field.",
    "camera": "Change the viewpoint without moving the robot or restarting physics. Close view centers on the current robot; you can still orbit, pan, and zoom with the mouse.",
    "physics_hz": "Fixed simulation steps per simulated second. A higher rate resolves motor/contact changes more finely but costs more CPU/GPU work. Lowering it changes numerical accuracy; viewport FPS does not change this rate. For MuJoCo frictional grasps, keep the default 240 Hz or test a higher rate: 120 Hz softens its stabilized contacts and can let a pin creep in the claw before release.",
    "render_fps": "Maximum requested viewport updates per wall-clock second. Higher values increase rendering work. Lower values can improve simulation throughput without skipping or changing physics steps.",
    "gravity": "Downward acceleration in metres per second squared. Earth gravity is about 9.80665. Lower values reduce weight and contact loads; zero is useful for isolated mechanism experiments and does not represent a field robot.",
    "restitution": "Collision bounce from 0 (inelastic) to 1 (highly elastic). Increasing it makes colliding pieces rebound more strongly. Contact-pair combination rules depend on the physics backend.",
    "joint": "Choose a movable URDF joint for direct mechanism testing. This control bypasses your PROS motor commands; use the PID tuning lab to test a motor controller through its mapped ports.",
    "joint_target": "Direct joint position: radians for revolute/continuous joints, metres for prismatic joints. URDF travel limits bound the target. Stop robot code before moving a joint.",
    "X": "Robot base-link X coordinate in metres from field center. Apply settings to use the new starting position; Reset restores the last applied position.",
    "Y": "Robot base-link Y coordinate in metres from field center. Apply settings to use the new starting position; Reset restores the last applied position.",
    "Z": "Height of the URDF base-link origin in metres. Set enough clearance for the wheels and chassis; overlapping the floor or field objects can create large initial collision forces.",
    "Heading": "Starting yaw in degrees, counterclockwise about the upward +Z axis. This rotates the complete robot around its base-link origin.",
    "seed": "Seed for repeatable sensor-noise sequences. Keep it fixed when comparing code changes; change it to test another randomized trial. Reset restarts the selected sequence.",
    "position_std_m": "Gaussian standard deviation in metres added to the external odometry position measurement. Larger values test position-estimator robustness; the physical robot pose is unchanged.",
    "heading_std_deg": "Gaussian standard deviation in degrees added to heading measurements. Larger values perturb IMU/odometry feedback without rotating the physical robot.",
    "encoder_std_deg": "Gaussian standard deviation in motor degrees added to encoder positions. Larger values make PID position feedback noisier; motor velocity is not derived from this noisy position here.",
    "distance_std_mm": "Gaussian standard deviation in millimetres added to detected distance readings. Larger values test obstacle-detection thresholds; geometry and actual distances are unchanged.",
    "latency_ms": "Delay applied to delivered sensor snapshots, in milliseconds, resolved at physics ticks. Controllers receive older measurements while the physical scene continues moving.",
    "dropout": "Probability from 0 to 1 of dropping an available sensor update. A dropped update retains the last delivered snapshot; 0 delivers every update and 1 holds the initial reading.",
}

MOTOR_HELP = (
    "Physical V5 smart port, 1–21. Each port must be unique; several distinct motors may power the same URDF joint.",
    "Movable URDF joint driven by this motor. Match it to the real mechanism; several rows may share one joint to model a motor group.",
    "Reverse the mapping between positive motor commands/encoder readings and positive joint motion. Match the actual motor mounting and code port signs.",
    "Physical V5 cartridge: red 100 RPM, green 200 RPM, or blue 600 RPM. It selects motor shaft speed and torque; external gears use the Ratio setting.",
    "External ratio: motor output turns per joint turn. A larger reduction increases joint torque and reduces joint speed. This is separate from the internal VEX cartridge.",
    "Cartridge output-shaft stall torque in N·m. Use a measured value for calibration; changing cartridge restores its default. External ratio and efficiency determine joint torque.",
)


def _table_help(table, explanations):
    """Visible header help icons plus keyboard-accessible cell descriptions."""
    for column, explanation in enumerate(explanations):
        item = table.horizontalHeaderItem(column)
        if item is not None:
            item.setIcon(table.style().standardIcon(QStyle.StandardPixmap.SP_MessageBoxQuestion))
            item.setToolTip(explanation)
            item.setData(Qt.ItemDataRole.AccessibleDescriptionRole, explanation)


def _local_feedback(window, title, message, *, warning=False, details=""):
    """File/settings failures must not invalidate an otherwise running scene."""
    message = str(message)
    reporter = getattr(window, "feedback", None)
    if reporter is not None:
        (reporter.warning if warning else reporter.error)(title, message, details=details)
    else:
        window._log(("WARNING: " if warning else "ERROR: ") + message + ("\n" + str(details) if details else ""))
    window.statusBar().showMessage(message)
    if not warning:
        window._logo_event("error")


def _replay_time(seconds):
    seconds = max(0.0, float(seconds))
    minutes, seconds = divmod(seconds, 60)
    return f"{int(minutes):02d}:{seconds:06.3f}"


def _timeline_position(x, start, span, minimum, maximum, upside_down=False):
    """Map the handle's leading edge into the style-provided travel interval."""
    fraction = min(1.0, max(0.0, (x - start) / span)) if span > 0 else 0.0
    if upside_down:
        fraction = 1.0 - fraction
    return minimum + round(fraction * (maximum - minimum))


class ReplayTimeline(QSlider):
    """A video-style slider: groove clicks seek immediately, drags scrub."""
    seek_started = Signal()
    seek_requested = Signal(float)
    seek_finished = Signal(float)
    toggle_requested = Signal()

    def __init__(self):
        super().__init__(Qt.Orientation.Horizontal)
        self._seeking = False
        self.setRange(0, 0)
        self.setMinimumWidth(90)
        self.setAccessibleName("Recorded run timeline")
        self.setToolTip("Click or drag to seek. Left/Right: small seek; Home/End: endpoints; Space: play/pause.")

    def _at_pointer(self, event):
        option = QStyleOptionSlider()
        self.initStyleOption(option)
        groove = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, option,
                                             QStyle.SubControl.SC_SliderGroove, self)
        handle = self.style().subControlRect(QStyle.ComplexControl.CC_Slider, option,
                                             QStyle.SubControl.SC_SliderHandle, self)
        start, span = groove.x(), groove.width() - handle.width()
        return _timeline_position(event.position().x() - handle.width() / 2,
                                  start, span, self.minimum(), self.maximum(), option.upsideDown)

    def cancel_seek(self):
        self._seeking = False
        self.setSliderDown(False)

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton or not self.isEnabled():
            return super().mousePressEvent(event)
        self.setFocus()
        self._seeking = True
        self.setSliderDown(True)
        self.seek_started.emit()
        self.setValue(self._at_pointer(event))
        self.seek_requested.emit(self.value() / 1000)
        event.accept()

    def mouseMoveEvent(self, event):
        if not self._seeking:
            return super().mouseMoveEvent(event)
        self.setValue(self._at_pointer(event))
        self.seek_requested.emit(self.value() / 1000)
        event.accept()

    def mouseReleaseEvent(self, event):
        if not self._seeking or event.button() != Qt.MouseButton.LeftButton:
            return super().mouseReleaseEvent(event)
        self.setValue(self._at_pointer(event))
        self._seeking = False
        self.setSliderDown(False)
        self.seek_finished.emit(self.value() / 1000)
        event.accept()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Space:
            if not event.isAutoRepeat():
                self.toggle_requested.emit()
            event.accept()
            return
        before = self.value()
        super().keyPressEvent(event)
        if self.value() != before:
            self.seek_requested.emit(self.value() / 1000)


class AppLogoState:
    """Resolve expressive branding without letting routine telemetry erase outcomes."""

    def __init__(self):
        self.outcome = "happy"
        self.error = False
        self._was_running = False
        self._manual_stop = False

    def event(self, event):
        if event == "error":
            self.error = True
            return
        self.error = False
        self.outcome = {"success": "done_confirm_good", "stopped": "snooze",
                        "robot_import": "love", "field_import": "wink"}.get(event, "happy")
        self._manual_stop = event == "stopped"
        if event in ("begin", "reset"):
            self._was_running = False

    def observe(self, state):
        running = bool(state.get("running"))
        if self._was_running and not running:
            duration = float(state.get("duration", 0))
            complete = duration > 0 and float(state.get("mode_time", 0)) >= duration - 1e-8
            self.outcome = "done_confirm_good" if complete and not self._manual_stop else "snooze"
        self._was_running = running

    def name(self, state, *, building=False, loading=False):
        if self.error:
            return "question_confused"
        if building or loading:
            return "alert_normal"
        if state.get("paused"):
            return "snooze"
        if state.get("running"):
            if state.get("phase") == "initialize":
                return "alert_normal"
            return "swag" if state.get("mode") in ("skills_driver", "driver_auto") else "alert_happy"
        return self.outcome


def label(text: str, name: str = "", wrap: bool = False) -> QLabel:
    widget = QLabel(text)
    widget.setObjectName(name)
    widget.setWordWrap(wrap)
    return widget


def button(text: str, callback=None, name: str = "") -> QPushButton:
    widget = QPushButton(text)
    widget.setObjectName(name)
    if callback is not None:
        widget.clicked.connect(callback)
    return widget


def real_spin(low: float, high: float, value: float, decimals=3, suffix="") -> QDoubleSpinBox:
    widget = QDoubleSpinBox()
    widget.setRange(low, high)
    widget.setDecimals(decimals)
    widget.setValue(value)
    widget.setSingleStep(10 ** -min(decimals, 2))
    widget.setSuffix(suffix)
    return widget


def cartridge_select(motor: dict) -> QComboBox:
    combo = QComboBox()
    selected = motor.get("cartridge")
    rpm = float(motor.get("rpm", 200))
    if selected not in CARTRIDGES:
        selected = next((name for name, (_, speed, _) in CARTRIDGES.items() if speed == rpm), "custom")
    for name, (caption, _, _) in CARTRIDGES.items():
        combo.addItem(caption, name)
    if selected == "custom":
        combo.addItem(f"Custom · {rpm:g} RPM", "custom")
    combo.setCurrentIndex(combo.findData(selected))
    combo.setToolTip("Physical V5 11 W cartridge. External ratio is configured separately.")
    return combo


def fit_window(window: QWidget, width: int, height: int, minimum_width: int = 1120):
    """Leave room for native window borders on laptop displays."""
    screen = window.screen() or QApplication.primaryScreen()
    available = screen.availableGeometry() if screen else None
    max_width = max(640, available.width() - 24) if available else width
    max_height = max(480, available.height() - 48) if available else height
    window.setMinimumSize(min(minimum_width, max_width), min(640, max_height))
    window.resize(min(width, max_width), min(height, max_height))


class DiagnosticsExportDialog(QDialog):
    """Choose a reproducible diagnostic run without starting physics or graphics."""

    def __init__(self, destination: str, mode: str = "skills_auto", parent=None):
        super().__init__(parent)
        self.feedback = ErrorReporter(self, log=getattr(parent, "_log", None))
        self.setWindowTitle("Export performance diagnostics")
        self.setStyleSheet(STYLESHEET + "QDialog { background: #101e2d; }")
        self.setMinimumWidth(480)
        self.resize(580, 430)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(13)
        layout.addWidget(label("PERFORMANCE DIAGNOSTICS", "Section"))
        layout.addWidget(label("Export the settings currently shown, including edits you have not applied. Run the exported script later on this computer to measure performance.", "Muted", True))
        form = HelpFormLayout()
        folder_row = QHBoxLayout()
        self.destination = QLineEdit(destination)
        self.destination.setReadOnly(True)
        apply_help(self.destination, "Export location", "A new diagnostic folder will be created inside this location. Choose a writable existing parent folder.")
        folder_row.addWidget(self.destination, 1)
        folder_row.addWidget(button("Browse…", self._browse))
        form.addRow("Export location", folder_row, "Choose an existing writable parent folder. Export creates a new diagnostic folder there; it does not run benchmarks or upload files.")
        self.mode = QComboBox()
        for value, caption, _, _ in MODES:
            self.mode.addItem(caption, value)
        selected = self.mode.findData(mode)
        self.mode.setCurrentIndex(selected if selected >= 0 else self.mode.findData("skills_auto"))
        form.addRow("Robot program mode", self.mode, "Competition callback and duration used by the exported benchmark. Driver modes use neutral controller input; choose an autonomous mode to measure autonomous code.")
        self.wall_seconds = QSpinBox()
        self.wall_seconds.setRange(60, 600)
        self.wall_seconds.setSingleStep(30)
        self.wall_seconds.setValue(120)
        self.wall_seconds.setSuffix(" s per case")
        self.wall_seconds.setToolTip("Wall-clock measurement budget for each case. Scene loading, native compilation, and cleanup add time.")
        form.addRow("Wall-time limit", self.wall_seconds, self.wall_seconds.toolTip())
        layout.addLayout(form)
        self.compare_rates = QCheckBox("Compare 120 Hz and 240 Hz")
        self.compare_rates.setChecked(True)
        self.compare_rates.setToolTip("Run two separate cases. Uncheck to use the physics frequency currently shown in Field.")
        apply_help(self.compare_rates, "Compare physics rates", self.compare_rates.toolTip())
        compare_row = QHBoxLayout()
        compare_row.addWidget(self.compare_rates)
        compare_row.addWidget(help_label("", self.compare_rates.property("settingHelp"), self))
        compare_row.addStretch()
        layout.addLayout(compare_row)
        self.include_rendering = QCheckBox("Measure rendering separately")
        self.include_rendering.setChecked(True)
        self.include_rendering.setToolTip("After physics measurement, time a stationary scene with the selected renderer. No images are saved.")
        apply_help(self.include_rendering, "Measure rendering", self.include_rendering.toolTip())
        rendering_row = QHBoxLayout()
        rendering_row.addWidget(self.include_rendering)
        rendering_row.addWidget(help_label("", self.include_rendering.property("settingHelp"), self))
        rendering_row.addStretch()
        layout.addLayout(rendering_row)
        layout.addWidget(label("Driver modes use neutral controller input. The scripts keep local robot, field, and workspace paths; use them on this computer. Exporting does not run the tests.", "Muted", True))
        layout.addWidget(label("The report ZIP excludes raw PROS source and asset files. Nothing is uploaded automatically.", "Muted", True))
        self.error = label("", "Muted", True)
        layout.addWidget(self.error)
        actions = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        actions.button(QDialogButtonBox.StandardButton.Ok).setText("Export diagnostics")
        actions.button(QDialogButtonBox.StandardButton.Ok).setObjectName("Primary")
        actions.accepted.connect(self._accept_destination)
        actions.rejected.connect(self.reject)
        layout.addWidget(actions)

    def _browse(self):
        selected = QFileDialog.getExistingDirectory(self, "Choose diagnostic export location", self.destination.text())
        if selected:
            self.destination.setText(selected)
            self.error.setText("")

    def _accept_destination(self):
        if not self.destination.text().strip() or not Path(self.destination.text()).expanduser().is_dir():
            self.error.setText("Choose an existing folder for the diagnostic export.")
            self.feedback.warning("Choose an export folder", "Use Browse to select an existing writable folder, then export again.")
            return
        self.accept()

    def options(self):
        return {"destination": Path(self.destination.text()).expanduser(), "mode": self.mode.currentData(),
                "wall_seconds": self.wall_seconds.value(), "compare_rates": self.compare_rates.isChecked(),
                "include_rendering": self.include_rendering.isChecked()}

    def done(self, result):
        self.feedback.dismiss()
        super().done(result)


class RobotImportDialog(QDialog):
    """Map a standard URDF to V5 hardware without hand-writing a sidecar."""

    def __init__(self, inspection: dict, patch: dict, parent=None):
        super().__init__(parent)
        self.feedback = ErrorReporter(self, log=getattr(parent, "_log", None))
        from .preview import UrdfPreview

        self.setWindowTitle("Import robot · connect VEX hardware")
        fit_window(self, 1230, 810, 1060)
        self.patch = copy.deepcopy(patch)
        self._joint_types = {joint["name"]: joint["type"] for joint in inspection.get("joints", [])}
        self._jog_active = None
        self.preview = UrdfPreview()
        self._preview_closed = False
        self._preview_camera_initialized = False
        self._preview_camera_fit = None
        self._preview_revision = -1
        self._preview_state = {}
        self.setStyleSheet(STYLESHEET + "QDialog { background: #101e2d; }")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(13)
        layout.addWidget(label("CONNECT YOUR ROBOT", "Section"))
        layout.addWidget(label(str(inspection.get("name", "Imported URDF")), "Title"))
        self.preview_progress = OperationProgress(self)
        layout.addWidget(self.preview_progress)
        layout.addWidget(label("Assign smart ports to powered joints. Leave a port at 0 for a passive joint. You can add multiple motors per joint in the Robot tab after import.", "Muted", True))
        split = QSplitter(Qt.Orientation.Horizontal)
        split.setHandleWidth(14)
        split.setChildrenCollapsible(False)
        preview_pane = QWidget()
        preview_layout = QVBoxLayout(preview_pane)
        preview_layout.setContentsMargins(0, 0, 0, 0)
        preview_layout.setSpacing(10)
        preview_title = QHBoxLayout()
        preview_title.addWidget(label("LIVE JOINT PREVIEW", "Section"))
        preview_title.addStretch()
        preview_title.addWidget(button("Fit robot", self._fit_preview))
        preview_layout.addLayout(preview_title)
        self.preview_scene = SceneView(driving=False)
        self.preview_scene.setMinimumWidth(350)
        self.preview_scene.scene_name = "URDF PREVIEW"
        self.preview_scene.status = "BASE FIXED"
        self.preview_scene.set_camera(35, -25, 1.3, [0, 0, .15])
        preview_layout.addWidget(self.preview_scene, 1)
        preview_layout.addWidget(label("Base fixed for joint setup. Hold Jog to test the selected smart port; release to stop. Left-drag orbits, right-drag pans, and the wheel zooms.", "Muted", True))
        split.addWidget(preview_pane)
        hardware_pane = QWidget()
        hardware = QVBoxLayout(hardware_pane)
        hardware.setContentsMargins(0, 0, 0, 0)
        hardware.setSpacing(10)
        hardware.addWidget(label("PORTS & PHYSICAL CARTRIDGES", "Section"))
        self.rows = []
        self._lead_fields = {}
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(["URDF joint", "Smart port", "V5 cartridge", "Ratio", "Lead mm/rev", "Reverse"])
        self._column_help = (MOTOR_HELP[1], "Assign a unique smart port 1–21, or leave 0 for a passive joint.",
                             MOTOR_HELP[3], MOTOR_HELP[4],
                             "Linear travel in millimetres per motor output revolution, before the external gear ratio. Used only for prismatic joints such as lead screws.",
                             MOTOR_HELP[2])
        _table_help(self.table, self._column_help)
        self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for col, width in ((1, 90), (2, 164), (3, 77), (4, 110), (5, 85)):
            self.table.setColumnWidth(col, width)
        motors = copy.deepcopy(patch.get("motors") or [])
        known = {motor.get("joint") for motor in motors}
        for joint in inspection.get("joints", []):
            if joint.get("type") in ("continuous", "revolute", "prismatic") and joint.get("name") not in known:
                motors.append({"port": 0, "joint": joint["name"], "rpm": 200, "gear_ratio": 1})
        for index, motor in enumerate(motors):
            self.table.insertRow(index)
            item = QTableWidgetItem(str(motor.get("joint", "")))
            item.setFlags(Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
            item.setToolTip(self._column_help[0])
            self.table.setItem(index, 0, item)
            port = QSpinBox()
            port.setRange(0, 21)
            port.setSpecialValueText("Passive")
            signed_port = int(motor.get("port", 0))
            port.setValue(abs(signed_port))
            apply_help(port, "Smart port", self._column_help[1])
            self.table.setCellWidget(index, 1, port)
            cartridge = cartridge_select(motor)
            apply_help(cartridge, "V5 cartridge", MOTOR_HELP[3])
            self.table.setCellWidget(index, 2, cartridge)
            ratio = real_spin(0.001, 10000, float(motor.get("gear_ratio", 1)), 3)
            apply_help(ratio, "External ratio", MOTOR_HELP[4])
            self.table.setCellWidget(index, 3, ratio)
            if self._joint_types.get(motor.get("joint")) == "prismatic":
                lead = real_spin(.01, 10000, float(motor.get("lead_m", .01)) * 1000, 2)
                apply_help(lead, "Lead millimetres per revolution", self._column_help[4])
                self._lead_fields[index] = lead
                self.table.setCellWidget(index, 4, lead)
            else:
                unused = label("—", "Muted")
                unused.setAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setCellWidget(index, 4, unused)
            reverse = QTableWidgetItem()
            reverse.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
            reverse.setCheckState(Qt.CheckState.Checked if motor.get("reversed", signed_port < 0) else Qt.CheckState.Unchecked)
            reverse.setToolTip(MOTOR_HELP[2])
            self.table.setItem(index, 5, reverse)
            self.table.setRowHeight(index, 39)
            self.rows.append((motor, port, cartridge, ratio, reverse))
        hardware.addWidget(self.table, 1)
        hardware.addWidget(label("External ratio = motor output turns per joint turn. Your PROS gearing setting should match the physical cartridge.", "Muted", True))
        self.jog_label = label("Select an assigned smart port to test", "Title", True)
        hardware.addWidget(self.jog_label)
        jog_controls = QHBoxLayout()
        self.jog_reverse = button("Jog −")
        self.jog_stop = button("Stop", self._stop_preview, "Danger")
        self.jog_forward = button("Jog +", name="Primary")
        self.jog_reverse.pressed.connect(lambda: self._jog(-1))
        self.jog_forward.pressed.connect(lambda: self._jog(1))
        self.jog_reverse.released.connect(self._stop_preview)
        self.jog_forward.released.connect(self._stop_preview)
        for control in (self.jog_reverse, self.jog_stop, self.jog_forward):
            control.setAutoDefault(False)
            control.installEventFilter(self)
            jog_controls.addWidget(control)
        self.jog_voltage = QSpinBox()
        self.jog_voltage.setRange(500, 12000)
        self.jog_voltage.setSingleStep(500)
        self.jog_voltage.setValue(4000)
        self.jog_voltage.setSuffix(" mV")
        apply_help(self.jog_voltage, "Preview jog voltage", "Voltage applied while a Jog button is held, in millivolts. Higher voltage moves the selected motor faster and increases available torque. Release Jog or press Stop to disable output.")
        jog_controls.addWidget(help_label("Voltage", self.jog_voltage.property("settingHelp"), self))
        jog_controls.addWidget(self.jog_voltage)
        hardware.addLayout(jog_controls)
        self.jog_feedback = label("Motor output is zero until a Jog button is held.", "Muted", True)
        hardware.addWidget(self.jog_feedback)
        split.addWidget(hardware_pane)
        split.setSizes([520, 635])
        layout.addWidget(split, 1)
        warnings = inspection.get("warnings", [])
        if warnings:
            notice = QPlainTextEdit("\n".join(map(str, warnings)))
            notice.setReadOnly(True)
            notice.setMaximumHeight(90)
            layout.addWidget(notice)
        self.error = label("", "Muted", True)
        self.error.setStyleSheet("color: #efb0a5;")
        layout.addWidget(self.error)
        actions = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        actions.button(QDialogButtonBox.StandardButton.Ok).setText("Import robot")
        actions.accepted.connect(self._accept)
        actions.rejected.connect(self.reject)
        layout.addWidget(actions)
        self._preview_timer = QTimer(self)
        self._preview_timer.setSingleShot(True)
        self._preview_timer.setInterval(200)
        self._preview_timer.timeout.connect(self._configure_preview)
        for index, (_, port, cartridge, ratio, _) in enumerate(self.rows):
            port.valueChanged.connect(lambda value, row=index: self._queue_preview(row))
            cartridge.currentIndexChanged.connect(lambda value, row=index: self._queue_preview(row))
            ratio.valueChanged.connect(lambda value, row=index: self._queue_preview(row))
            if index in self._lead_fields:
                self._lead_fields[index].valueChanged.connect(lambda value, row=index: self._queue_preview(row))
        self.table.itemChanged.connect(lambda item: self._queue_preview(item.row()))
        self.table.currentCellChanged.connect(lambda *args: self._selected_preview_row())
        self.preview.frame.connect(self.preview_scene.set_frame)
        self.preview.state.connect(self._on_preview_state)
        self.preview.failed.connect(self._preview_failed)
        self.preview_scene.camera_changed.connect(self.preview.set_camera)
        self.preview_scene.focus_lost.connect(self._stop_preview)
        QApplication.instance().applicationStateChanged.connect(self._preview_app_state)
        self.installEventFilter(self)
        if self.rows:
            self.table.setCurrentCell(0, 0)
        self._selected_preview_row()
        QTimer.singleShot(0, self._configure_preview)

    def _motor_configuration(self):
        motors, ports = [], set()
        for index, (original, port_field, cartridge, ratio, reverse) in enumerate(self.rows):
            port = port_field.value()
            if not port:
                continue
            if port in ports:
                raise ValueError(f"Smart port {port} is assigned twice. Each V5 port needs a unique assignment.")
            ports.add(port)
            motor = copy.deepcopy(original)
            chosen = cartridge.currentData()
            original_choice = original.get("cartridge") or next((key for key, (_, rpm, _) in CARTRIDGES.items() if rpm == original.get("rpm", 200)), "custom")
            motor.update(port=port, reversed=reverse.checkState() == Qt.CheckState.Checked, gear_ratio=ratio.value())
            if index in self._lead_fields:
                motor["lead_m"] = self._lead_fields[index].value() / 1000
            if chosen in CARTRIDGES:
                _, rpm, torque = CARTRIDGES[chosen]
                motor.update(cartridge=chosen, rpm=rpm)
                if chosen != original_choice or "stall_torque" not in motor:
                    motor["stall_torque"] = torque
            motors.append(motor)
        return motors

    def _accept(self):
        try:
            self.patch["motors"] = self._motor_configuration()
        except ValueError as exc:
            self.error.setText(str(exc))
            self.feedback.warning("Check motor assignments", "Correct the duplicate or invalid port assignment before importing.", details=str(exc))
            return
        self.accept()

    def _queue_preview(self, row=None):
        self._stop_preview()
        if row is not None:
            self.table.setCurrentCell(row, 0)
        self._selected_preview_row()
        self._preview_timer.start()

    def _configure_preview(self):
        if self._preview_closed:
            return False
        self._preview_timer.stop()
        try:
            motors = self._motor_configuration()
        except ValueError as exc:
            self.error.setText(str(exc))
            self.preview.stop_all()
            return False
        self.error.clear()
        self.preview_progress.begin("preview", "Loading robot preview and motor bindings…")
        self.preview.configure(self.patch["robot_urdf"], motors)
        return True

    def _selected_preview_row(self):
        self._stop_preview()
        row = self.table.currentRow()
        assigned = 0 <= row < len(self.rows) and self.rows[row][1].value() != 0
        self.jog_forward.setEnabled(assigned)
        self.jog_reverse.setEnabled(assigned)
        if assigned:
            original, port, _, _, _ = self.rows[row]
            self.jog_label.setText(f"Port {port.value()}  ·  {original['joint']}")
        else:
            self.jog_label.setText("Assign a smart port to test this joint")

    def _jog(self, direction):
        if self._preview_closed:
            return
        if self._preview_timer.isActive() and not self._configure_preview():
            return
        row = self.table.currentRow()
        if 0 <= row < len(self.rows):
            port = self.rows[row][1].value()
            if port:
                self.preview.stop_all()
                voltage = direction * self.jog_voltage.value()
                self._jog_active = (port, voltage)
                self.preview.set_motor(port, voltage)
                self.jog_feedback.setText(f"Jogging port {port} at {voltage:+d} mV · release to stop")

    def _stop_preview(self):
        self._jog_active = None
        if not getattr(self, "_preview_closed", True):
            self.preview.stop_all()

    def _on_preview_state(self, state):
        if self._preview_closed:
            return
        self._preview_state = state
        graphics = state.get("renderer") or {}
        if graphics.get("backend") not in (None, "pending"):
            self.preview_scene.status = "BASE FIXED · " + ("GPU" if graphics.get("hardware_accelerated") else "CPU")
            self.preview_scene.setToolTip("Left-drag to orbit · right/middle-drag to pan · scroll to zoom\n" +
                                           str(graphics.get("device", "")) +
                                           ("\n" + str(graphics["reason"]) if graphics.get("reason") else ""))
        if state.get("ready") and state.get("camera") and state.get("revision", 0) != self._preview_revision:
            self.preview_progress.finish("preview")
            self._preview_revision = state.get("revision", 0)
            self._preview_camera_fit = state["camera"]
            if not self._preview_camera_initialized:
                self._preview_camera_initialized = True
                self._fit_preview()
            else:
                scene = self.preview_scene
                self.preview.set_camera(scene.yaw, scene.pitch, scene.distance, scene.target)
        row = self.table.currentRow()
        if 0 <= row < len(self.rows):
            name = self.rows[row][0]["joint"]
            joints = state.get("joints", [])
            if isinstance(joints, list):
                joint = next((item for item in joints if item.get("name") == name), None)
                if joint:
                    position = float(joint.get("position", 0))
                    measure = f"{position:+.4f} m" if self._joint_types.get(name) == "prismatic" else f"{math.degrees(position):+.2f}°"
                    voltage = self._jog_active[1] if self._jog_active else 0
                    self.jog_feedback.setText(f"Joint position {measure}  ·  motor command {voltage:+d} mV")

    def _fit_preview(self):
        if self._preview_camera_fit:
            camera = self._preview_camera_fit
            self.preview_scene.set_camera(camera["yaw"], camera["pitch"], camera["distance"], camera.get("target", [0, 0, 0]))

    def _preview_failed(self, message):
        self.error.setText("Preview: " + str(message))
        self._stop_preview()
        self.preview_progress.finish("preview", failed=True)
        self.feedback.error("Robot preview could not load", "Check the URDF, its referenced meshes, and the motor assignments. Correct the file or settings, then retry the preview.", details=str(message))

    def _preview_app_state(self, state):
        if state != Qt.ApplicationState.ApplicationActive:
            self._stop_preview()

    def eventFilter(self, watched, event):
        if event.type() in (QEvent.Type.WindowDeactivate, QEvent.Type.FocusOut):
            self._stop_preview()
        return super().eventFilter(watched, event)

    def done(self, result):
        self.feedback.dismiss()
        if not self._preview_closed:
            self._preview_timer.stop()
            self.preview.stop_all()
            self.preview.close()
            self._preview_closed = True
        super().done(result)


class SceneView(QWidget):
    """Aspect-correct camera presentation with orbit and focusable driving input."""

    camera_changed = Signal(float, float, float, object)
    key_changed = Signal(int, bool)
    focus_lost = Signal()
    replay_toggle = Signal()

    def __init__(self, driving: bool = True):
        super().__init__()
        self.setMinimumSize(440, 310)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        self._image: QImage | None = None
        self._drag: QPointF | None = None
        self._pan = False
        self.driving = driving
        self.target = [0.0, 0.0, 0.0]
        self.yaw, self.pitch, self.distance = 45.0, -52.0, 6.4
        self.scene_name = "PHYSICS VIEW"
        self.status = "INITIALIZING"
        self.score_text = ""
        self.setToolTip("Left-drag to orbit · right/middle-drag or Shift-drag to pan · scroll to zoom")

    def set_frame(self, frame):
        if isinstance(frame, QImage):
            self._image = frame.copy()
        else:
            import numpy as np
            pixels = np.ascontiguousarray(frame, dtype=np.uint8)
            if pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
                return
            height, width, channels = pixels.shape
            fmt = QImage.Format.Format_RGBA8888 if channels == 4 else QImage.Format.Format_RGB888
            self._image = QImage(pixels.data, width, height, pixels.strides[0], fmt).copy()
        self.update()

    def set_camera(self, yaw, pitch, distance, target=None):
        self.yaw, self.pitch, self.distance = float(yaw), float(pitch), float(distance)
        if target is not None:
            self.target = list(map(float, target))
        self.camera_changed.emit(self.yaw, self.pitch, self.distance, self.target.copy())

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#091420"))
        if self._image is not None:
            scaled = self._image.size().scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio)
            target = QRectF((self.width() - scaled.width()) / 2, (self.height() - scaled.height()) / 2,
                            scaled.width(), scaled.height())
            painter.drawImage(target, self._image)
        else:
            painter.setPen(QPen(QColor("#152a3c"), 1))
            for x in range(0, self.width(), 40):
                painter.drawLine(x, 0, x, self.height())
            for y in range(0, self.height(), 40):
                painter.drawLine(0, y, self.width(), y)
            painter.setPen(QColor("#92a9bd"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Loading the physics world…")
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(8, 19, 32, 215))
        painter.drawRoundedRect(QRectF(16, 16, min(self.width() - 32, 390), 53), 7, 7)
        painter.setPen(QColor(GOLD))
        painter.setFont(QFont("Segoe UI", 9, QFont.Weight.DemiBold))
        painter.drawText(28, 37, self.scene_name[:48].upper())
        painter.setPen(QColor("#b9c9d8"))
        painter.setFont(QFont("Segoe UI", 8))
        scene_kind = "RECORDED MOTION" if self.status.startswith("REPLAY") else "CONTACT DYNAMICS"
        painter.drawText(28, 55, scene_kind + "  /  SI UNITS  /  " + self.status)
        if self.score_text:
            badge_x = self.width() - 263 if self.width() >= 690 else 16
            badge_y = 16 if self.width() >= 690 else 78
            painter.setBrush(QColor(15, 39, 41, 230))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRoundedRect(QRectF(badge_x, badge_y, 247, 35), 6, 6)
            painter.setPen(QColor("#a5dfc5"))
            painter.drawText(QRectF(badge_x + 12, badge_y, 226, 35), Qt.AlignmentFlag.AlignVCenter, self.score_text)
        painter.setBrush(QColor(8, 19, 32, 220))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(QRectF(16, self.height() - 42, self.width() - 32, 27), 5, 5)
        painter.setPen(QColor("#c1ceda"))
        hint = "KEYBOARD ACTIVE · left WASD / right IJKL" if self.hasFocus() and self.driving else "Left: orbit  ·  Right: pan  ·  Scroll: zoom"
        painter.drawText(QRectF(28, self.height() - 42, self.width() - 56, 27), Qt.AlignmentFlag.AlignVCenter, hint)
        if self.hasFocus() and self.driving:
            painter.setPen(QPen(QColor(GOLD), 2))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRoundedRect(QRectF(1, 1, self.width() - 2, self.height() - 2), 6, 6)

    def mousePressEvent(self, event):
        self.setFocus()
        if event.button() in (Qt.MouseButton.LeftButton, Qt.MouseButton.RightButton, Qt.MouseButton.MiddleButton):
            self._drag = event.position()
            self._pan = event.button() != Qt.MouseButton.LeftButton or bool(event.modifiers() & Qt.KeyboardModifier.ShiftModifier)
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        if self._drag is not None:
            delta = event.position() - self._drag
            self._drag = event.position()
            if self._pan:
                yaw = math.radians(self.yaw)
                scale = self.distance * 0.0018
                target = self.target.copy()
                target[0] += (-delta.x() * math.cos(yaw) - delta.y() * math.sin(yaw)) * scale
                target[1] += (-delta.x() * math.sin(yaw) + delta.y() * math.cos(yaw)) * scale
                self.set_camera(self.yaw, self.pitch, self.distance, target)
            else:
                self.set_camera(self.yaw - delta.x() * 0.35, max(-89, min(-5, self.pitch - delta.y() * 0.3)), self.distance)

    def mouseReleaseEvent(self, event):
        self._drag = None
        self.setCursor(Qt.CursorShape.OpenHandCursor)

    def wheelEvent(self, event):
        self.set_camera(self.yaw, self.pitch, max(0.5, min(20, self.distance * math.exp(-event.angleDelta().y() / 1200))))
        event.accept()

    def keyPressEvent(self, event: QKeyEvent):
        if not self.driving and event.key() == Qt.Key.Key_Space and not event.isAutoRepeat():
            self.replay_toggle.emit()
        if self.driving and not event.isAutoRepeat():
            self.key_changed.emit(event.key(), True)
        event.accept()

    def keyReleaseEvent(self, event: QKeyEvent):
        if self.driving and not event.isAutoRepeat():
            self.key_changed.emit(event.key(), False)
        event.accept()

    def focusOutEvent(self, event):
        self.focus_lost.emit()
        self.update()
        super().focusOutEvent(event)

    def focusInEvent(self, event):
        self.update()
        super().focusInEvent(event)


class Stick(QWidget):
    moved = Signal(int, int)

    def __init__(self, title: str):
        super().__init__()
        self.title = title
        self.x_value = self.y_value = 0
        self._held = False
        self.setMinimumSize(130, 151)
        self.setMaximumHeight(155)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        center = QPointF(self.width() / 2, 61)
        painter.setBrush(QColor("#0a1726"))
        painter.setPen(QPen(QColor("#354b63"), 1))
        painter.drawEllipse(center, 43, 43)
        painter.setPen(QPen(QColor("#263d53"), 1))
        painter.drawLine(QPointF(center.x() - 43, center.y()), QPointF(center.x() + 43, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - 43), QPointF(center.x(), center.y() + 43))
        painter.setBrush(QColor(GOLD if self._held else "#9eafc0"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(center + QPointF(self.x_value / 127 * 33, -self.y_value / 127 * 33), 13, 13)
        painter.setPen(QColor("#a8bbce"))
        painter.drawText(QRectF(0, 110, self.width(), 18), Qt.AlignmentFlag.AlignCenter, self.title)
        painter.setPen(QColor("#6f859e"))
        painter.drawText(QRectF(0, 129, self.width(), 15), Qt.AlignmentFlag.AlignCenter,
                         f"{self.x_value:+4d}  /  {self.y_value:+4d}")

    def display(self, x: int, y: int):
        self.x_value, self.y_value = x, y
        self.update()

    def _move(self, point):
        x, y = (point.x() - self.width() / 2) / 43, -(point.y() - 61) / 43
        scale = max(1, math.hypot(x, y))
        self.display(round(x / scale * 127), round(y / scale * 127))
        self.moved.emit(self.x_value, self.y_value)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._held = True
            self._move(event.position())

    def mouseMoveEvent(self, event):
        if self._held:
            self._move(event.position())

    def mouseReleaseEvent(self, event):
        self._held = False
        self.display(0, 0)
        self.moved.emit(0, 0)


class Workbench(QMainWindow):
    def __init__(self, config: dict | None = None, workspace: str | None = None):
        super().__init__()
        self.config = copy.deepcopy(config if config is not None else default_config())
        self._applied_config = None
        self._loaded_urdf_path = self.config.get("robot_urdf", "")
        if workspace is not None:
            self.config["workspace"] = str(workspace)
        self.engine = SimulationEngine(defer_configuration=True, coalesce_frames=True)
        self._state: dict = {}
        self._replay: dict = {}
        self._replay_pending = None
        self._replay_pending_generation = -1
        self._view_generation = 0
        self._replay_dragging = False
        self._replay_resume_after_seek = False
        self._pending_replay_seek = None
        self._replay_reason = ""
        self._tuning_window = None
        self._tuning_close_pending = False
        self._built_workspace: str | None = None
        self._pending_workspace: str | None = None
        self._building = False
        self._reset_pending = False
        self._pending_scene_applies = 0
        self._scene_apply_ready = False
        self._logo = AppLogoState()
        self._logo_cache = {}
        self._shown_logo = None
        self._mode = "skills_auto"
        self._keys: set[int] = set()
        self._controller = {"analog": {name: 0 for name in ANALOG_NAMES},
                            "digital": {name: False for name in DIGITAL_NAMES}}
        self._pygame = self._gamepad = None
        self._joint_metadata: dict[str, dict] = {}
        self.setWindowTitle("EER SIM · WVU Robotics Autonomy Workbench")
        fit_window(self, 1500, 950)
        self.setStyleSheet(STYLESHEET)
        self.console = FeedbackConsole(self)
        self.log_view = self.console.editor
        self.feedback = ErrorReporter(self, log=self._log)
        console_action = self.console.toggleViewAction()
        console_action.setText("Console")
        self.menuBar().addMenu("View").addAction(console_action)
        root = QWidget()
        root.setObjectName("Root")
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(18, 14, 18, 6)
        layout.setSpacing(12)
        self._make_header(layout, workspace)
        self.operation_progress = OperationProgress(self)
        layout.addWidget(self.operation_progress)
        self._refresh_logo()
        split = QSplitter(Qt.Orientation.Horizontal)
        split.setHandleWidth(12)
        split.setChildrenCollapsible(False)
        split.addWidget(self._make_scene())
        split.addWidget(self._make_settings())
        split.setSizes([910, 520])
        layout.addWidget(split, 1)
        layout.addWidget(self._make_controls())
        self.statusBar().showMessage("Fixed physics steps preserve simulation fidelity when rendering slows.")
        self.engine.frame.connect(self._on_live_frame)
        self.engine.replay_frame.connect(self._on_replay_frame)
        self.engine.replay_state.connect(self._on_replay_state)
        self.engine.state.connect(self._on_state)
        self.engine.log.connect(self._log)
        self.engine.built.connect(self._on_built)
        self.engine.reset_completed.connect(self._on_reset_completed)
        self.engine.failed.connect(self._on_failed)
        self.engine.operation.connect(self._on_operation)
        self.engine.configuration_applied.connect(self._on_configuration_applied)
        self.scene.camera_changed.connect(self.engine.set_camera)
        self.scene.key_changed.connect(self._key)
        self.scene.focus_lost.connect(self._release_keys)
        self.scene.replay_toggle.connect(self._replay_toggle)
        self._replay_seek_timer = QTimer(self)
        self._replay_seek_timer.setSingleShot(True)
        self._replay_seek_timer.setInterval(35)
        self._replay_seek_timer.timeout.connect(self._flush_replay_seek)
        self._fill_widgets(self.config)
        self._controller_timer = QTimer(self)
        self._controller_timer.timeout.connect(self._poll_gamepad)
        self._controller_timer.start(25)
        QApplication.instance().applicationStateChanged.connect(self._application_state)
        self._refresh_replay_display()
        QTimer.singleShot(0, self._apply_config)

    def _make_header(self, parent, workspace):
        header = QHBoxLayout()
        self.brand_logo = QLabel()
        self.brand_logo.setFixedSize(58, 62)
        self.brand_logo.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.brand_logo.setAccessibleName("EERSim status logo")
        header.addWidget(self.brand_logo)
        identity = QVBoxLayout()
        identity.setSpacing(0)
        identity.addWidget(label("EER / SIM", "Brand"))
        identity.addWidget(label("WVU ROBOTICS  ·  AUTONOMY LAB", "BrandSub"))
        header.addLayout(identity)
        header.addStretch()
        workspace_row = QHBoxLayout()
        workspace_row.addWidget(help_label("PROS WORKSPACE", SETTING_HELP["workspace"], self))
        self.workspace = QLineEdit(str(workspace or self.config.get("workspace") or ""))
        self.workspace.setPlaceholderText("Choose a PROS project folder…")
        self.workspace.setMinimumWidth(240)
        apply_help(self.workspace, "PROS workspace", SETTING_HELP["workspace"])
        self.workspace.textChanged.connect(self._workspace_changed)
        workspace_row.addWidget(self.workspace, 1)
        workspace_row.addWidget(button("Browse…", self._choose_workspace))
        self.build_button = button("Build workspace", self._build, "Primary")
        header.addWidget(self.build_button)
        header.addWidget(button("PID tuning…", self._open_tuning))
        self.console_button = button("Console", self.console.toggleViewAction().trigger)
        self.console_button.setToolTip("Show or hide the detachable build and runtime console (Ctrl+Shift+C). Drag its title bar to undock it.")
        header.addWidget(self.console_button)
        self.status_badge = label("LOADING WORLD", "Badge")
        header.addWidget(self.status_badge)
        parent.addLayout(header)
        parent.addLayout(workspace_row)

    def _open_tuning(self):
        try:
            from .tuning_gui import TuningWindow
            if not isinstance(self._applied_config, dict):
                _local_feedback(self, "Apply a robot before tuning", "Wait for the scene to load successfully, or correct its settings and click Apply settings before opening PID tuning.", warning=True)
                return
            applied = copy.deepcopy(self._applied_config)
            if self._tuning_window is None:
                self._tuning_window = TuningWindow(self._workspace_key(), applied, self.engine, parent=self)
                self._tuning_window.shutdown_finished.connect(self._tuning_closed)
            else:
                self._tuning_window.set_workspace(self._workspace_key(), applied)
            self._tuning_window.show()
            self._tuning_window.raise_()
            self._tuning_window.activateWindow()
        except (ValueError, TypeError, OSError, ImportError) as exc:
            _local_feedback(self, "PID tuning could not open", "Check the robot's motor-port assignments and installed simulator dependencies, then open PID tuning again.", details=str(exc))

    def _on_configuration_applied(self, config):
        """Use worker-confirmed mappings, including edited files at the same path."""
        self._applied_config = copy.deepcopy(config)
        window = self._tuning_window
        if (window is not None and window.isVisible() and not window.busy
                and not getattr(window, "_capturing", False)):
            # Keep this tuner's source association; reopening from the main
            # button explicitly selects the currently chosen workspace.
            window.set_workspace(window.workspace, copy.deepcopy(self._applied_config))

    def _on_operation(self, event):
        self.operation_progress.update_event(event)
        if event.get("state") == "running":
            self.statusBar().showMessage(str(event.get("message", "Working…")))
        elif event.get("state") == "completed":
            self.statusBar().showMessage(str(event.get("message", "Operation complete.")))
        elif event.get("state") == "failed" and event.get("action") == "replay_enter" and event.get("error"):
            _local_feedback(self, "Replay could not load", "Return to Live and record a new run, or inspect the console for an asset or renderer error.", details=str(event["error"]))
        elif event.get("state") == "failed" and event.get("action") == "export" and event.get("error"):
            _local_feedback(self, "Telemetry export failed", "Choose a writable CSV destination and export again. The current simulation is still available.", details=str(event["error"]))

    def _tuning_closed(self):
        if self._tuning_close_pending:
            self._tuning_close_pending = False
            QTimer.singleShot(0, self.close)

    def _logo_event(self, event):
        self._logo.event(event)
        self._refresh_logo()

    def _refresh_logo(self):
        state = self._state
        loading = not state or bool(self._pending_scene_applies or self._scene_apply_ready or getattr(self, "_reset_pending", False))
        name = self._logo.name(state, building=self._building, loading=loading)
        if name == self._shown_logo:
            return
        if name not in self._logo_cache:
            icon = QIcon(str(ASSETS / "logos" / f"{name}.png"))
            if icon.isNull():
                icon = QIcon(str(ASSETS / "logos" / "happy.png"))
            if icon.isNull():
                return
            self._logo_cache[name] = (icon, icon.pixmap(54, 60))
        icon, pixmap = self._logo_cache[name]
        self._shown_logo = name
        self.setWindowIcon(icon)
        application = QApplication.instance()
        if application is not None:
            application.setWindowIcon(icon)
        self.brand_logo.setPixmap(pixmap)
        caption = {"happy": "Ready", "snooze": "Paused or stopped", "alert_normal": "Loading, initializing, or building",
                   "alert_happy": "Autonomous program running", "swag": "Driver program running",
                   "done_confirm_good": "Completed", "question_confused": "Needs attention",
                   "love": "Robot imported", "wink": "Field imported"}[name]
        self.brand_logo.setToolTip(caption)
        self.brand_logo.setAccessibleName("EERSim: " + caption)

    def _make_scene(self):
        pane = QWidget()
        layout = QVBoxLayout(pane)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(9)
        top = QHBoxLayout()
        top.addWidget(label("Simulation", "Title"))
        self.scene_subtitle = label("/ live world", "Muted")
        top.addWidget(self.scene_subtitle)
        top.addStretch()
        self.camera_preset = QComboBox()
        self.camera_preset.addItems(["Orbit camera", "Top down", "Driver station", "Close view"])
        self.camera_preset.currentIndexChanged.connect(self._camera_preset)
        apply_help(self.camera_preset, "Camera preset", SETTING_HELP["camera"])
        top.addWidget(help_label("View", SETTING_HELP["camera"], self))
        top.addWidget(self.camera_preset)
        layout.addLayout(top)
        self.scene = SceneView()
        # Keep the complete driver station reachable on a 1120 × 700 laptop.
        # Larger windows still give the expanding viewport all available space.
        self.scene.setMinimumHeight(170)
        layout.addWidget(self.scene, 1)
        metrics = QHBoxLayout()
        metrics.setSpacing(9)
        self.metric_values = {}
        self.metric_captions = {}
        for key, caption, initial in (("time", "SIMULATION TIME", "00:00.000"), ("remaining", "TIME REMAINING", "60.00 s"),
                                      ("rtf", "REAL-TIME FACTOR", "1.00×"), ("pose", "ROBOT POSITION", "0.00 / 0.00 m")):
            card = QFrame()
            card.setObjectName("Metric")
            box = QVBoxLayout(card)
            box.setContentsMargins(12, 9, 12, 9)
            box.setSpacing(2)
            self.metric_captions[key] = label(caption, "MetricLabel")
            box.addWidget(self.metric_captions[key])
            value = label(initial, "MetricValue")
            if key == "pose":
                value.setStyleSheet("font-size: 16px;")
            self.metric_values[key] = value
            box.addWidget(value)
            metrics.addWidget(card, 1)
        layout.addLayout(metrics)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        self.progress.setToolTip("Competition period elapsed in simulated time. Scene loading and build progress are shown separately above.")
        self.progress.setAccessibleName("Competition period progress")
        layout.addWidget(self.progress)
        transport = QHBoxLayout()
        transport.setSpacing(6)
        self.replay_button = button("Replay last run", self._replay_enter)
        self.replay_button.setEnabled(False)
        self.replay_play_button = button("Play", self._replay_toggle)
        self.replay_play_button.setEnabled(False)
        self.replay_time = label("00:00.000 / 00:00.000", "Muted")
        self.replay_time.setStyleSheet("font-size: 11px;")
        self.replay_time.setAccessibleName("Replay position and duration")
        self.replay_timeline = ReplayTimeline()
        self.replay_timeline.setEnabled(False)
        self.replay_timeline.seek_started.connect(self._replay_seek_started)
        self.replay_timeline.seek_requested.connect(self._queue_replay_seek)
        self.replay_timeline.seek_finished.connect(self._replay_seek_finished)
        self.replay_timeline.toggle_requested.connect(self._replay_toggle)
        self.replay_live_button = button("Live", self._replay_live)
        self.replay_live_button.setEnabled(False)
        self.replay_live_button.setToolTip("Return to the live world. Resume remains an explicit action.")
        transport.addWidget(self.replay_button)
        transport.addWidget(self.replay_play_button)
        transport.addWidget(self.replay_time)
        transport.addWidget(self.replay_timeline, 1)
        transport.addWidget(self.replay_live_button)
        layout.addLayout(transport)
        self.replay_status = label("No recorded run yet · completed and stopped runs become available here.", "Muted")
        self.replay_status.setStyleSheet("font-size: 11px;")
        self.replay_status.setWordWrap(True)
        self.replay_status.setMaximumHeight(34)
        layout.addWidget(self.replay_status)
        return pane

    def _tab(self, name: str):
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        self.tabs.addTab(scroll, name)
        return layout

    def _make_settings(self):
        pane = QWidget()
        pane.setMinimumWidth(440)
        layout = QVBoxLayout(pane)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(9)
        heading = QHBoxLayout()
        heading.addWidget(label("Experiment setup", "Title"))
        heading.addStretch()
        self.load_profile_button = button("Load", self._load_profile)
        self.save_profile_button = button("Save", self._save_profile)
        heading.addWidget(self.load_profile_button)
        heading.addWidget(self.save_profile_button)
        layout.addLayout(heading)
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.tabBar().setDrawBase(False)
        layout.addWidget(self.tabs, 1)
        self._workspace_tab()
        self._robot_tab()
        self._field_tab()
        self._pose_tab()
        self._sensors_tab()
        self._telemetry_tab()
        self.apply_button = button("Apply settings", self._apply_config)
        self.apply_button.setToolTip("Physical changes or reapplying unchanged settings reload the scene and end the run. Changing only viewport FPS or simulation speed keeps the current physics state. Use Apply after editing external asset files.")
        layout.addWidget(self.apply_button)
        return pane

    def _workspace_tab(self):
        layout = self._tab("Workspace")
        layout.addWidget(label("ROBOT PROGRAM", "Section"))
        layout.addWidget(label("Your code. A repeatable test field.", "Title"))
        layout.addWidget(label("Select your PROS workspace, build its C++ sources for the simulator, then choose a competition mode below.", "Muted", True))
        self.build_status = label("No native program built", "Muted", True)
        layout.addWidget(self.build_status)
        self.experiment_notes = label("", "Muted", True)
        self.experiment_notes.hide()
        layout.addWidget(self.experiment_notes)
        self.demo_button = button("Load claw scoring demo", self._load_demo)
        self.demo_button.setToolTip("Loads the example X-drive claw robot, current game field and PROS workspace. Press Build workspace when ready, then Start.")
        layout.addWidget(self.demo_button)
        layout.addWidget(label("Native PROS compatibility covers the APIs implemented by the simulator. Unsupported calls are reported by the build or runtime. Brain binaries and custom hardware protocols require adapters.", "Muted", True))
        layout.addWidget(label("DRIVER STATION", "Section"))
        row = QHBoxLayout()
        self.left_stick = Stick("LEFT  ·  WASD")
        self.right_stick = Stick("RIGHT  ·  IJKL")
        self.left_stick.moved.connect(lambda x, y: self._stick("left", x, y))
        self.right_stick.moved.connect(lambda x, y: self._stick("right", x, y))
        row.addWidget(self.left_stick)
        row.addWidget(self.right_stick)
        layout.addLayout(row)
        grid = QGridLayout()
        grid.setSpacing(5)
        self.digital_buttons = {}
        captions = {"l1": "L1 · Q", "l2": "L2 · R", "r1": "R1 · E", "r2": "R2 · F",
                    "a": "A · Z", "b": "B · X", "x": "X · C", "y": "Y · V",
                    "up": "↑", "down": "↓", "left": "←", "right": "→"}
        for index, name in enumerate(DIGITAL_NAMES):
            control = button(captions[name], name="Digital")
            control.setCheckable(False)
            control.pressed.connect(lambda n=name: self._digital(n, True))
            control.released.connect(lambda n=name: self._digital(n, False))
            grid.addWidget(control, index // 4, index % 4)
            self.digital_buttons[name] = control
        layout.addLayout(grid)
        layout.addWidget(label("Click the viewport to enable keyboard input. Sticks and buttons also work with the mouse. Releasing focus releases keyboard controls.", "Muted", True))
        self.gamepad_button = button("Connect USB gamepad", self._connect_gamepad)
        layout.addWidget(self.gamepad_button)
        self.gamepad_status = label("Standard SDL gamepads · optional pygame dependency", "Muted", True)
        layout.addWidget(self.gamepad_status)
        layout.addStretch()

    def _file_row(self, kind: str, file_filter: str, callback=None):
        row = QHBoxLayout()
        edit = QLineEdit()
        edit.setPlaceholderText(kind)
        edit.setMinimumWidth(120)
        row.addWidget(edit, 1)
        browse = button("…")
        browse.setFixedWidth(35)
        def choose():
            path, _ = QFileDialog.getOpenFileName(self, f"Choose {kind}", edit.text() or str(ASSETS), file_filter)
            if path:
                edit.setText(path)
                if callback:
                    callback(path)
        browse.clicked.connect(choose)
        row.addWidget(browse)
        return row, edit

    def _robot_tab(self):
        layout = self._tab("Robot")
        layout.addWidget(label("ROBOT DESCRIPTION", "Section"))
        row, self.urdf_path = self._file_row("robot URDF", "Robot description (*.urdf)", self._urdf_selected)
        self.urdf_path.setReadOnly(True)
        apply_help(self.urdf_path, "Robot URDF", SETTING_HELP["robot_urdf"])
        layout.addWidget(help_label("Robot file", SETTING_HELP["robot_urdf"], self))
        row.itemAt(1).widget().setFixedWidth(118)
        row.itemAt(1).widget().setText("Import URDF…")
        layout.addLayout(row)
        layout.addWidget(button("Preview robot & test motors", self._edit_robot_setup))
        self.urdf_path.editingFinished.connect(lambda: self._scan_joints(self.urdf_path.text()))
        layout.addWidget(label("Import a standard URDF, then assign V5 ports and cartridges in the setup dialog. Passive joints need no port.", "Muted", True))
        layout.addWidget(help_label("Optional hardware bindings", SETTING_HELP["bindings"], self))
        row, self.binding_path = self._file_row("hardware bindings", "Hardware bindings (*.json)", self._bindings_selected)
        self.binding_path.setPlaceholderText("Optional · motor assignments below work without a JSON file")
        apply_help(self.binding_path, "Hardware bindings", SETTING_HELP["bindings"])
        self.binding_path.editingFinished.connect(lambda: self._bindings_selected(self.binding_path.text()))
        layout.addLayout(row)
        presets = QHBoxLayout()
        presets.addWidget(button("WVU X-drive", lambda: self._robot_preset("xdrive")))
        presets.addWidget(button("Tank drive", lambda: self._robot_preset("tank")))
        presets.addWidget(button("X-drive + claw", lambda: self._robot_preset("xdrive_claw")))
        layout.addLayout(presets)
        layout.addWidget(label("SMART PORT → URDF JOINT", "Section"))
        self.motor_table = QTableWidget(0, 6)
        self.motor_table.setHorizontalHeaderLabels(["Port", "Joint", "Flip", "V5 cartridge", "Ratio", "Shaft\nN·m"])
        _table_help(self.motor_table, MOTOR_HELP)
        self.motor_table.verticalHeader().hide()
        self.motor_table.setAlternatingRowColors(True)
        self.motor_table.setMinimumHeight(177)
        self.motor_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        for col, width in ((0, 60), (2, 56), (3, 156), (4, 63), (5, 76)):
            self.motor_table.setColumnWidth(col, width)
        layout.addWidget(self.motor_table)
        row = QHBoxLayout()
        row.addWidget(button("+ Motor", self._add_motor))
        row.addWidget(button("Remove", self._remove_motor))
        row.addStretch()
        layout.addLayout(row)
        layout.addWidget(label("Cartridges set physical gearing and shaft stall torque. Ratio is external motor turns per joint turn. Edit shaft N·m for measured calibration; changing cartridge restores its default.", "Muted", True))
        layout.addWidget(label("MANUAL JOINT CONTROL", "Section"))
        form = HelpFormLayout()
        self.joint_select = QComboBox()
        self.joint_select.currentTextChanged.connect(self._joint_changed)
        form.addRow("Joint", self.joint_select, SETTING_HELP["joint"])
        self.joint_target = real_spin(-10000, 10000, 0, 3, " rad")
        form.addRow("Target", self.joint_target, SETTING_HELP["joint_target"])
        layout.addLayout(form)
        self.joint_button = button("Move selected joint", self._move_joint)
        layout.addWidget(self.joint_button)
        layout.addWidget(label("Test mechanisms independently of robot code. Revolute joints use radians; prismatic joints use meters. Stop the program to avoid competing motor commands.", "Muted", True))
        layout.addStretch()

    def _field_tab(self):
        layout = self._tab("Field")
        layout.addWidget(label("GAME PACK", "Section"))
        row, self.game_path = self._file_row("field or game pack", "Field imports (*.json *.urdf);;Game pack (*.json);;Field URDF (*.urdf)", self._import_field)
        self.game_path.setReadOnly(True)
        apply_help(self.game_path, "Field file", SETTING_HELP["field"])
        layout.addWidget(help_label("Field file", SETTING_HELP["field"], self))
        row.itemAt(1).widget().setFixedWidth(118)
        row.itemAt(1).widget().setText("Import field…")
        self.game_path.editingFinished.connect(lambda: self._describe_game(self.game_path.text()))
        layout.addLayout(row)
        self.field_preset = QComboBox()
        self.field_preset.addItem("Imported / custom field", None)
        self.field_preset.addItem("Autonomy practice field", "practice.json")
        self.field_preset.addItem("2026–27 Override · full game-piece dynamics", "override-2026-27.json")
        self.field_preset.addItem("Override route planning · fixed game pieces", "override-route-planning.json")
        self.field_preset.currentIndexChanged.connect(lambda index: self._game_preset(self.field_preset.itemData(index)) if self.field_preset.itemData(index) else None)
        apply_help(self.field_preset, "Field preset", SETTING_HELP["field_preset"])
        layout.addWidget(help_label("Field preset", SETTING_HELP["field_preset"], self))
        layout.addWidget(self.field_preset)
        self.game_title = label("Practice field", "Title", True)
        layout.addWidget(self.game_title)
        self.game_description = label("", "Muted", True)
        layout.addWidget(self.game_description)
        layout.addWidget(label("Choose the fixed-piece route-planning preset for fast path iteration. Full game-piece dynamics support grasping and stacking, and may run substantially below real time.", "Muted", True))
        layout.addWidget(label("Import a game pack JSON or a standard field URDF. URDF imports become static field geometry automatically. Game packs describe their visual sources and collision approximations; official scoring and referee rules are not simulated.", "Muted", True))
        layout.addWidget(label("PHYSICS & RENDERING", "Section"))
        form = HelpFormLayout()
        self.physics_select = QComboBox()
        self.physics_select.addItem("Auto · MuJoCo CPU", "auto")
        self.physics_select.addItem("MuJoCo · CPU", "mujoco")
        self.physics_select.addItem("Bullet · CPU", "bullet")
        self.physics_select.addItem("PhysX · NVIDIA CUDA", "physx_gpu")
        self.physics_select.addItem("PhysX · CPU", "physx_cpu")
        self.physics_select.setToolTip("Selects the engine that computes motion and contacts. Auto uses MuJoCo CPU and reports any Bullet fallback below. MuJoCo computes physics independently of GPU rendering. Explicit engine selections report an error if unavailable. Pin an engine in saved experiments for repeatable comparisons.")
        self.physics_device = QSpinBox()
        self.physics_device.setRange(0, 2147483647)
        self.physics_device.setToolTip("Logical CUDA device index, starting at 0, after any CUDA_VISIBLE_DEVICES filtering. This selects the physics GPU independently of the graphics renderer.")
        self.physics_iterations = QSpinBox()
        self.physics_iterations.setRange(1, 255)
        self.physics_iterations.setToolTip("Advanced contact and joint solver setting. Default 100. Reducing iterations changes solver convergence; validate grasping and stacking after changes. MuJoCo uses this as a Newton solver iteration limit and may converge earlier. Equal counts across engines do not imply equal accuracy.")
        self.physics_cpu_threads = QSpinBox()
        self.physics_cpu_threads.setRange(0, min(1024, max(2, os.cpu_count() or 1)))
        self.physics_cpu_threads.setSpecialValueText("0 · inline")
        self.physics_cpu_threads.setToolTip("Native physics task workers. MuJoCo can parallelize collision detection and independent constraint groups within each fixed step; PhysX uses workers for CPU and CUDA host work. 0 runs inline; default 2. More workers can be slower. EERSIM_PHYSX_THREADS overrides only PhysX. Does not parallelize dependent time steps or PROS callbacks.")
        self.physics_select.currentIndexChanged.connect(
            lambda: self.physics_cpu_threads.setEnabled(self.physics_select.currentData() != "bullet"))
        self.physics_select.currentIndexChanged.connect(
            lambda: self.physics_device.setEnabled(self.physics_select.currentData() == "physx_gpu"))
        self.physics_hz = QSpinBox()
        self.physics_hz.setRange(120, 2000)
        self.physics_hz.setSuffix(" Hz")
        self.render_fps = QSpinBox()
        self.render_fps.setRange(1, 120)
        self.render_fps.setSuffix(" FPS")
        self.renderer_select = QComboBox()
        self.renderer_select.addItem("Auto · GPU when available", "auto")
        self.renderer_select.addItem("GPU · OpenGL", "gpu")
        self.renderer_select.addItem("CPU · Bullet TinyRenderer", "cpu")
        self.renderer_select.setToolTip("Selects only viewport rendering. The Physics engine control independently selects motion and contact computation. Unsupported graphics fall back to CPU; Telemetry shows the active device and reason.")
        self.gravity = real_spin(0, 30, 9.80665, 5, " m/s²")
        self.friction = real_spin(0, 5, 0.9, 3)
        self.friction.setToolTip("Body coefficient for wheels and field tiles. The backends multiply the coefficients of the two touching bodies; omniwheel traction uses its separate tire model. Adjust from measured hardware behavior.")
        self.plastic_friction = real_spin(0, 5, 0.5, 3)
        self.plastic_friction.setToolTip("Body coefficient for pins, cups, goals, loaders and toggles. The backends multiply the two body coefficients: defaults give plastic/plastic μ = 0.25 and plastic/tile μ = 0.45. These are calibration assumptions, not official measured VEX values. Per-object game-pack overrides take precedence.")
        self.restitution = real_spin(0, 1, 0.05, 3)
        form.addRow("Physics engine", self.physics_select, self.physics_select.toolTip())
        form.addRow("CUDA physics device", self.physics_device, self.physics_device.toolTip())
        form.addRow("Solver iterations · advanced", self.physics_iterations, self.physics_iterations.toolTip())
        form.addRow("Physics CPU workers · advanced", self.physics_cpu_threads, self.physics_cpu_threads.toolTip())
        form.addRow("Fixed physics rate", self.physics_hz, SETTING_HELP["physics_hz"])
        form.addRow("Viewport frame rate", self.render_fps, SETTING_HELP["render_fps"])
        form.addRow("Graphics renderer", self.renderer_select, self.renderer_select.toolTip())
        form.addRow("Gravity", self.gravity, SETTING_HELP["gravity"])
        form.addRow("Wheel / tile grip", self.friction, self.friction.toolTip())
        form.addRow("Plastic contact friction", self.plastic_friction, self.plastic_friction.toolTip())
        form.addRow("Restitution", self.restitution, SETTING_HELP["restitution"])
        layout.addLayout(form)
        self.physics_field_status = label("Physics initialization pending…", "Muted", True)
        layout.addWidget(self.physics_field_status)
        layout.addWidget(label("Physics uses a fixed time step, independent of viewport FPS. On slower hardware the simulation takes longer in wall time; it keeps every physics step. Real-time factor shows this slowdown.", "Muted", True))
        layout.addStretch()

    def _pose_tab(self):
        layout = self._tab("Start pose")
        layout.addWidget(label("INITIAL CONDITIONS", "Section"))
        layout.addWidget(label("Start each run from the same state.", "Title", True))
        layout.addWidget(label("Coordinates are meters from field center. +Z is up; heading rotates counterclockwise about +Z. Z positions the URDF base link origin.", "Muted", True))
        form = HelpFormLayout()
        self.pose_fields = [real_spin(-20, 20, 0, 3, " m"), real_spin(-20, 20, 0, 3, " m"),
                            real_spin(0, 10, 0.15, 3, " m"), real_spin(-3600, 3600, 0, 2, " °")]
        for title, field in zip(("X", "Y", "Z", "Heading"), self.pose_fields):
            form.addRow(title, field, SETTING_HELP[title])
        layout.addLayout(form)
        row = QHBoxLayout()
        row.addWidget(button("Field center", lambda: self._pose_preset(0, 0, 0)))
        row.addWidget(button("Red side", lambda: self._pose_preset(-1.15, -1.15, 45)))
        row.addWidget(button("Blue side", lambda: self._pose_preset(1.15, 1.15, 225)))
        layout.addLayout(row)
        layout.addWidget(button("Use current robot pose", self._use_current_pose))
        layout.addWidget(label("REPRODUCIBILITY", "Section"))
        form = HelpFormLayout()
        self.seed = QSpinBox()
        self.seed.setRange(0, 2147483647)
        form.addRow("Noise random seed", self.seed, SETTING_HELP["seed"])
        layout.addLayout(form)
        layout.addWidget(label("Reset restores the scene, start pose and seeded sensor noise. Save an experiment profile to share the complete setup with your team.", "Muted", True))
        layout.addWidget(label("RUN DURATION", "Section"))
        layout.addWidget(label("Defaults follow VEX U periods: 60 s skills, 30 s match autonomous, 90 s driver. Adjust below for drills or other formats.", "Muted", True))
        form = HelpFormLayout()
        self.duration_fields = {}
        for mode, title, _, duration in MODES:
            field = real_spin(0.1, 3600, duration, 1, " s")
            self.duration_fields[mode] = field
            field.valueChanged.connect(self._duration_changed)
            callback = next(item[2] for item in MODES if item[0] == mode)
            form.addRow(title, field, f"Duration of {title} in simulated seconds. This mode runs {callback}(); slow hardware increases wall time, not this game clock. Applied on the next run.")
        layout.addLayout(form)
        layout.addStretch()

    def _sensors_tab(self):
        layout = self._tab("Sensors")
        layout.addWidget(label("ROBUSTNESS LAB", "Section"))
        layout.addWidget(label("Find the edge of your autonomy.", "Title", True))
        layout.addWidget(label("Inject seeded Gaussian measurement noise, sensor latency and dropped updates. Ground-truth pose in the viewport stays unmodified.", "Muted", True))
        form = HelpFormLayout()
        self.noise_fields = {}
        noise_settings = (("position_std_m", "Position σ", 2, 4, " m"),
                          ("heading_std_deg", "Heading σ", 90, 3, " °"),
                          ("encoder_std_deg", "Encoder σ", 180, 3, " °"),
                          ("distance_std_mm", "Distance σ", 1000, 2, " mm"),
                          ("latency_ms", "Sensor latency", 5000, 1, " ms"),
                          ("dropout", "Drop probability", 1, 3, ""))
        for key, title, high, precision, unit in noise_settings:
            field = real_spin(0, high, 0, precision, unit)
            self.noise_fields[key] = field
            form.addRow(title, field, SETTING_HELP[key])
        layout.addLayout(form)
        row = QHBoxLayout()
        row.addWidget(button("Ideal", lambda: self._noise_preset("ideal")))
        row.addWidget(button("Light noise", lambda: self._noise_preset("light")))
        row.addWidget(button("Stress test", lambda: self._noise_preset("stress")))
        layout.addLayout(row)
        layout.addWidget(label("These presets are test scenarios, not measured VEX sensor specifications. Calibrate distributions against recorded robot data for predictive results.", "Muted", True))
        layout.addWidget(label("SENSOR PLACEMENT", "Section"))
        layout.addWidget(label("Distance sensor mounts, smart ports and pneumatic joints are configured in the robot bindings JSON. Select that file in the Robot tab. Its paths and settings are preserved in your experiment profile.", "Muted", True))
        layout.addStretch()

    def _telemetry_tab(self):
        layout = self._tab("Telemetry")
        self.telemetry_caption = label("LIVE ROBOT STATE", "Section")
        layout.addWidget(self.telemetry_caption)
        self.pose_detail = label("Waiting for physics state…", "Muted", True)
        layout.addWidget(self.pose_detail)
        self.physics_detail = label("Physics initialization pending…", "Muted", True)
        layout.addWidget(self.physics_detail)
        self.renderer_detail = label("Graphics initialization pending…", "Muted", True)
        layout.addWidget(self.renderer_detail)
        self.telemetry_table = QTableWidget(0, 4)
        self.telemetry_table.setHorizontalHeaderLabels(["Port", "Position °", "RPM", "Torque N·m"])
        self.telemetry_table.verticalHeader().hide()
        self.telemetry_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.telemetry_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.telemetry_table.setMinimumHeight(160)
        layout.addWidget(self.telemetry_table)
        layout.addWidget(button("Export telemetry CSV…", self._export_csv))
        layout.addWidget(button("Export performance diagnostics…", self._export_diagnostics))
        self.diagnostics_status = label("Create a script to measure these settings on your computer.", "Muted", True)
        self.diagnostics_status.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.diagnostics_status)
        layout.addWidget(button("Show / hide console", self.console.toggleViewAction().trigger))
        layout.addWidget(label("Build and runtime details are available in the detachable console. Open it with Console or Ctrl+Shift+C; drag its title bar to use a separate window.", "Muted", True))
        layout.addStretch()

    def _make_controls(self):
        frame = QFrame()
        frame.setObjectName("ControlBar")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(8)
        titles = QVBoxLayout()
        titles.setSpacing(3)
        titles.addWidget(label("COMPETITION", "Section"))
        titles.addWidget(label("Select a mode", "Muted"))
        layout.addLayout(titles)
        layout.addSpacing(6)
        self.mode_group = QButtonGroup(self)
        self.mode_buttons = {}
        for mode, title, callback, duration in MODES:
            control = button(f"{title}\n{duration} s  ·  {callback}", name="Mode")
            control.setCheckable(True)
            control.setChecked(mode == self._mode)
            control.clicked.connect(lambda checked=False, m=mode: self._select_mode(m))
            control.setToolTip("Match driver period: calls opcontrol(). Label follows the requested Driver Auto mode." if mode == "driver_auto" else f"Runs {callback}() for {duration} simulated seconds.")
            self.mode_group.addButton(control)
            self.mode_buttons[mode] = control
            layout.addWidget(control, 1)
        layout.addSpacing(8)
        self.start_button = button("Start", self._start, "Primary")
        self.start_button.setEnabled(False)
        self.start_button.setToolTip("Build the selected PROS workspace first.")
        self.pause_button = button("Pause", self._pause_resume)
        self.pause_button.setEnabled(False)
        self.stop_button = button("Stop", self._stop, "Danger")
        self.stop_button.setEnabled(False)
        self.step_button = button("Step", self._single_step)
        self.step_button.setToolTip("Advance exactly one fixed physics step while paused.")
        self.reset_button = button("Reset", self._reset)
        self.reset_button.setToolTip("Restore the robot and field to the last applied starting state. The camera stays in place.")
        actions = QGridLayout()
        actions.setSpacing(6)
        actions.addWidget(self.start_button, 0, 0)
        actions.addWidget(self.pause_button, 0, 1)
        actions.addWidget(self.stop_button, 0, 2)
        actions.addWidget(self.reset_button, 1, 0, 1, 2)
        actions.addWidget(self.step_button, 1, 2)
        layout.addLayout(actions)
        return frame

    def _fill_widgets(self, config):
        self.urdf_path.setText(str(config.get("robot_urdf", "")))
        self._loaded_urdf_path = self.urdf_path.text()
        self._scan_joints(self.urdf_path.text())
        self.binding_path.setText(str(config.get("robot_config", "")))
        self.game_path.setText(str(config.get("game_pack", "")))
        self.physics_hz.setValue(int(config.get("physics_hz", 240)))
        self.physics_select.setCurrentIndex(max(0, self.physics_select.findData(config.get("physics_backend", "auto"))))
        self.physics_device.setValue(int(config.get("physics_device", 0)))
        self.physics_device.setEnabled(self.physics_select.currentData() == "physx_gpu")
        self.physics_iterations.setValue(int(config.get("physics_solver_iterations", 100)))
        threads = int(config.get("physics_cpu_threads", 2))
        self.physics_cpu_threads.setMaximum(min(1024, max(2, os.cpu_count() or 1, threads)))
        self.physics_cpu_threads.setValue(threads)
        self.physics_cpu_threads.setEnabled(self.physics_select.currentData() != "bullet")
        self.render_fps.setValue(int(config.get("render_fps", 30)))
        self.renderer_select.setCurrentIndex(max(0, self.renderer_select.findData(config.get("renderer", "auto"))))
        self.gravity.setValue(float(config.get("gravity", 9.80665)))
        self.friction.setValue(float(config.get("friction", 0.9)))
        self.plastic_friction.setValue(float(config.get("plastic_friction", 0.5)))
        self.restitution.setValue(float(config.get("restitution", 0.05)))
        self.seed.setValue(int(config.get("seed", 7)))
        self.experiment_notes.setText(str(config.get("notes", "")))
        self.experiment_notes.setVisible(bool(config.get("notes")))
        for field, value in zip(self.pose_fields, config.get("start_pose", [0, 0, 0.15, 0])):
            field.setValue(float(value))
        for key, field in self.noise_fields.items():
            field.setValue(float(config.get("noise", {}).get(key, 0)))
        for mode, _, _, duration in MODES:
            self.duration_fields[mode].setValue(float(config.get("duration_overrides", {}).get(mode, duration)))
        self._bindings_selected(self.binding_path.text(), log_errors=False)
        if config.get("motors") is not None:
            self._fill_motors(config["motors"])
        if config.get("motor_overrides"):
            motors = self._read_motors()
            for motor in motors:
                overrides = config["motor_overrides"].get(str(motor["port"]), {})
                if "rpm" in overrides and "cartridge" not in overrides:
                    motor.pop("cartridge", None)
                motor.update(overrides)
            self._fill_motors(motors)
        self._scan_joints(self.urdf_path.text())
        self._describe_game(self.game_path.text())
        if config.get("camera"):
            camera = config["camera"]
            self.scene.set_camera(camera.get("yaw", 45), camera.get("pitch", -52),
                                  camera.get("distance", 6.4), camera.get("target", [0, 0, 0]))
        if config.get("workspace"):
            self.workspace.setText(str(config["workspace"]))

    def _collect_config(self):
        result = copy.deepcopy(self.config)
        result.update(robot_urdf=self.urdf_path.text().strip(), robot_config=self.binding_path.text().strip(),
                      game_pack=self.game_path.text().strip(), workspace=self._workspace_key(),
                      physics_hz=self.physics_hz.value(), render_fps=self.render_fps.value(),
                      physics_backend=self.physics_select.currentData(), physics_device=self.physics_device.value(),
                      physics_solver_iterations=self.physics_iterations.value(),
                      physics_cpu_threads=self.physics_cpu_threads.value(),
                      renderer=self.renderer_select.currentData(),
                      gravity=self.gravity.value(), friction=self.friction.value(),
                      plastic_friction=self.plastic_friction.value(), restitution=self.restitution.value(),
                      start_pose=[field.value() for field in self.pose_fields], seed=self.seed.value(),
                      noise={key: field.value() for key, field in self.noise_fields.items()},
                      duration_overrides={key: field.value() for key, field in self.duration_fields.items()},
                      motors=self._read_motors(), motor_overrides={},
                      camera={"yaw": self.scene.yaw, "pitch": self.scene.pitch,
                              "distance": self.scene.distance, "target": self.scene.target.copy()})
        return result

    def _apply_config(self, *, reuse_unchanged=False):
        if not self._live_action_allowed():
            return False
        try:
            config = self._collect_config()
            for key in ("robot_urdf", "robot_config", "game_pack"):
                if key == "robot_config" and not config[key]:
                    continue
                if not Path(config[key]).is_file():
                    raise ValueError(f"Choose an existing {key.replace('_', ' ')} file.")
            self.config = config
            self._neutral_controller()
            self._pending_scene_applies += 1
            self._scene_apply_ready = False
            self.reset_button.setEnabled(False)
            self._logo_event("begin")
            if hasattr(self, "operation_progress"):
                self.operation_progress.begin("configure", APPLY_STATUS)
            self.engine.configure(config, reuse_unchanged=reuse_unchanged)
            self.engine.set_camera(self.scene.yaw, self.scene.pitch, self.scene.distance, self.scene.target)
            self.statusBar().showMessage(APPLY_STATUS)
            return True
        except (ValueError, OSError, TypeError) as exc:
            _local_feedback(self, "Settings could not be applied", str(exc) + " Check the selected files and motor assignments, then Apply settings again.")
            return False

    def _choose_workspace(self):
        if not self._live_action_allowed():
            return
        path = QFileDialog.getExistingDirectory(self, "Select a PROS workspace", self.workspace.text())
        if path:
            self.workspace.setText(str(Path(path)))

    def _workspace_key(self):
        text = self.workspace.text().strip()
        return str(Path(text).expanduser().resolve()) if text else ""

    def _workspace_changed(self):
        if hasattr(self, "start_button"):
            self.start_button.setEnabled(self._built_workspace == self._workspace_key() and not self._state.get("running", False)
                                         and not self._replay_blocked())
            if self._built_workspace != self._workspace_key():
                self.build_status.setText("Workspace changed · build before running")

    def _build(self):
        if not self._live_action_allowed():
            return
        key = self._workspace_key()
        if not key:
            _local_feedback(self, "Choose a robot workspace", "Use Browse to select the PROS project containing src/ and include/, then click Build workspace.", warning=True)
            return
        workspace = Path(key)
        if not workspace.is_dir() or not (workspace / "src").is_dir():
            _local_feedback(self, "Choose a robot workspace", "This folder is not a PROS workspace. Select the project containing src/ and include/, then build again.", warning=True)
            return
        self._building = True
        self._logo_event("begin")
        self._built_workspace = None
        self._pending_workspace = str(workspace)
        self.build_button.setEnabled(False)
        self.build_button.setText("Building…")
        self.workspace.setEnabled(False)
        self.start_button.setEnabled(False)
        self.build_status.setText("Compiling workspace C++ for the host simulator…")
        if hasattr(self, "operation_progress"):
            self.operation_progress.begin("build", "Compiling workspace C++ for the simulator…")
        self.engine.build(str(workspace))

    def _on_built(self, path):
        self._building = False
        self._built_workspace = self._pending_workspace
        self._logo_event("success")
        self.build_button.setEnabled(True)
        self.build_button.setText("Rebuild workspace")
        self.workspace.setEnabled(True)
        self.start_button.setEnabled(not self._state.get("running", False) and self._built_workspace == self._workspace_key())
        self.start_button.setToolTip("Start the selected competition period.")
        self.build_status.setText("Native program ready · " + Path(str(path)).name)
        self.build_status.setToolTip(str(path))
        self._log("Build complete. Select a competition mode and press Start.")
        self.statusBar().showMessage("Workspace built · ready to run.")
        self._update_live_controls()
        self._refresh_replay_display()

    def _start(self):
        if not self._live_action_allowed():
            return
        if self._built_workspace != self._workspace_key():
            _local_feedback(self, "Build before starting", "Click Build workspace for the selected project, wait for it to finish, then press Start.", warning=True)
            return
        # Start reuses an unchanged prepared scene; Session.start still resets
        # any scene that has advanced. Apply explicitly reloads edited assets.
        if self._apply_config(reuse_unchanged=True):
            self.engine.start(self._mode)
            self.scene.setFocus()

    def _pause_resume(self):
        if not self._live_action_allowed():
            return
        if self._state.get("paused", False):
            self.engine.resume()
            self.scene.setFocus()
        else:
            self._neutral_controller()
            self.engine.pause()

    def _stop(self):
        if not self._live_action_allowed():
            return
        self._neutral_controller()
        self._logo_event("stopped")
        self.engine.stop()

    def _reset(self):
        if not self._live_action_allowed():
            return
        if self._pending_scene_applies or getattr(self, "_scene_apply_ready", False):
            self.statusBar().showMessage(APPLY_STATUS)
            return
        self._neutral_controller()
        self._reset_pending = True
        self._logo_event("begin")
        self._update_live_controls()
        self._refresh_replay_display()
        self.statusBar().showMessage(RESET_STATUS)
        if hasattr(self, "operation_progress"):
            self.operation_progress.begin("reset", RESET_STATUS)
        self.engine.reset()

    def _on_reset_completed(self, state):
        # Only the worker's successful reset can acknowledge this request;
        # queued pre-reset telemetry must not make the button look ready.
        if not getattr(self, "_reset_pending", False):
            return
        self._reset_pending = False
        self._on_state(state)
        self._logo_event("reset")
        message = "Scene reset to the last applied starting state."
        self.statusBar().showMessage(message)
        self._log(message)

    def _select_mode(self, mode):
        self._mode = mode
        if not self._state.get("running", False):
            self.metric_values["remaining"].setText(f"{self.duration_fields[mode].value():.2f} s")

    def _duration_changed(self):
        if hasattr(self, "mode_buttons"):
            for mode, title, callback, _ in MODES:
                self.mode_buttons[mode].setText(f"{title}\n{self.duration_fields[mode].value():g} s  ·  {callback}")

    def _fill_motors(self, motors):
        self.motor_table.setRowCount(0)
        for motor in motors:
            self._add_motor(motor)

    def _add_motor(self, motor=None):
        if not isinstance(motor, dict):
            motor = {"port": self.motor_table.rowCount() + 1, "joint": self.joint_select.currentText() or "joint_name", "rpm": 600}
        row = self.motor_table.rowCount()
        self.motor_table.insertRow(row)
        port = int(motor.get("port", row + 1))
        rpm = float(CARTRIDGES[motor["cartridge"]][1] if motor.get("cartridge") in CARTRIDGES else motor.get("rpm", 200))
        values = [str(abs(port)), str(motor.get("joint", "")), None,
                  str(rpm), str(motor.get("gear_ratio", 1)), str(motor.get("stall_torque", 210 / rpm if rpm > 0 else 0.35))]
        for col, value in enumerate(values):
            item = QTableWidgetItem(value or "")
            item.setToolTip(MOTOR_HELP[col])
            item.setData(Qt.ItemDataRole.AccessibleDescriptionRole, MOTOR_HELP[col])
            if col == 0:
                item.setData(Qt.ItemDataRole.UserRole, copy.deepcopy(motor))
            if col == 2:
                item.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable)
                item.setCheckState(Qt.CheckState.Checked if motor.get("reversed", port < 0) else Qt.CheckState.Unchecked)
            self.motor_table.setItem(row, col, item)
        joint = QComboBox()
        joint.addItems(list(self._joint_metadata))
        if joint.findText(str(motor.get("joint", ""))) < 0:
            joint.addItem(str(motor.get("joint", "")))
        joint.setCurrentText(str(motor.get("joint", "")))
        apply_help(joint, "Motor joint", MOTOR_HELP[1])
        self.motor_table.setCellWidget(row, 1, joint)
        cartridge = cartridge_select(motor)
        apply_help(cartridge, "V5 cartridge", MOTOR_HELP[3])
        self.motor_table.setCellWidget(row, 3, cartridge)
        self.motor_table.setRowHeight(row, 35)
        cartridge.currentIndexChanged.connect(lambda index, widget=cartridge: self._motor_cartridge_changed(widget))

    def _motor_cartridge_changed(self, widget):
        selected = widget.currentData()
        if selected not in CARTRIDGES:
            return
        _, rpm, torque = CARTRIDGES[selected]
        for row in range(self.motor_table.rowCount()):
            if self.motor_table.cellWidget(row, 3) is widget:
                self.motor_table.item(row, 3).setText(str(rpm))
                self.motor_table.item(row, 5).setText(str(torque))
                break

    def _remove_motor(self):
        rows = sorted({index.row() for index in self.motor_table.selectedIndexes()}, reverse=True)
        for row in rows:
            self.motor_table.removeRow(row)

    def _read_motors(self):
        result, ports = [], set()
        for row in range(self.motor_table.rowCount()):
            try:
                port = int(self.motor_table.item(row, 0).text())
                name = self.motor_table.cellWidget(row, 1).currentText().strip()
                rpm = float(self.motor_table.item(row, 3).text())
                ratio = float(self.motor_table.item(row, 4).text())
                torque = float(self.motor_table.item(row, 5).text())
                if port not in range(1, 22) or port in ports or not name or not all(math.isfinite(v) and v > 0 for v in (rpm, ratio, torque)):
                    raise ValueError()
                motor = copy.deepcopy(self.motor_table.item(row, 0).data(Qt.ItemDataRole.UserRole) or {})
                motor.update(port=port, joint=name, reversed=self.motor_table.item(row, 2).checkState() == Qt.CheckState.Checked,
                             rpm=rpm, gear_ratio=ratio, stall_torque=torque)
                cartridge = self.motor_table.cellWidget(row, 3).currentData()
                if cartridge in CARTRIDGES:
                    motor["cartridge"] = cartridge
                else:
                    motor.pop("cartridge", None)
                result.append(motor)
                ports.add(port)
            except (ValueError, AttributeError, TypeError):
                raise ValueError(f"Motor row {row + 1}: use a unique port 1–21, a joint name, a cartridge, and positive ratio and torque.") from None
        return result

    def _bindings_selected(self, path, log_errors=True):
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
            self._fill_motors(data.get("motors", []))
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            if log_errors:
                _local_feedback(self, "Hardware bindings could not load", "Select a readable bindings JSON with a motors list, or clear the optional path and edit the motor table.", details=str(exc))

    def _urdf_selected(self, path, current=False):
        if not self._live_action_allowed():
            return
        try:
            from .importers import create_robot_config, inspect_robot_urdf
            inspection = inspect_robot_urdf(path)
            patch = ({"robot_urdf": path, "robot_config": self.binding_path.text(), "motors": self._read_motors()}
                     if current else create_robot_config(path))
            dialog = RobotImportDialog(inspection, patch, self)
            accepted = dialog.exec() == QDialog.DialogCode.Accepted
            imported_patch = copy.deepcopy(dialog.patch)
            dialog.deleteLater()
            if not accepted:
                self.urdf_path.setText(self._loaded_urdf_path)
                return
            self._accept_robot_import(imported_patch, inspection)
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.urdf_path.setText(self._loaded_urdf_path)
            _local_feedback(self, "Robot import failed", "Select a valid URDF and keep its referenced meshes beside it. Correct the file or port assignments, then import again.", details=str(exc))

    def _accept_robot_import(self, patch, inspection):
        if not self._live_action_allowed():
            return
        self.urdf_path.setText(str(patch["robot_urdf"]))
        self._loaded_urdf_path = self.urdf_path.text()
        self.binding_path.setText(str(patch.get("robot_config", "")))
        self._scan_joints(self.urdf_path.text())
        self._fill_motors(patch.get("motors", []))
        # Old robot calibration must never leak into an unfamiliar URDF.
        self.config["motor_overrides"] = {}
        self._log(f"Imported robot: {inspection.get('name', Path(patch['robot_urdf']).stem)} · {len(patch.get('motors', []))} powered smart ports")
        for warning in inspection.get("warnings", []):
            self._log("URDF: " + str(warning))
        if inspection.get("warnings"):
            _local_feedback(self, "Robot import notes", "The robot was imported with notes. Review its geometry and motor assignments before applying settings.",
                            warning=True, details="\n".join(map(str, inspection["warnings"])))
        self.statusBar().showMessage("Robot imported. Adjust start height and click Apply settings to load it into the field.")
        self._logo_event("robot_import")

    def _edit_robot_setup(self):
        if not self._live_action_allowed():
            return
        self._urdf_selected(self.urdf_path.text(), current=True)

    def _scan_joints(self, path):
        self.joint_select.clear()
        self._joint_metadata = {}
        try:
            tree = ET.parse(path)
            for joint in tree.getroot().findall("joint"):
                if joint.get("type") not in ("continuous", "revolute", "prismatic"):
                    continue
                name = joint.get("name", "")
                limit = joint.find("limit")
                self._joint_metadata[name] = {"type": joint.get("type"), "lower": limit.get("lower") if limit is not None else None,
                                              "upper": limit.get("upper") if limit is not None else None}
            self.joint_select.addItems(list(self._joint_metadata))
            for row in range(self.motor_table.rowCount()):
                combo = self.motor_table.cellWidget(row, 1)
                if combo is None:
                    continue
                selected = combo.currentText()
                combo.clear()
                combo.addItems(list(self._joint_metadata))
                if combo.findText(selected) < 0:
                    combo.addItem(selected)
                combo.setCurrentText(selected)
        except (OSError, ET.ParseError):
            pass

    def _joint_changed(self, name):
        if not hasattr(self, "joint_target"):
            return
        metadata = self._joint_metadata.get(name, {})
        self.joint_target.setSuffix(" m" if metadata.get("type") == "prismatic" else " rad")
        lower, upper = metadata.get("lower"), metadata.get("upper")
        self.joint_target.setRange(float(lower) if lower is not None else -10000,
                                  float(upper) if upper is not None else 10000)

    def _move_joint(self):
        if self._replay_blocked():
            return
        joint = self.joint_select.currentText()
        if joint:
            self.engine.set_joint(joint, self.joint_target.value())

    def _robot_preset(self, name):
        if not self._live_action_allowed():
            return
        self.urdf_path.setText(str(ASSETS / "robots" / f"{name}.urdf"))
        self._loaded_urdf_path = self.urdf_path.text()
        self.binding_path.setText(str(ASSETS / "robots" / f"{name}.json"))
        self._scan_joints(self.urdf_path.text())
        self._bindings_selected(self.binding_path.text())

    def _game_preset(self, name):
        if not self._live_action_allowed():
            return
        self.config.pop("field_objects", None)
        self.game_path.setText(str(ASSETS / "games" / name))
        self._describe_game(self.game_path.text())

    def _import_field(self, path):
        if not self._live_action_allowed():
            return
        try:
            from .importers import default_import_directory, import_field
            destination = default_import_directory(self.workspace.text().strip() or None)
            imported = import_field(path, dest=destination if Path(path).suffix.lower() == ".urdf" else None)
            self.config.pop("field_objects", None)
            self.game_path.setText(str(imported["path"]))
            self.field_preset.setCurrentIndex(0)
            self._describe_game(self.game_path.text())
            warnings = imported.get("metadata", {}).get("warnings", [])
            for warning in warnings:
                self._log("Field import: " + str(warning))
            if warnings:
                _local_feedback(self, "Review imported field", "The field was imported with assumptions. Review the details before using it for contact or scoring tests.", warning=True, details="\n".join(map(str, warnings)))
            self.statusBar().showMessage("Field imported. Click Apply settings to load its geometry.")
            self._logo_event("field_import")
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.game_path.setText(str(self.config.get("game_pack", "")))
            _local_feedback(self, "Field import failed", "Select a valid game-pack JSON or field URDF with its referenced assets, then import again.", details=str(exc))

    def _describe_game(self, path):
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
            name = str(data.get("name", Path(path).stem))
            self.game_title.setText(name)
            self.scene.scene_name = name
            details = data.get("description", data.get("notes", "Editable JSON game pack with physical collision geometry."))
            if isinstance(details, list):
                details = "\n".join(map(str, details))
            accuracy = data.get("geometry_accuracy", "")
            self.game_description.setText(str(details) + ("\n\n" + str(accuracy) if accuracy else ""))
            sources = data.get("sources", [])
            self.game_description.setToolTip("\n".join(map(str, sources)) if isinstance(sources, list) else str(sources))
            built_in = next((index for index in range(1, self.field_preset.count())
                             if Path(path).resolve() == (ASSETS / "games" / self.field_preset.itemData(index)).resolve()), 0)
            self.field_preset.blockSignals(True)
            self.field_preset.setCurrentIndex(built_in)
            self.field_preset.blockSignals(False)
            self.scene.update()
        except (OSError, ValueError, AttributeError):
            self.game_title.setText("Choose a game pack")
            self.game_description.setText("Select a valid JSON game pack to inspect its description.")

    def _pose_preset(self, x, y, yaw):
        for index, value in ((0, x), (1, y), (3, yaw)):
            self.pose_fields[index].setValue(value)

    def _pose_values(self, state):
        pose = state.get("pose", [0, 0, 0, 0])
        if isinstance(pose, dict):
            return [float(pose.get("x", 0)), float(pose.get("y", 0)), float(pose.get("z", 0)),
                    float(pose.get("yaw_deg", math.degrees(pose.get("yaw", pose.get("theta", 0)))))]
        values = (list(pose) + [0, 0, 0, 0])[:4]
        values[3] = math.degrees(float(values[3]))
        return values

    def _use_current_pose(self):
        for field, value in zip(self.pose_fields, self._pose_values(self._state)):
            field.setValue(value)

    def _noise_preset(self, name):
        values = {"ideal": (0, 0, 0, 0, 0, 0), "light": (0.002, 0.3, 0.2, 5, 10, 0.005),
                  "stress": (0.025, 3, 2, 35, 100, 0.08)}[name]
        for field, value in zip(self.noise_fields.values(), values):
            field.setValue(value)

    def _camera_preset(self, index):
        observed = self._replay if self._replay.get("active", False) else self._state
        target = self._pose_values(observed)[:3] if index == 3 else [0, 0, 0]
        self.scene.set_camera(*((45, -52, 6.4), (0, -89, 5), (90, -24, 6.4), (45, -30, 1.3))[index], target=target)

    def _save_profile(self):
        if not self._live_action_allowed():
            return
        try:
            initial = _dialog_default("experiment.json", "experiments", getattr(self, "_profile_path", None),
                                      self.workspace.text().strip() or None)
            path, _ = QFileDialog.getSaveFileName(self, "Save experiment profile", initial, "Experiment profile (*.json)")
            if not path:
                return
            from .config import save_config
            config = self._collect_config()
            save_config(config, path)
            self._profile_path = path
            self.statusBar().showMessage(f"Saved experiment profile: {path}")
            self._logo_event("success")
        except (OSError, ValueError, TypeError) as exc:
            _local_feedback(self, "Experiment profile could not save", "Check the motor settings and choose a writable output folder, then save again.", details=str(exc))

    def _load_profile(self):
        if not self._live_action_allowed():
            return
        try:
            initial = _dialog_default("", "experiments", getattr(self, "_profile_path", None),
                                      self.workspace.text().strip() or None)
            path, _ = QFileDialog.getOpenFileName(self, "Load experiment profile", initial, "Experiment profile (*.json)")
            if not path:
                return
            from .config import load_config
            config = load_config(path)
            self._fill_widgets(config)
            self.config = config
            self._profile_path = path
            self._apply_config()
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            _local_feedback(self, "Experiment profile could not load", "Choose a valid experiment JSON and check that its referenced robot and field files are available.", details=str(exc))

    def _load_demo(self):
        if not self._live_action_allowed():
            return
        try:
            from .demos import claw_demo_config
            config = claw_demo_config()
            self._fill_widgets(config)
            self.config = config
            if self._apply_config():
                self._select_mode("skills_auto")
                self.mode_buttons["skills_auto"].setChecked(True)
                self._built_workspace = None
                self.start_button.setEnabled(False)
                self.build_status.setText("Claw demo loaded · Build workspace before running")
                self._log("Claw demo loaded. Press Build workspace, then Start.")
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            _local_feedback(self, "Claw demo could not load", "Check that the bundled demo assets are installed and the user-data folder is writable, then try loading the demo again.", details=str(exc))

    def _export_csv(self):
        try:
            initial = _dialog_default("telemetry.csv", "telemetry", getattr(self, "_telemetry_path", None),
                                      self.workspace.text().strip() or None)
            path, _ = QFileDialog.getSaveFileName(self, "Export run telemetry", initial, "CSV data (*.csv)")
            if path:
                self._telemetry_path = path
                self.engine.export_csv(path)
        except (OSError, ValueError, TypeError) as exc:
            _local_feedback(self, "Telemetry export failed", "Choose a writable CSV destination and export again.", details=str(exc))

    def _export_diagnostics(self):
        try:
            destination = getattr(self, "_diagnostics_directory", "")
            if not destination or not Path(destination).is_dir():
                destination = QStandardPaths.writableLocation(QStandardPaths.StandardLocation.DocumentsLocation)
            if not destination or not Path(destination).is_dir():
                destination = str(Path.home())
            dialog = DiagnosticsExportDialog(destination, getattr(self, "_mode", "skills_auto"), self)
            if dialog.exec() != QDialog.DialogCode.Accepted:
                return
            options = dialog.options()
            from .diagnostics_export import export_diagnostics
            exported = export_diagnostics(self._collect_config(), **options)
            self._diagnostics_directory = str(options["destination"])
            run = ("Run in PowerShell inside that folder:\npowershell -NoProfile -ExecutionPolicy Bypass -File .\\run-diagnostics.ps1"
                   if sys.platform == "win32" else "See the instructions in the exported folder to run the diagnostics.")
            message = f"Diagnostics exported to {exported}\n{run}"
            self.diagnostics_status.setText(message)
            self._log(message.replace("\n", " · "))
            self.statusBar().showMessage(f"Performance diagnostics exported to {exported}")
            self._logo_event("success")
        except (OSError, ValueError) as exc:
            message = f"Could not export performance diagnostics: {exc}"
            self.diagnostics_status.setText(message)
            _local_feedback(self, "Diagnostic export failed", message,
                            details="Choose an existing writable export folder and check the selected robot/field settings. Exporting does not run or rebuild the workspace.")

    def _key(self, key, down):
        if self._replay_blocked() or getattr(self, "_reset_pending", False):
            return
        if down:
            self._keys.add(key)
        else:
            self._keys.discard(key)
        pairs = {"left_x": (Qt.Key.Key_D, Qt.Key.Key_A), "left_y": (Qt.Key.Key_W, Qt.Key.Key_S),
                 "right_x": (Qt.Key.Key_L, Qt.Key.Key_J), "right_y": (Qt.Key.Key_I, Qt.Key.Key_K)}
        for name, (positive, negative) in pairs.items():
            self._controller["analog"][name] = 127 * (int(positive in self._keys) - int(negative in self._keys))
        keys = {Qt.Key.Key_Q: "l1", Qt.Key.Key_R: "l2", Qt.Key.Key_E: "r1", Qt.Key.Key_F: "r2",
                Qt.Key.Key_Z: "a", Qt.Key.Key_X: "b", Qt.Key.Key_C: "x", Qt.Key.Key_V: "y",
                Qt.Key.Key_Up: "up", Qt.Key.Key_Down: "down", Qt.Key.Key_Left: "left", Qt.Key.Key_Right: "right"}
        for code, name in keys.items():
            self._controller["digital"][name] = code in self._keys
        self._send_controller()

    def _release_keys(self):
        if self._keys:
            self._keys.clear()
            self._neutral_controller()

    def _stick(self, side, x, y):
        if self._replay_blocked() or getattr(self, "_reset_pending", False):
            return
        self._controller["analog"][side + "_x"] = x
        self._controller["analog"][side + "_y"] = y
        self._send_controller()

    def _digital(self, name, down):
        if self._replay_blocked() or getattr(self, "_reset_pending", False):
            return
        self._controller["digital"][name] = down
        self._send_controller()

    def _send_controller(self):
        if self._replay_blocked() or getattr(self, "_reset_pending", False):
            return
        self.left_stick.display(self._controller["analog"]["left_x"], self._controller["analog"]["left_y"])
        self.right_stick.display(self._controller["analog"]["right_x"], self._controller["analog"]["right_y"])
        for name, control in self.digital_buttons.items():
            control.setDown(self._controller["digital"][name])
        self.engine.set_controller(copy.deepcopy(self._controller))

    def _neutral_controller(self):
        self._keys.clear()
        self._controller = {"analog": {name: 0 for name in ANALOG_NAMES}, "digital": {name: False for name in DIGITAL_NAMES}}
        self._send_controller()

    def _application_state(self, state):
        if state != Qt.ApplicationState.ApplicationActive:
            self._neutral_controller()

    def _connect_gamepad(self):
        if self._replay_blocked() or getattr(self, "_reset_pending", False):
            return
        if self._gamepad is not None:
            self._gamepad.quit()
            self._gamepad = None
            self.gamepad_button.setText("Connect USB gamepad")
            self.gamepad_status.setText("Gamepad disconnected")
            self._neutral_controller()
            return
        try:
            self._pygame = importlib.import_module("pygame")
            self._pygame.joystick.init()
            self._pygame.display.init()
            if not self._pygame.joystick.get_count():
                self.gamepad_status.setText("No gamepad detected. Connect a USB/Bluetooth controller and try again.")
                _local_feedback(self, "No gamepad detected", self.gamepad_status.text(), warning=True)
                return
            self._gamepad = self._pygame.joystick.Joystick(0)
            self._gamepad.init()
            self.gamepad_status.setText(self._gamepad.get_name() + " · standard SDL axes/buttons")
            self.gamepad_button.setText("Disconnect gamepad")
        except ImportError:
            self.gamepad_status.setText('Gamepad support needs pygame. Install the simulator with its [gamepad] extra.')
            _local_feedback(self, "Gamepad support unavailable", "Source installs need the [gamepad] extra. For an installed executable, reinstall the current package. Keyboard and on-screen driving are available in the meantime.", warning=True)
        except Exception as exc:
            self.gamepad_status.setText(f"Gamepad connection failed: {exc}")
            _local_feedback(self, "Gamepad connection failed", "Reconnect the controller and retry Connect USB gamepad. Keyboard and on-screen driving remain available.", warning=True, details=str(exc))

    def _poll_gamepad(self):
        if self._replay_blocked() or getattr(self, "_reset_pending", False):
            return
        if self._gamepad is None or QApplication.applicationState() != Qt.ApplicationState.ApplicationActive:
            return
        if QApplication.activeModalWidget() is not None:
            if any(self._controller["analog"].values()) or any(self._controller["digital"].values()):
                self._neutral_controller()
            return
        try:
            self._pygame.event.pump()
            def axis(index, invert=False):
                value = self._gamepad.get_axis(index) if index < self._gamepad.get_numaxes() else 0
                return 0 if abs(value) < 0.08 else round(max(-1, min(1, value)) * (-127 if invert else 127))
            analog = dict(zip(ANALOG_NAMES, (axis(0), axis(1, True), axis(2), axis(3, True))))
            digital = {name: False for name in DIGITAL_NAMES}
            for index, name in enumerate(("a", "b", "x", "y", "l1", "r1")):
                digital[name] = index < self._gamepad.get_numbuttons() and bool(self._gamepad.get_button(index))
            if self._gamepad.get_numaxes() > 5:
                digital["l2"], digital["r2"] = self._gamepad.get_axis(4) > 0.2, self._gamepad.get_axis(5) > 0.2
            if self._gamepad.get_numhats():
                x, y = self._gamepad.get_hat(0)
                digital.update(up=y > 0, down=y < 0, left=x < 0, right=x > 0)
            controller = {"analog": analog, "digital": digital}
            if controller != self._controller:
                self._controller = controller
                self._send_controller()
        except Exception as exc:
            self._gamepad = None
            self.gamepad_button.setText("Connect USB gamepad")
            self.gamepad_status.setText(f"Gamepad disconnected: {exc}")
            self._neutral_controller()
            _local_feedback(self, "Gamepad disconnected", "Controller inputs have been released. Reconnect the controller and select Connect USB gamepad to resume driving.", warning=True, details=str(exc))

    def _replay_blocked(self):
        return bool(getattr(self, "_replay_pending", None) or getattr(self, "_replay", {}).get("active"))

    def _live_action_allowed(self):
        if getattr(self, "_reset_pending", False):
            self.statusBar().showMessage(RESET_STATUS)
            return False
        if self._replay_blocked():
            self.statusBar().showMessage("Return to Live to change the simulation.")
            return False
        return True

    def _single_step(self):
        if self._live_action_allowed():
            self.engine.single_step()

    def _replay_enter(self):
        if self._replay_blocked() or getattr(self, "_reset_pending", False) or not self._replay.get("available") or self._building or self._pending_scene_applies:
            return
        self._neutral_controller()
        self._live_scene_name = self.scene.scene_name
        self._replay_pending = "enter"
        self._replay_pending_generation = self._view_generation
        self._update_live_controls()
        self._refresh_replay_display()
        if hasattr(self, "operation_progress"):
            self.operation_progress.begin("replay_enter", "Preparing the previous run for playback…")
        self.engine.replay_enter()

    def _replay_live(self):
        if not self._replay.get("active") or self._replay_pending:
            return
        self._replay_seek_timer.stop()
        self._pending_replay_seek = None
        self._replay_dragging = False
        self._replay_resume_after_seek = False
        self._replay_pending = "live"
        self._replay_pending_generation = self._view_generation
        self._update_live_controls()
        self._refresh_replay_display()
        self.engine.replay_live()

    def _replay_toggle(self):
        if not self._replay.get("active") or self._replay_pending or self._replay_dragging:
            return
        if self._replay.get("playing"):
            self.engine.replay_pause()
        else:
            self._flush_replay_seek()
            self.engine.replay_play()

    def _replay_seek_started(self):
        if not self._replay.get("active") or self._replay_pending:
            return
        self._replay_dragging = True
        self._replay_resume_after_seek = bool(self._replay.get("playing"))
        self.engine.replay_pause()

    def _queue_replay_seek(self, seconds):
        if not self._replay.get("active") or self._replay_pending:
            return
        seconds = float(seconds)
        if not math.isfinite(seconds):
            return
        duration = max(0.0, float(self._replay.get("duration_s", 0)))
        self._pending_replay_seek = min(duration, max(0.0, seconds))
        self.replay_time.setText(f"{_replay_time(self._pending_replay_seek)} / {_replay_time(duration)}")
        if not self._replay_seek_timer.isActive():
            self._replay_seek_timer.start()

    def _flush_replay_seek(self):
        self._replay_seek_timer.stop()
        seconds, self._pending_replay_seek = self._pending_replay_seek, None
        if seconds is not None and self._replay.get("active") and not self._replay_pending:
            self.engine.replay_seek(seconds)

    def _replay_seek_finished(self, seconds):
        if not self._replay_dragging:
            return
        resume = self._replay_resume_after_seek
        self._replay_dragging = False
        self._replay_resume_after_seek = False
        self._queue_replay_seek(seconds)
        self._flush_replay_seek()
        if resume and self._replay.get("active") and not self._replay_pending:
            self.engine.replay_play()

    def _on_live_frame(self, packet):
        if isinstance(packet, dict) and "frame_token" in packet:
            packet = self.engine.take_frame(packet)
            if packet is None:
                return
        if self._replay_blocked():
            return
        if isinstance(packet, dict):
            if packet.get("generation") != self._view_generation or "pixels" not in packet:
                return
            packet = packet["pixels"]
        elif self._view_generation != 0:
            return
        self.scene.set_frame(packet)

    def _on_replay_frame(self, packet):
        if isinstance(packet, dict) and "frame_token" in packet:
            packet = self.engine.take_frame(packet)
            if packet is None:
                return
        if not self._replay.get("active") or self._replay_pending:
            return
        if not isinstance(packet, dict) or packet.get("generation") != self._view_generation or "pixels" not in packet:
            return
        self.scene.set_frame(packet["pixels"])

    def _on_replay_state(self, state):
        generation = int(state.get("generation", 0))
        if generation < self._view_generation:
            return
        if self._replay_pending and generation <= self._replay_pending_generation:
            return  # Old queued metadata cannot acknowledge a new source selection.
        changed = generation != self._view_generation or bool(state.get("active")) != bool(self._replay.get("active"))
        self._view_generation = generation
        self._replay = dict(state)
        self._replay_pending = None
        if changed:
            self._replay_seek_timer.stop()
            self._pending_replay_seek = None
            self._replay_dragging = False
            self._replay_resume_after_seek = False
            self.replay_timeline.cancel_seek()
            self._neutral_controller()
            camera = state.get("camera") or {}
            if camera:
                # Apply the acknowledged camera without echoing another engine command.
                self.scene.yaw = float(camera.get("yaw", self.scene.yaw))
                self.scene.pitch = float(camera.get("pitch", self.scene.pitch))
                self.scene.distance = float(camera.get("distance", self.scene.distance))
                target = camera.get("target", self.scene.target)
                self.scene.target = list(target) if target is not None else [0., 0., 0.]
        if self._state:
            self._on_state(self._state)  # Refresh only from the authoritative live state.
        else:
            self._update_live_controls()
            self._refresh_replay_display()
        reason = str(state.get("reason") or "")
        if reason and reason != self._replay_reason:
            self._log("Replay: " + reason)
            self.statusBar().showMessage(reason)
        self._replay_reason = reason

    def _update_live_controls(self):
        resetting = bool(getattr(self, "_reset_pending", False))
        blocked = self._replay_blocked() or resetting
        self.reset_button.setText("Resetting…" if resetting else "Reset")
        if resetting:
            self.status_badge.setText("RESETTING")
            self.scene.status = "RESETTING"
        running, paused = bool(self._state.get("running")), bool(self._state.get("paused"))
        self.start_button.setEnabled(not blocked and not running and not self._building and self._built_workspace == self._workspace_key())
        self.pause_button.setEnabled(not blocked and running)
        self.pause_button.setText("Resume" if paused else "Pause")
        self.stop_button.setEnabled(not blocked and running)
        self.step_button.setEnabled(not blocked and (paused or not running))
        self.reset_button.setEnabled(not blocked and not self._building
                                     and not self._pending_scene_applies and not getattr(self, "_scene_apply_ready", False))
        self.apply_button.setEnabled(not blocked and not self._building)
        self.build_button.setEnabled(not blocked and not running and not self._building)
        self.demo_button.setEnabled(not blocked and not running and not self._building)
        self.joint_button.setEnabled(not blocked and not running)
        self.workspace.setEnabled(not blocked and not self._building)
        self.load_profile_button.setEnabled(not blocked and not self._building)
        self.save_profile_button.setEnabled(not blocked)
        for control in self.mode_buttons.values():
            control.setEnabled(not blocked and not running)
        for index in range(min(5, self.tabs.count())):
            self.tabs.widget(index).setEnabled(not blocked)
        self.scene.driving = not blocked

    def _refresh_replay_display(self):
        active = bool(self._replay.get("active"))
        pending = bool(self._replay_pending)
        available = bool(self._replay.get("available"))
        duration = max(0.0, float(self._replay.get("duration_s", 0)))
        position = max(0.0, min(duration, float(self._replay.get("position_s", 0))))
        count = int(self._replay.get("sample_count", 0))
        rate = float(self._replay.get("effective_hz", self._replay.get("recording_hz", 0)) or 0)
        self.replay_button.setEnabled(available and not active and not pending and not self._building and not self._pending_scene_applies and not getattr(self, "_reset_pending", False))
        self.replay_play_button.setEnabled(active and not pending and count > 1)
        self.replay_play_button.setText("Pause" if self._replay.get("playing") else "Play")
        self.replay_live_button.setEnabled(active and not pending)
        self.replay_timeline.setEnabled(active and not pending and count > 1)
        if not self._replay_dragging and self._pending_replay_seek is None:
            previous = self.replay_timeline.blockSignals(True)
            try:
                self.replay_timeline.setRange(0, max(0, round(duration * 1000)))
                self.replay_timeline.setValue(round(position * 1000))
                self.replay_timeline.setSingleStep(max(1, round(1000 / rate)) if rate > 0 else 33)
                self.replay_timeline.setPageStep(5000)
            finally:
                self.replay_timeline.blockSignals(previous)
            self.replay_time.setText(f"{_replay_time(position)} / {_replay_time(duration)}")
        title = next((title for key, title, *_ in MODES if key == self._replay.get("mode")), "Previous run")
        termination = str(self._replay.get("termination", "recorded")).replace("_", " ")
        resolution = f"{rate:g} samples/s" if rate > 0 else f"{count} recorded samples"
        description = f"{title} · {termination} · {resolution} · recorded motion"
        if active and self._replay.get("phase"):
            description += " · " + str(self._replay["phase"]).replace("_", " ")
        if self._replay.get("truncated"):
            description += " · truncated recording"
        if pending:
            description = "Preparing replay… Live physics is paused." if self._replay_pending == "enter" else "Returning to the paused live world…"
        elif self._replay.get("reason"):
            description = str(self._replay["reason"])
        elif not available:
            description = "No recorded run yet · completed and stopped runs become available here."
        self.replay_status.setText(description)
        self.replay_status.setToolTip(description)
        self.replay_button.setToolTip(description)
        self.progress.setVisible(not active)
        self.scene_subtitle.setText("/ recorded run" if active else "/ live world")
        self.telemetry_caption.setText("LIVE TELEMETRY · PAUSED DURING REPLAY" if active or pending else "LIVE ROBOT STATE")
        self.metric_captions["time"].setText("RECORDED TIME" if active else "SIMULATION TIME")
        self.metric_captions["remaining"].setText("RECORDED MODE TIME" if active else "TIME REMAINING")
        self.metric_captions["rtf"].setText("PLAYBACK" if active else "REAL-TIME FACTOR")
        if active:
            self.metric_values["time"].setText(_replay_time(position))
            self.metric_values["remaining"].setText(f"{float(self._replay.get('mode_time', 0)):.2f} s")
            self.metric_values["rtf"].setText("1×" if self._replay.get("playing") else "Paused")
            self.metric_values["rtf"].setToolTip("Recorded simulated-time playback; this is not live physics throughput.")
            x, y, *_ = self._pose_values(self._replay)
            self.metric_values["pose"].setText(f"{x:.2f} / {y:.2f} m")
            self.status_badge.setText("REPLAY")
            self.scene.status = "REPLAY · PLAYING" if self._replay.get("playing") else "REPLAY · PAUSED"
            self.scene.scene_name = str(self._replay.get("scene_name") or self._replay.get("field_name") or title)
            placed = self._replay.get("placed_pins")
            self.scene.score_text = f"RECORDED · {placed} pin placements" if placed is not None else ""
        elif pending:
            self.status_badge.setText("REPLAY LOADING")
            self.scene.status = "REPLAY LOADING"
        elif hasattr(self, "_live_scene_name"):
            self.scene.scene_name = self._live_scene_name
            del self._live_scene_name
        self.scene.update()

    def _on_state(self, state):
        # A prior scene can still emit state while a replacement is queued.
        # Clear only our loading message after all requested applies were
        # acknowledged and their following state reaches the GUI.
        if self._scene_apply_ready:
            self._scene_apply_ready = False
            if self.statusBar().currentMessage() == APPLY_STATUS:
                self.statusBar().clearMessage()
        self._state = state
        self._logo.observe(state)
        self._refresh_logo()
        running, paused = bool(state.get("running", False)), bool(state.get("paused", False))
        elapsed = float(state.get("time", 0))
        duration = float(state.get("duration", 0)) or self.duration_fields[self._mode].value()
        mode_time = float(state.get("mode_time", elapsed))
        remaining = float(state.get("remaining", max(0, duration - mode_time))) if state.get("duration", 0) else duration
        minutes, seconds = divmod(elapsed, 60)
        self.metric_values["time"].setText(f"{int(minutes):02d}:{seconds:06.3f}")
        self.metric_values["remaining"].setText(f"{max(0, remaining):.2f} s")
        factor = float(state.get("real_time_factor", 1))
        self.metric_values["rtf"].setText(f"{factor:.2f}×")
        self.metric_values["rtf"].setToolTip("Simulated seconds per wall-clock second. Values below 1 mean slower playback with unchanged physics steps.")
        x, y, z, yaw = self._pose_values(state)
        self.metric_values["pose"].setText(f"{x:.2f} / {y:.2f} m")
        self.pose_detail.setText(f"X {x:+.3f} m    Y {y:+.3f} m    Z {z:.3f} m\nHeading {yaw:.2f}°  ·  physics {self.config.get('physics_hz', 240)} Hz\nBacklog {float(state.get('physics_backlog_s', 0)):.3f} s  ·  sensor age {float(state.get('sensor_age_ms', 0)):.1f} ms")
        physics = state.get("physics") or {}
        backend = {"bullet": "Bullet", "physx": "PhysX"}.get(physics.get("backend"), "Physics")
        acceleration = "CUDA" if physics.get("hardware_accelerated") else "CPU"
        physics_text = (f"Physics: {backend} · {acceleration} · {physics.get('device', 'unknown')}"
                        if physics.get("backend") else "Physics initialization pending…")
        if physics.get("reason"):
            physics_text += "\n" + str(physics["reason"])
        self.physics_detail.setText(physics_text)
        self.physics_field_status.setText(physics_text)
        graphics = state.get("renderer") or {}
        device = graphics.get("device", "Graphics initialization pending")
        accelerated = bool(graphics.get("hardware_accelerated"))
        graphics_label = "GPU" if accelerated else "CPU" if graphics.get("backend") == "cpu" else "Graphics"
        reason = graphics.get("reason", "")
        self.renderer_detail.setText(f"Graphics: {graphics_label} · {device}"
                                     + f"\nLive rendered frames: {float(state.get('viewport_fps', 0)):.1f} FPS (wall time)"
                                     + (f"\n{reason}" if reason else ""))
        self.renderer_detail.setToolTip(str(graphics.get("version", "")))
        self.progress.setValue(round(min(1, mode_time / duration) * 1000) if duration else 0)
        status = "PID TUNING" if state.get("tuning_active") else "PAUSED" if paused else "RUNNING" if running else "SCENE READY"
        self.status_badge.setText(status)
        self.scene.status = status
        scoring = state.get("scoring") or {}
        placed = scoring.get("placed_pins") if isinstance(scoring, dict) else None
        if isinstance(placed, (list, tuple, set)):
            placed = len(placed)
        self.scene.score_text = f"PROTOTYPE  ·  {placed} pin placements" if placed is not None else ""
        self._update_live_controls()
        motors = state.get("motors", {})
        if isinstance(motors, dict):
            rows = [(port, values) for port, values in motors.items()]
        else:
            rows = [(values.get("port", index + 1), values) for index, values in enumerate(motors)]
        self.telemetry_table.setRowCount(len(rows))
        for row, (port, values) in enumerate(rows):
            if not isinstance(values, dict):
                continue
            position = float(values.get("position_deg", math.degrees(values.get("position", 0))))
            rpm = float(values.get("velocity_rpm", values.get("velocity", 0) * 60 / (2 * math.pi)))
            fields = [str(port), f"{position:.2f}", f"{rpm:.1f}", f"{float(values.get('torque', 0)):.3f}"]
            for col, value in enumerate(fields):
                item = self.telemetry_table.item(row, col)
                if item is None:
                    self.telemetry_table.setItem(row, col, QTableWidgetItem(value))
                else:
                    item.setText(value)
        self.scene.update()
        self._refresh_replay_display()

    def _log(self, message):
        if message == "Scene configuration applied" and self._pending_scene_applies:
            self._pending_scene_applies -= 1
            self._scene_apply_ready = self._pending_scene_applies == 0
        if hasattr(self, "console"):
            self.console.append(message)
        else:
            self.log_view.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _on_failed(self, message):
        was_building = self._building
        was_resetting = getattr(self, "_reset_pending", False)
        self._reset_pending = False
        self._pending_scene_applies = 0
        self._scene_apply_ready = False
        self._building = False
        self._logo_event("error")
        self.build_button.setEnabled(not self._state.get("running", False))
        self.build_button.setText("Build workspace")
        self.workspace.setEnabled(True)
        self.status_badge.setText("ATTENTION")
        if hasattr(self, "operation_progress"):
            self.operation_progress.clear()
        if was_building:
            title = "Workspace build failed"
            guidance = "Open the console for compiler errors. Check that the selected folder contains src and include, fix the reported source or host-compiler problem, then select Build workspace again."
        elif was_resetting:
            title = "Simulation reset failed"
            guidance = "Open the console for the reset error. Check the selected robot and field assets, then apply settings to reload the scene before retrying."
        else:
            title = "Simulation operation failed"
            guidance = "Open the console for details. Check the selected robot, field and physics backend, then apply settings to reload the scene."
        _local_feedback(self, title, guidance, details=str(message))
        self.statusBar().showMessage(str(message))
        self._update_live_controls()
        self._refresh_replay_display()

    def closeEvent(self, event):
        tuning_window = getattr(self, "_tuning_window", None)
        if tuning_window is not None and tuning_window.busy:
            self._tuning_close_pending = True
            tuning_window.close()
            event.ignore()
            return
        if tuning_window is not None:
            tuning_window.close()
        if hasattr(self, "feedback"):
            self.feedback.dismiss()
        if hasattr(self, "console"):
            self.console.errors.dismiss()
            self.console.hide()
        self._controller_timer.stop()
        self._replay_seek_timer.stop()
        self._pending_replay_seek = None
        self._neutral_controller()
        if self._pygame:
            self._pygame.quit()
        self.engine.close()
        event.accept()


def run(config: dict | None = None, workspace: str | None = None) -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    app.setApplicationName("EER SIM")
    app.setOrganizationName("WVU Robotics")
    app.setStyle("Fusion")
    app.setWindowIcon(QIcon(str(ASSETS / "logos" / "happy.png")))
    window = Workbench(config=config, workspace=workspace)
    hook = ExceptionFeedback(window.feedback)
    try:
        window.show()
        return app.exec()
    finally:
        hook.restore()
