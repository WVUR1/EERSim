"""Desktop autonomy workbench. Physics and native robot code run in the engine.

The GUI never advances physics or builds C++ itself. It sends commands to the
engine, and displays copied camera frames and state delivered through Qt signals.
"""

from __future__ import annotations

import copy
import importlib
import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from PySide6.QtCore import QEvent, QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QImage, QKeyEvent, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication, QButtonGroup, QComboBox, QDoubleSpinBox, QDialog, QDialogButtonBox,
    QFileDialog, QFormLayout, QFrame, QGridLayout, QHBoxLayout, QHeaderView,
    QLabel, QLineEdit, QMainWindow, QPlainTextEdit, QProgressBar,
    QPushButton, QScrollArea, QSizePolicy, QSpinBox, QSplitter,
    QTabWidget, QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from .config import ASSETS, default_config
from .engine import SimulationEngine
from .gui_styles import GOLD, MUTED, STYLESHEET
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
APPLY_STATUS = "Applying experiment settings and resetting physics…"


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


class RobotImportDialog(QDialog):
    """Map a standard URDF to V5 hardware without hand-writing a sidecar."""

    def __init__(self, inspection: dict, patch: dict, parent=None):
        super().__init__(parent)
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
        self.table.verticalHeader().hide()
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeMode.Stretch)
        for col, width in ((1, 78), (2, 164), (3, 77), (4, 92), (5, 55)):
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
            self.table.setItem(index, 0, item)
            port = QSpinBox()
            port.setRange(0, 21)
            port.setSpecialValueText("Passive")
            signed_port = int(motor.get("port", 0))
            port.setValue(abs(signed_port))
            self.table.setCellWidget(index, 1, port)
            cartridge = cartridge_select(motor)
            self.table.setCellWidget(index, 2, cartridge)
            ratio = real_spin(0.001, 10000, float(motor.get("gear_ratio", 1)), 3)
            self.table.setCellWidget(index, 3, ratio)
            if self._joint_types.get(motor.get("joint")) == "prismatic":
                lead = real_spin(.01, 10000, float(motor.get("lead_m", .01)) * 1000, 2)
                lead.setToolTip("Linear travel per motor output revolution, before the external gear ratio.")
                self._lead_fields[index] = lead
                self.table.setCellWidget(index, 4, lead)
            else:
                unused = label("—", "Muted")
                unused.setAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setCellWidget(index, 4, unused)
            reverse = QTableWidgetItem()
            reverse.setFlags(Qt.ItemFlag.ItemIsUserCheckable | Qt.ItemFlag.ItemIsEnabled)
            reverse.setCheckState(Qt.CheckState.Checked if motor.get("reversed", signed_port < 0) else Qt.CheckState.Unchecked)
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

    def _preview_app_state(self, state):
        if state != Qt.ApplicationState.ApplicationActive:
            self._stop_preview()

    def eventFilter(self, watched, event):
        if event.type() in (QEvent.Type.WindowDeactivate, QEvent.Type.FocusOut):
            self._stop_preview()
        return super().eventFilter(watched, event)

    def done(self, result):
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
        painter.drawText(28, 55, "CONTACT DYNAMICS  /  SI UNITS  /  " + self.status)
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
        self._loaded_urdf_path = self.config.get("robot_urdf", "")
        if workspace is not None:
            self.config["workspace"] = str(workspace)
        self.engine = SimulationEngine(defer_configuration=True)
        self._state: dict = {}
        self._built_workspace: str | None = None
        self._pending_workspace: str | None = None
        self._building = False
        self._pending_scene_applies = 0
        self._scene_apply_ready = False
        self._mode = "skills_auto"
        self._keys: set[int] = set()
        self._controller = {"analog": {name: 0 for name in ANALOG_NAMES},
                            "digital": {name: False for name in DIGITAL_NAMES}}
        self._pygame = self._gamepad = None
        self._joint_metadata: dict[str, dict] = {}
        self.setWindowTitle("EER SIM · WVU Robotics Autonomy Workbench")
        fit_window(self, 1500, 950)
        self.setStyleSheet(STYLESHEET)
        root = QWidget()
        root.setObjectName("Root")
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        layout.setContentsMargins(18, 14, 18, 6)
        layout.setSpacing(12)
        self._make_header(layout, workspace)
        split = QSplitter(Qt.Orientation.Horizontal)
        split.setHandleWidth(12)
        split.setChildrenCollapsible(False)
        split.addWidget(self._make_scene())
        split.addWidget(self._make_settings())
        split.setSizes([910, 520])
        layout.addWidget(split, 1)
        layout.addWidget(self._make_controls())
        self.statusBar().showMessage("Fixed physics steps preserve simulation fidelity when rendering slows.")
        self.engine.frame.connect(self.scene.set_frame)
        self.engine.state.connect(self._on_state)
        self.engine.log.connect(self._log)
        self.engine.built.connect(self._on_built)
        self.engine.failed.connect(self._on_failed)
        self.scene.camera_changed.connect(self.engine.set_camera)
        self.scene.key_changed.connect(self._key)
        self.scene.focus_lost.connect(self._release_keys)
        self._fill_widgets(self.config)
        self._controller_timer = QTimer(self)
        self._controller_timer.timeout.connect(self._poll_gamepad)
        self._controller_timer.start(25)
        QApplication.instance().applicationStateChanged.connect(self._application_state)
        QTimer.singleShot(0, self._apply_config)

    def _make_header(self, parent, workspace):
        header = QHBoxLayout()
        identity = QVBoxLayout()
        identity.setSpacing(0)
        identity.addWidget(label("EER / SIM", "Brand"))
        identity.addWidget(label("WVU ROBOTICS  ·  AUTONOMY LAB", "BrandSub"))
        header.addLayout(identity)
        header.addSpacing(24)
        header.addWidget(label("PROS WORKSPACE", "Section"))
        self.workspace = QLineEdit(str(workspace or self.config.get("workspace") or ""))
        self.workspace.setPlaceholderText("Choose a PROS project folder…")
        self.workspace.setMinimumWidth(240)
        self.workspace.textChanged.connect(self._workspace_changed)
        header.addWidget(self.workspace, 1)
        header.addWidget(button("Browse…", self._choose_workspace))
        self.build_button = button("Build workspace", self._build, "Primary")
        header.addWidget(self.build_button)
        self.status_badge = label("LOADING WORLD", "Badge")
        header.addWidget(self.status_badge)
        parent.addLayout(header)

    def _make_scene(self):
        pane = QWidget()
        layout = QVBoxLayout(pane)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(9)
        top = QHBoxLayout()
        top.addWidget(label("Simulation", "Title"))
        top.addWidget(label("/ live world", "Muted"))
        top.addStretch()
        self.camera_preset = QComboBox()
        self.camera_preset.addItems(["Orbit camera", "Top down", "Driver station", "Close view"])
        self.camera_preset.currentIndexChanged.connect(self._camera_preset)
        top.addWidget(self.camera_preset)
        layout.addLayout(top)
        self.scene = SceneView()
        layout.addWidget(self.scene, 1)
        metrics = QHBoxLayout()
        metrics.setSpacing(9)
        self.metric_values = {}
        for key, caption, initial in (("time", "SIMULATION TIME", "00:00.000"), ("remaining", "TIME REMAINING", "60.00 s"),
                                      ("rtf", "REAL-TIME FACTOR", "1.00×"), ("pose", "ROBOT POSITION", "0.00 / 0.00 m")):
            card = QFrame()
            card.setObjectName("Metric")
            box = QVBoxLayout(card)
            box.setContentsMargins(12, 9, 12, 9)
            box.setSpacing(2)
            box.addWidget(label(caption, "MetricLabel"))
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
        layout.addWidget(self.progress)
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
        heading.addWidget(button("Load", self._load_profile))
        heading.addWidget(button("Save", self._save_profile))
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
        self.apply_button = button("Apply settings & reset scene", self._apply_config)
        self.apply_button.setToolTip("Rebuilds the physics scene with these settings; ends the current run.")
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
        self.demo_button.setToolTip("Loads the example X-drive claw robot, current game field and PROS workspace, then builds the program.")
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
        row.itemAt(1).widget().setFixedWidth(118)
        row.itemAt(1).widget().setText("Import URDF…")
        layout.addLayout(row)
        layout.addWidget(button("Preview robot & test motors", self._edit_robot_setup))
        self.urdf_path.editingFinished.connect(lambda: self._scan_joints(self.urdf_path.text()))
        layout.addWidget(label("Import a standard URDF, then assign V5 ports and cartridges in the setup dialog. Passive joints need no port.", "Muted", True))
        layout.addWidget(label("Optional advanced hardware bindings", "Muted"))
        row, self.binding_path = self._file_row("hardware bindings", "Hardware bindings (*.json)", self._bindings_selected)
        self.binding_path.setPlaceholderText("Optional · motor assignments below work without a JSON file")
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
        self.motor_table.horizontalHeaderItem(5).setToolTip("Cartridge output shaft stall torque. Joint stall torque scales by external ratio × efficiency; linear joints also use their configured lead.")
        self.motor_table.verticalHeader().hide()
        self.motor_table.setAlternatingRowColors(True)
        self.motor_table.setMinimumHeight(177)
        self.motor_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeMode.Stretch)
        for col, width in ((0, 40), (2, 37), (3, 156), (4, 43), (5, 55)):
            self.motor_table.setColumnWidth(col, width)
        layout.addWidget(self.motor_table)
        row = QHBoxLayout()
        row.addWidget(button("+ Motor", self._add_motor))
        row.addWidget(button("Remove", self._remove_motor))
        row.addStretch()
        layout.addLayout(row)
        layout.addWidget(label("Cartridges set physical gearing and shaft stall torque. Ratio is external motor turns per joint turn. Edit shaft N·m for measured calibration; changing cartridge restores its default.", "Muted", True))
        layout.addWidget(label("MANUAL JOINT CONTROL", "Section"))
        form = QFormLayout()
        self.joint_select = QComboBox()
        self.joint_select.currentTextChanged.connect(self._joint_changed)
        form.addRow("Joint", self.joint_select)
        self.joint_target = real_spin(-10000, 10000, 0, 3, " rad")
        form.addRow("Target", self.joint_target)
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
        layout.addWidget(self.field_preset)
        self.game_title = label("Practice field", "Title", True)
        layout.addWidget(self.game_title)
        self.game_description = label("", "Muted", True)
        layout.addWidget(self.game_description)
        layout.addWidget(label("Choose the fixed-piece route-planning preset for fast path iteration. Full game-piece dynamics support grasping and stacking, and may run substantially below real time.", "Muted", True))
        layout.addWidget(label("Import a game pack JSON or a standard field URDF. URDF imports become static field geometry automatically. Game packs describe their visual sources and collision approximations; official scoring and referee rules are not simulated.", "Muted", True))
        layout.addWidget(label("PHYSICS & RENDERING", "Section"))
        form = QFormLayout()
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
        self.renderer_select.setToolTip("GPU accelerates the viewport. Physics still uses the configured fixed step. Unsupported graphics fall back to CPU; Telemetry shows the active device and reason.")
        self.gravity = real_spin(0, 30, 9.80665, 5, " m/s²")
        self.friction = real_spin(0, 5, 0.9, 3)
        self.friction.setToolTip("Body coefficient for wheels and field tiles. Bullet multiplies the coefficients of the two touching bodies. Adjust from measured hardware behavior.")
        self.plastic_friction = real_spin(0, 5, 0.5, 3)
        self.plastic_friction.setToolTip("Body coefficient for pins, cups, goals, loaders and toggles. Bullet multiplies the two body coefficients: defaults give plastic/plastic μ = 0.25 and plastic/tile μ = 0.45. These are calibration assumptions, not official measured VEX values. Per-object game-pack overrides take precedence.")
        self.restitution = real_spin(0, 1, 0.05, 3)
        form.addRow("Fixed physics rate", self.physics_hz)
        form.addRow("Viewport frame rate", self.render_fps)
        form.addRow("Graphics renderer", self.renderer_select)
        form.addRow("Gravity", self.gravity)
        form.addRow("Wheel / tile grip", self.friction)
        form.addRow("Plastic contact friction", self.plastic_friction)
        form.addRow("Restitution", self.restitution)
        layout.addLayout(form)
        layout.addWidget(label("Physics uses a fixed time step, independent of viewport FPS. On slower hardware the simulation takes longer in wall time; it keeps every physics step. Real-time factor shows this slowdown.", "Muted", True))
        layout.addStretch()

    def _pose_tab(self):
        layout = self._tab("Start pose")
        layout.addWidget(label("INITIAL CONDITIONS", "Section"))
        layout.addWidget(label("Start each run from the same state.", "Title", True))
        layout.addWidget(label("Coordinates are meters from field center. +Z is up; heading rotates counterclockwise about +Z. Z positions the URDF base link origin.", "Muted", True))
        form = QFormLayout()
        self.pose_fields = [real_spin(-20, 20, 0, 3, " m"), real_spin(-20, 20, 0, 3, " m"),
                            real_spin(0, 10, 0.15, 3, " m"), real_spin(-3600, 3600, 0, 2, " °")]
        for title, field in zip(("X", "Y", "Z", "Heading"), self.pose_fields):
            form.addRow(title, field)
        layout.addLayout(form)
        row = QHBoxLayout()
        row.addWidget(button("Field center", lambda: self._pose_preset(0, 0, 0)))
        row.addWidget(button("Red side", lambda: self._pose_preset(-1.15, -1.15, 45)))
        row.addWidget(button("Blue side", lambda: self._pose_preset(1.15, 1.15, 225)))
        layout.addLayout(row)
        layout.addWidget(button("Use current robot pose", self._use_current_pose))
        layout.addWidget(label("REPRODUCIBILITY", "Section"))
        form = QFormLayout()
        self.seed = QSpinBox()
        self.seed.setRange(0, 2147483647)
        form.addRow("Noise random seed", self.seed)
        layout.addLayout(form)
        layout.addWidget(label("Reset restores the scene, start pose and seeded sensor noise. Save an experiment profile to share the complete setup with your team.", "Muted", True))
        layout.addWidget(label("RUN DURATION", "Section"))
        layout.addWidget(label("Defaults follow VEX U periods: 60 s skills, 30 s match autonomous, 90 s driver. Adjust below for drills or other formats.", "Muted", True))
        form = QFormLayout()
        self.duration_fields = {}
        for mode, title, _, duration in MODES:
            field = real_spin(0.1, 3600, duration, 1, " s")
            self.duration_fields[mode] = field
            field.valueChanged.connect(self._duration_changed)
            form.addRow(title, field)
        layout.addLayout(form)
        layout.addStretch()

    def _sensors_tab(self):
        layout = self._tab("Sensors")
        layout.addWidget(label("ROBUSTNESS LAB", "Section"))
        layout.addWidget(label("Find the edge of your autonomy.", "Title", True))
        layout.addWidget(label("Inject seeded Gaussian measurement noise, sensor latency and dropped updates. Ground-truth pose in the viewport stays unmodified.", "Muted", True))
        form = QFormLayout()
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
            form.addRow(title, field)
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
        layout.addWidget(label("LIVE ROBOT STATE", "Section"))
        self.pose_detail = label("Waiting for physics state…", "Muted", True)
        layout.addWidget(self.pose_detail)
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
        heading = QHBoxLayout()
        heading.addWidget(label("BUILD & RUNTIME LOG", "Section"))
        heading.addStretch()
        heading.addWidget(button("Clear", lambda: self.log_view.clear()))
        layout.addLayout(heading)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(3000)
        self.log_view.setMinimumHeight(240)
        layout.addWidget(self.log_view, 1)

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
        self.step_button = button("Step", self.engine.single_step)
        self.step_button.setToolTip("Advance exactly one fixed physics step while paused.")
        self.reset_button = button("Reset", self._reset)
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

    def _apply_config(self):
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
            self.engine.configure(config)
            self.engine.set_camera(self.scene.yaw, self.scene.pitch, self.scene.distance, self.scene.target)
            self.statusBar().showMessage(APPLY_STATUS)
            return True
        except (ValueError, OSError, TypeError) as exc:
            self._on_failed(str(exc))
            return False

    def _choose_workspace(self):
        path = QFileDialog.getExistingDirectory(self, "Select a PROS workspace", self.workspace.text())
        if path:
            self.workspace.setText(str(Path(path)))

    def _workspace_key(self):
        text = self.workspace.text().strip()
        return str(Path(text).expanduser().resolve()) if text else ""

    def _workspace_changed(self):
        if hasattr(self, "start_button"):
            self.start_button.setEnabled(self._built_workspace == self._workspace_key() and not self._state.get("running", False))
            if self._built_workspace != self._workspace_key():
                self.build_status.setText("Workspace changed · build before running")

    def _build(self):
        key = self._workspace_key()
        if not key:
            self._on_failed("Select a PROS workspace containing a src folder.")
            return
        workspace = Path(key)
        if not workspace.is_dir() or not (workspace / "src").is_dir():
            self._on_failed("Select a PROS workspace containing a src folder.")
            return
        self._building = True
        self._built_workspace = None
        self._pending_workspace = str(workspace)
        self.build_button.setEnabled(False)
        self.build_button.setText("Building…")
        self.workspace.setEnabled(False)
        self.start_button.setEnabled(False)
        self.build_status.setText("Compiling workspace C++ for the host simulator…")
        self.tabs.setCurrentIndex(5)
        self.engine.build(str(workspace))

    def _on_built(self, path):
        self._building = False
        self._built_workspace = self._pending_workspace
        self.build_button.setEnabled(True)
        self.build_button.setText("Rebuild workspace")
        self.workspace.setEnabled(True)
        self.start_button.setEnabled(not self._state.get("running", False) and self._built_workspace == self._workspace_key())
        self.start_button.setToolTip("Start the selected competition period.")
        self.build_status.setText("Native program ready · " + Path(str(path)).name)
        self.build_status.setToolTip(str(path))
        self._log("Build complete. Select a competition mode and press Start.")
        self.statusBar().showMessage("Workspace built · ready to run.")

    def _start(self):
        if self._built_workspace != self._workspace_key():
            self._on_failed("Build the selected workspace before starting.")
            return
        # Applying the visible settings is part of starting a repeatable run.
        if self._apply_config():
            self.engine.start(self._mode)
            self.scene.setFocus()

    def _pause_resume(self):
        if self._state.get("paused", False):
            self.engine.resume()
            self.scene.setFocus()
        else:
            self._neutral_controller()
            self.engine.pause()

    def _stop(self):
        self._neutral_controller()
        self.engine.stop()

    def _reset(self):
        self._neutral_controller()
        self.engine.reset()

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
        self.motor_table.setCellWidget(row, 1, joint)
        cartridge = cartridge_select(motor)
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
                self._on_failed(f"Cannot read hardware bindings: {exc}")

    def _urdf_selected(self, path, current=False):
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
            self._on_failed(f"Could not import robot: {exc}")

    def _accept_robot_import(self, patch, inspection):
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
        self.statusBar().showMessage("Robot imported. Adjust start height and click Apply settings to load it into the field.")

    def _edit_robot_setup(self):
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
        joint = self.joint_select.currentText()
        if joint:
            self.engine.set_joint(joint, self.joint_target.value())

    def _robot_preset(self, name):
        self.urdf_path.setText(str(ASSETS / "robots" / f"{name}.urdf"))
        self._loaded_urdf_path = self.urdf_path.text()
        self.binding_path.setText(str(ASSETS / "robots" / f"{name}.json"))
        self._scan_joints(self.urdf_path.text())
        self._bindings_selected(self.binding_path.text())

    def _game_preset(self, name):
        self.config.pop("field_objects", None)
        self.game_path.setText(str(ASSETS / "games" / name))
        self._describe_game(self.game_path.text())

    def _import_field(self, path):
        try:
            from .importers import default_import_directory, import_field
            destination = default_import_directory(self.workspace.text().strip() or None)
            imported = import_field(path, dest=destination if Path(path).suffix.lower() == ".urdf" else None)
            self.config.pop("field_objects", None)
            self.game_path.setText(str(imported["path"]))
            self.field_preset.setCurrentIndex(0)
            self._describe_game(self.game_path.text())
            for warning in imported.get("metadata", {}).get("warnings", []):
                self._log("Field import: " + str(warning))
            self.statusBar().showMessage("Field imported. Click Apply settings to load its geometry.")
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.game_path.setText(str(self.config.get("game_pack", "")))
            self._on_failed(f"Could not import field: {exc}")

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
        target = self._pose_values(self._state)[:3] if index == 3 else [0, 0, 0]
        self.scene.set_camera(*((45, -52, 6.4), (0, -89, 5), (90, -24, 6.4), (45, -30, 1.3))[index], target=target)

    def _save_profile(self):
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
        except (OSError, ValueError) as exc:
            self._on_failed(str(exc))

    def _load_profile(self):
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
            self._on_failed(f"Could not load experiment profile: {exc}")

    def _load_demo(self):
        try:
            from .demos import claw_demo_config
            config = claw_demo_config()
            self._fill_widgets(config)
            self.config = config
            if self._apply_config():
                self._select_mode("skills_auto")
                self.mode_buttons["skills_auto"].setChecked(True)
                self._build()
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self._on_failed(f"Could not load the claw scoring demo: {exc}")

    def _export_csv(self):
        try:
            initial = _dialog_default("telemetry.csv", "telemetry", getattr(self, "_telemetry_path", None),
                                      self.workspace.text().strip() or None)
            path, _ = QFileDialog.getSaveFileName(self, "Export run telemetry", initial, "CSV data (*.csv)")
            if path:
                self._telemetry_path = path
                self.engine.export_csv(path)
        except OSError as exc:
            self._on_failed(str(exc))

    def _key(self, key, down):
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
        self._controller["analog"][side + "_x"] = x
        self._controller["analog"][side + "_y"] = y
        self._send_controller()

    def _digital(self, name, down):
        self._controller["digital"][name] = down
        self._send_controller()

    def _send_controller(self):
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
                return
            self._gamepad = self._pygame.joystick.Joystick(0)
            self._gamepad.init()
            self.gamepad_status.setText(self._gamepad.get_name() + " · standard SDL axes/buttons")
            self.gamepad_button.setText("Disconnect gamepad")
        except ImportError:
            self.gamepad_status.setText('Gamepad support needs pygame. Install the simulator with its [gamepad] extra.')
        except Exception as exc:
            self.gamepad_status.setText(f"Gamepad connection failed: {exc}")

    def _poll_gamepad(self):
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

    def _on_state(self, state):
        # A prior scene can still emit state while a replacement is queued.
        # Clear only our loading message after all requested applies were
        # acknowledged and their following state reaches the GUI.
        if self._scene_apply_ready:
            self._scene_apply_ready = False
            if self.statusBar().currentMessage() == APPLY_STATUS:
                self.statusBar().clearMessage()
        self._state = state
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
        graphics = state.get("renderer") or {}
        device = graphics.get("device", "Graphics initialization pending")
        accelerated = bool(graphics.get("hardware_accelerated"))
        graphics_label = "GPU" if accelerated else "CPU" if graphics.get("backend") == "cpu" else "Graphics"
        reason = graphics.get("reason", "")
        self.renderer_detail.setText(f"{graphics_label} · {device}" + (f"\n{reason}" if reason else ""))
        self.renderer_detail.setToolTip(str(graphics.get("version", "")))
        self.progress.setValue(round(min(1, mode_time / duration) * 1000) if duration else 0)
        status = "PAUSED" if paused else "RUNNING" if running else "SCENE READY"
        self.status_badge.setText(status)
        self.scene.status = status
        scoring = state.get("scoring") or {}
        placed = scoring.get("placed_pins") if isinstance(scoring, dict) else None
        if isinstance(placed, (list, tuple, set)):
            placed = len(placed)
        self.scene.score_text = f"PROTOTYPE  ·  {placed} pin placements" if placed is not None else ""
        self.start_button.setEnabled(not running and not self._building and self._built_workspace == self._workspace_key())
        self.pause_button.setEnabled(running)
        self.pause_button.setText("Resume" if paused else "Pause")
        self.stop_button.setEnabled(running)
        self.step_button.setEnabled(paused or not running)
        self.build_button.setEnabled(not running and not self._building)
        self.demo_button.setEnabled(not running and not self._building)
        self.joint_button.setEnabled(not running)
        for control in self.mode_buttons.values():
            control.setEnabled(not running)
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

    def _log(self, message):
        if message == "Scene configuration applied" and self._pending_scene_applies:
            self._pending_scene_applies -= 1
            self._scene_apply_ready = self._pending_scene_applies == 0
        self.log_view.appendPlainText(f"[{time.strftime('%H:%M:%S')}] {message}")

    def _on_failed(self, message):
        self._pending_scene_applies = 0
        self._scene_apply_ready = False
        self._building = False
        self.build_button.setEnabled(not self._state.get("running", False))
        self.build_button.setText("Build workspace")
        self.workspace.setEnabled(True)
        self.status_badge.setText("ATTENTION")
        self._log("ERROR: " + str(message))
        self.tabs.setCurrentIndex(5)
        self.statusBar().showMessage(str(message))

    def closeEvent(self, event):
        self._controller_timer.stop()
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
    window = Workbench(config=config, workspace=workspace)
    window.show()
    return app.exec()
