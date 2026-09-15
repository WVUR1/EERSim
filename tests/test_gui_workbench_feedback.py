"""Main-window feedback wiring; Qt runs offscreen in an owned child process."""
from pathlib import Path
import os
import subprocess
import sys


QT_CHECK = r'''
import os
from pathlib import Path
from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtGui import QFontDatabase
from PySide6.QtWidgets import QApplication, QScrollArea, QToolButton
from eersim import gui, preview

class Engine(QObject):
    frame = Signal(object)
    replay_frame = Signal(object)
    replay_state = Signal(dict)
    state = Signal(dict)
    reset_completed = Signal(dict)
    operation = Signal(dict)
    log = Signal(str)
    built = Signal(str)
    failed = Signal(str)
    def __init__(self, **kwargs):
        super().__init__()
        self.calls = []
        self.closed = False
    def configure(self, *args, **kwargs): self.calls.append(('configure', args, kwargs))
    def set_camera(self, *args): pass
    def set_controller(self, *args): pass
    def close(self): self.closed = True

class Preview(QObject):
    frame = Signal(object)
    state = Signal(dict)
    failed = Signal(str)
    def __init__(self):
        super().__init__()
        self.closed = False
        self.calls = []
    def configure(self, *args): self.calls.append(('configure', args))
    def set_camera(self, *args): pass
    def stop_all(self): self.calls.append(('stop',))
    def set_motor(self, *args): self.calls.append(('motor', args))
    def close(self): self.closed = True

gui.SimulationEngine = Engine
preview.UrdfPreview = Preview
app = QApplication([])
# Windows' offscreen Qt plugin does not discover system fonts automatically.
if os.environ.get('WINDIR'):
    font = Path(os.environ['WINDIR']) / 'Fonts' / 'segoeui.ttf'
    if font.is_file():
        QFontDatabase.addApplicationFont(str(font))
app.setStyle('Fusion')
window = gui.Workbench()
window.resize(1540, 1040)
window.show()
app.processEvents()
try:
    assert window.engine.calls and not window.console.isVisible()
    assert not window.log_view.isVisible()
    assert window.log_view.parentWidget() is window.console.widget()
    window._log('Build detail retained while the console is hidden')
    assert window.console._pending
    window.console.toggleViewAction().trigger()
    app.processEvents()
    assert window.console.isVisible()
    assert 'Build detail retained' in window.console.editor.toPlainText()
    window.console.setFloating(True)
    app.processEvents()
    assert window.console.isFloating()
    window.console.setFloating(False)
    window.console.hide()

    controls = [window.workspace, window.urdf_path, window.binding_path, window.game_path,
                window.field_preset, window.camera_preset, window.joint_select, window.joint_target,
                window.physics_select, window.physics_device, window.physics_iterations,
                window.physics_cpu_threads, window.physics_hz, window.render_fps,
                window.renderer_select, window.gravity, window.friction, window.plastic_friction,
                window.restitution, window.seed, *window.pose_fields,
                *window.noise_fields.values(), *window.duration_fields.values()]
    for control in controls:
        assert control.property('settingHelp'), control.objectName() or type(control).__name__
        assert control.accessibleDescription()
    assert len(window.findChildren(QToolButton, 'SettingHelp')) >= len(controls)
    for col in range(window.motor_table.columnCount()):
        header = window.motor_table.horizontalHeaderItem(col)
        assert header.toolTip() and not header.icon().isNull()

    window.tabs.setCurrentIndex(2)
    window.progress.setValue(370)
    window.engine.operation.emit({'id': 1, 'action': 'configure', 'state': 'running',
                                  'message': 'Loading robot and field collision geometry…', 'fraction': None})
    app.processEvents()
    assert window.operation_progress.isVisible() and window.operation_progress.bar.maximum() == 0
    assert window.progress.value() == 370
    window.resize(1120, 700)
    app.processEvents()
    assert window.minimumSizeHint().width() <= 1120, window.minimumSizeHint()
    assert window.minimumSizeHint().height() <= 700, window.minimumSizeHint()
    assert window.width() == 1120 and window.height() == 700
    screenshot = os.environ.get('EERSIM_FEEDBACK_SCREENSHOT')
    if screenshot:
        window.resize(1540, 1040)
        app.processEvents()
        page = window.tabs.currentWidget()
        if isinstance(page, QScrollArea):
            page.ensureWidgetVisible(window.physics_select, 0, 60)
        else:
            scroll = page.findChild(QScrollArea)
            if scroll:
                scroll.ensureWidgetVisible(window.physics_select, 0, 60)
        app.processEvents()
        assert window.grab().save(screenshot)
    window.engine.operation.emit({'id': 1, 'action': 'configure', 'state': 'completed',
                                  'message': 'Scene ready', 'fraction': 1.0})
    assert window.operation_progress.isHidden() and window.progress.value() == 370

    # An export/asset replay failure must not invalidate a live run or workspace build.
    window._state = {'running': True}
    window._building = True
    window._pending_scene_applies = 2
    window.engine.operation.emit({'id': 2, 'action': 'export', 'state': 'failed',
                                  'message': 'Export failed', 'error': 'Permission denied: telemetry.csv'})
    box = window.feedback._dialog
    assert box is not None and not box.isModal()
    assert 'writable' in box.text() and 'Permission denied' in box.detailedText()
    assert window._building and window._pending_scene_applies == 2 and window._state['running']
    assert window.tabs.currentIndex() == 2
    window.feedback.dismiss()
    window.engine.operation.emit({'id': 3, 'action': 'replay_enter', 'state': 'failed',
                                  'message': 'Replay failed', 'error': 'Missing mesh'})
    assert 'Missing mesh' in window.feedback._dialog.detailedText()
    assert window._building and window._pending_scene_applies == 2
    window.feedback.dismiss()

    # Actual build failures keep the compiler output in a nonmodal, actionable dialog.
    window._state = {}
    window.engine.failed.emit('Host compiler was not found')
    box = window.feedback._dialog
    assert box is not None and not box.isModal()
    assert box.windowTitle() == 'Workspace build failed'
    assert 'compiler' in box.text() and 'Host compiler' in box.detailedText()
    assert window.tabs.currentIndex() == 2 and not window._building
    window.feedback.dismiss()

    diag = gui.DiagnosticsExportDialog(str(Path.cwd()), parent=window)
    diag.show()
    for control in (diag.destination, diag.mode, diag.wall_seconds, diag.compare_rates, diag.include_rendering):
        assert control.property('settingHelp')
    diag.destination.setText('')
    diag._accept_destination()
    assert diag.feedback._dialog is not None
    diag.reject()
    assert diag.feedback._dialog is None

    robot = gui.RobotImportDialog({'name': 'Test mechanism', 'joints': [
        {'name': 'hinge', 'type': 'revolute'}, {'name': 'slide', 'type': 'prismatic'}]},
        {'robot_urdf': 'test.urdf', 'motors': [{'port': 1, 'joint': 'hinge', 'rpm': 200},
                                            {'port': 2, 'joint': 'slide', 'rpm': 100}]}, parent=window)
    robot.show()
    app.processEvents()
    assert robot.preview.calls and robot.preview_progress.isVisible()
    for col in range(robot.table.columnCount()):
        header = robot.table.horizontalHeaderItem(col)
        assert header.toolTip() and not header.icon().isNull()
    assert robot.jog_voltage.property('settingHelp')
    assert robot._lead_fields[1].property('settingHelp')
    robot.preview.failed.emit('Mesh file is missing')
    assert robot.feedback._dialog is not None and not robot.feedback._dialog.isModal()
    assert robot.preview_progress.isHidden()
    robot.reject()
    assert robot.preview.closed and robot.feedback._dialog is None
    window.console.setFloating(True)
    window.console.show()
    app.processEvents()
    assert window.console.isVisible() and window.console.isFloating()
finally:
    window.close()
    app.processEvents()
assert window.engine.closed
assert all(not item.isVisible() for item in app.topLevelWidgets())
print('MAIN_FEEDBACK_CHECK_OK')
'''


def test_main_feedback_lifecycle_settings_and_errors():
    environment = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    result = subprocess.run([sys.executable, "-c", QT_CHECK], env=environment,
                            cwd=Path(__file__).resolve().parents[1],
                            capture_output=True, text=True, timeout=90)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "MAIN_FEEDBACK_CHECK_OK" in result.stdout
