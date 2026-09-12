"""Exercise GUI export orchestration without an application, window, or engine."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

from eersim import gui


class Text:
    def __init__(self):
        self.value = ''

    def setText(self, value):
        self.value = value

    def showMessage(self, value):
        self.value = value


class ForbiddenEngine:
    def __getattr__(self, name):
        raise AssertionError(f'Diagnostic export must not call the live engine: {name}')


def fixture(monkeypatch, tmp_path, *, accepted=True, export_error=None):
    calls, logs, seen = [], [], {}
    seen['existing_application'] = gui.QApplication.instance()
    shown = {'physics_backend': 'physx_gpu', 'physics_hz': 240, 'workspace': str(tmp_path / 'chosen PROS'),
             'noise': {'encoder_std_deg': 1.5}, 'motors': [{'port': 6, 'joint': 'claw'}]}
    applied = {'physics_backend': 'bullet', 'noise': {'encoder_std_deg': 0}}
    options = {'destination': tmp_path, 'mode': 'driver_auto', 'wall_seconds': 180,
               'compare_rates': False, 'include_rendering': False}
    output = tmp_path / 'unique-diagnostics-folder'

    class Dialog:
        def __init__(self, destination, mode, parent):
            seen.update(destination=destination, mode=mode, parent=parent)

        def exec(self):
            return gui.QDialog.DialogCode.Accepted if accepted else gui.QDialog.DialogCode.Rejected

        def options(self):
            return dict(options)

    def exporter(config, **kwargs):
        calls.append((deepcopy(config), kwargs))
        if export_error:
            raise export_error
        output.mkdir()
        (output / 'run-diagnostics.ps1').write_text('# generated script, deliberately not executed')
        return output

    def collect():
        seen['collected'] = True
        return deepcopy(shown)

    status = Text()
    workbench = SimpleNamespace(config=deepcopy(applied), _mode='match_auto', _diagnostics_directory=str(tmp_path),
        _collect_config=collect, engine=ForbiddenEngine(), _state={'running': True}, _building=True,
        diagnostics_status=Text(), _log=logs.append, statusBar=lambda: status, _logo_event=lambda _: None)
    monkeypatch.setattr(gui, 'DiagnosticsExportDialog', Dialog)
    monkeypatch.setitem(sys.modules, 'eersim.diagnostics_export', SimpleNamespace(export_diagnostics=exporter))
    return workbench, calls, logs, seen, shown, applied, options, output


def test_export_uses_unapplied_controls_current_mode_and_selected_run_options(monkeypatch, tmp_path):
    workbench, calls, logs, seen, shown, applied, options, output = fixture(monkeypatch, tmp_path)
    gui.Workbench._export_diagnostics(workbench)
    assert seen['mode'] == 'match_auto' and seen['destination'] == str(tmp_path)
    assert calls == [(shown, options)]
    assert workbench.config == applied
    assert workbench._building is True and workbench._state == {'running': True}
    assert output.is_dir() and str(output) in workbench.diagnostics_status.value
    if sys.platform == 'win32':
        assert 'run-diagnostics.ps1' in workbench.diagnostics_status.value
    else:
        assert 'instructions' in workbench.diagnostics_status.value
    assert logs and gui.QApplication.instance() is seen['existing_application']


def test_cancel_does_not_collect_write_or_touch_running_simulation(monkeypatch, tmp_path):
    workbench, calls, logs, seen, *_ = fixture(monkeypatch, tmp_path, accepted=False)
    gui.Workbench._export_diagnostics(workbench)
    assert not calls and not logs and 'collected' not in seen
    assert not list(tmp_path.iterdir())
    assert workbench._building is True and workbench._state['running'] is True


@pytest.mark.parametrize('error', [ValueError('Selected robot is invalid'), OSError('Output folder is not writable')])
def test_export_failure_stays_visible_without_resetting_engine_or_build_state(monkeypatch, tmp_path, error):
    workbench, calls, logs, _, _, applied, _, output = fixture(monkeypatch, tmp_path, export_error=error)
    gui.Workbench._export_diagnostics(workbench)
    assert calls and not output.exists()
    assert str(error) in workbench.diagnostics_status.value
    assert logs[-1].startswith('ERROR:')
    assert workbench.config == applied and workbench._building is True and workbench._state['running'] is True


def test_missing_mode_and_deleted_previous_destination_use_safe_defaults(monkeypatch, tmp_path):
    workbench, _, _, seen, *_ = fixture(monkeypatch, tmp_path, accepted=False)
    del workbench._mode
    workbench._diagnostics_directory = str(tmp_path / 'no longer exists')
    monkeypatch.setattr(gui, 'QStandardPaths', SimpleNamespace(
        StandardLocation=SimpleNamespace(DocumentsLocation=1), writableLocation=lambda _: str(tmp_path)))
    gui.Workbench._export_diagnostics(workbench)
    assert seen['mode'] == 'skills_auto' and seen['destination'] == str(tmp_path)


def test_collect_failure_does_not_create_a_bundle(monkeypatch, tmp_path):
    workbench, calls, logs, *_ = fixture(monkeypatch, tmp_path)
    def invalid_controls():
        raise ValueError('Duplicate motor port')
    workbench._collect_config = invalid_controls
    gui.Workbench._export_diagnostics(workbench)
    assert not calls and not list(tmp_path.iterdir())
    assert 'Duplicate motor port' in workbench.diagnostics_status.value
    assert workbench._building is True and logs
