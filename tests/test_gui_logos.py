"""Status decisions and icon caching, without a Qt application or native window."""
from pathlib import Path
from types import SimpleNamespace

import pytest

from eersim import gui


@pytest.mark.parametrize('state,expected', [
    ({}, 'happy'), ({'paused': True, 'running': True}, 'snooze'),
    ({'running': True, 'phase': 'initialize', 'mode': 'skills_auto'}, 'alert_normal'),
    ({'running': True, 'mode': 'skills_auto'}, 'alert_happy'),
    ({'running': True, 'mode': 'match_auto'}, 'alert_happy'),
    ({'running': True, 'mode': 'skills_driver'}, 'swag'),
    ({'running': True, 'mode': 'driver_auto'}, 'swag')])
def test_mode_and_pause_select_the_named_user_asset(state, expected):
    assert gui.AppLogoState().name(state) == expected


def test_error_and_work_precedence_survive_repeated_telemetry_until_next_action():
    logo = gui.AppLogoState()
    state = {'running': True, 'mode': 'skills_auto'}
    assert logo.name(state, building=True) == 'alert_normal'
    assert logo.name({'paused': True}, loading=True) == 'alert_normal'
    logo.event('error')
    for _ in range(200):
        logo.observe(state)
        assert logo.name(state, building=True, loading=True) == 'question_confused'
    logo.event('begin')
    assert logo.name(state, loading=True) == 'alert_normal'
    assert logo.name(state) == 'alert_happy'


def test_completion_differs_from_manual_stop_and_never_clears_an_error():
    running = {'running': True, 'mode': 'skills_auto', 'duration': 30, 'mode_time': 29}
    finished = {**running, 'running': False, 'mode_time': 30}
    logo = gui.AppLogoState()
    logo.observe(running)
    logo.observe(finished)
    for _ in range(10):
        logo.observe(finished)
        assert logo.name(finished) == 'done_confirm_good'
    logo.observe(running)
    logo.event('stopped')
    logo.observe(finished)
    assert logo.name(finished) == 'snooze'
    logo.observe(running)
    logo.event('error')
    logo.observe(finished)
    assert logo.name(finished) == 'question_confused'


@pytest.mark.parametrize('event,name', [('success', 'done_confirm_good'), ('robot_import', 'love'), ('field_import', 'wink')])
def test_idle_success_remains_until_another_activity(event, name):
    logo = gui.AppLogoState()
    logo.event(event)
    for _ in range(20):
        logo.observe({'running': False})
        assert logo.name({}) == name
    assert logo.name({'running': True, 'mode': 'skills_driver'}) == 'swag'
    logo.event('begin')
    assert logo.name({}) == 'happy'


def test_repeated_states_do_not_recreate_icons_or_update_application_icon(monkeypatch):
    constructed, window_icons, app_icons, pixmaps = [], [], [], []

    class Icon:
        def __init__(self, path):
            self.name = Path(path).stem
            constructed.append(self.name)

        def isNull(self):
            return False

        def pixmap(self, width, height):
            return (self.name, width, height)

    monkeypatch.setattr(gui, 'QIcon', Icon)
    monkeypatch.setattr(gui, 'QApplication', SimpleNamespace(instance=lambda: SimpleNamespace(setWindowIcon=app_icons.append)))
    bench = SimpleNamespace(_state={'running': False}, _pending_scene_applies=0, _scene_apply_ready=False,
        _building=False, _logo=gui.AppLogoState(), _logo_cache={}, _shown_logo=None,
        setWindowIcon=window_icons.append, brand_logo=SimpleNamespace(setPixmap=pixmaps.append,
            setToolTip=lambda _: None, setAccessibleName=lambda _: None))
    for _ in range(50):
        gui.Workbench._refresh_logo(bench)
    assert constructed == ['happy'] and len(window_icons) == len(app_icons) == len(pixmaps) == 1
    bench._state = {'running': True, 'mode': 'match_auto'}
    gui.Workbench._refresh_logo(bench)
    bench._state = {'running': False}
    gui.Workbench._refresh_logo(bench)
    assert constructed == ['happy', 'alert_happy']
    assert [icon.name for icon in window_icons] == ['happy', 'alert_happy', 'happy']


def test_all_resolved_logos_are_packaged_source_assets():
    for name in ('happy', 'snooze', 'question_confused', 'done_confirm_good', 'alert_happy',
                 'alert_normal', 'love', 'swag', 'wink'):
        assert (gui.ASSETS / 'logos' / f'{name}.png').is_file()
