"""Static packaging policy: no import of PyInstaller, Qt, or native libraries."""
import ast
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def keep_payload():
    tree=ast.parse((ROOT/'packaging/EERSim.spec').read_text(encoding='utf-8'))
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef)and n.name=='keep_qt_payload')
    namespace={}
    exec(compile(ast.Module(body=[function],type_ignores=[]),'<Qt payload policy>','exec'),namespace)
    return namespace['keep_qt_payload']


def test_only_unused_virtual_keyboard_payload_is_removed():
    keep=keep_payload()
    for name in ('PySide6/Qt6VirtualKeyboard.dll','PySide6/plugins/platforminputcontexts/qtvirtualkeyboardplugin.dll',
                 r'PYSIDE6\Qt6VirtualKeyboard.DLL'):
        assert not keep((name,'source','BINARY'))
    for name in ('PySide6/Qt6Core.dll','PySide6/Qt6Gui.dll','PySide6/Qt6Widgets.dll',
                 'PySide6/plugins/platforms/qwindows.dll','PySide6/plugins/platforms/qoffscreen.dll',
                 'PySide6/plugins/imageformats/qsvg.dll','PySide6/plugins/styles/qmodernwindowsstyle.dll',
                 'other/Qt6VirtualKeyboard.dll','PySide6-6.8.3.dist-info/LicenseRef-Qt-Commercial.txt'):
        assert keep((name,'source','BINARY'))


def test_bundled_license_texts_match_the_recorded_official_source_bytes():
    notices=ROOT/'docs/notices'
    sources=json.loads((notices/'license-sources.json').read_text())
    assert set(sources)=={'LGPL-3.0-only.txt','GPL-3.0-only.txt'}
    for name,metadata in sources.items():
        data=(notices/name).read_bytes()
        assert hashlib.sha256(data).hexdigest()==metadata['sha256']
        assert len(data)==metadata['bytes']
        assert metadata['source'].startswith('https://raw.githubusercontent.com/qt/qtbase/v6.8.3/LICENSES/')
        assert b'Everyone is permitted to copy and distribute verbatim copies' in data
