# Qt and Qt for Python notices

EERSim uses Qt through **PySide6 6.8.3**, with **Shiboken6 6.8.3** and the
**Qt 6.8.3** shared libraries supplied in the Qt for Python wheels. Qt, PySide
and Shiboken are copyright The Qt Company Ltd. and their respective contributors.
The Qt Core, Gui and Widgets libraries used by the workbench are supplied under
the LGPL version 3 option. Their source was not modified by EERSim.

The accompanying [LGPL version 3](LGPL-3.0-only.txt) incorporates
[GPL version 3](GPL-3.0-only.txt); both texts are included verbatim from Qt's
6.8.3 source tree. [License text sources and hashes](license-sources.json)
record their provenance. Other third-party components retain their own notices
in the dependency metadata and source packages.

EERSim's academic/noncommercial license applies to its own material. It does
not restrict your rights under the Qt libraries' licenses, including modifying
or replacing those libraries, relinking, and reverse engineering to debug
those modifications. These permissions do not require commercial-use permission
from EERSim's author. They do not change the terms governing EERSim itself.

## Matching source

The complete upstream release sources are available from Qt's public download
server; no account or commercial Qt license is needed to obtain these archives:

- [PySide/Qt for Python 6.8.3 source, including Shiboken](https://download.qt.io/official_releases/QtForPython/pyside6/PySide6-6.8.3-src/pyside-setup-everywhere-src-6.8.3.tar.xz).
- [Qt framework 6.8.3 source](https://download.qt.io/archive/qt/6.8/6.8.3/single/qt-everywhere-src-6.8.3.tar.xz).

These archives include the upstream build files and component-specific license
notices. Follow the matching [Qt for Windows build instructions](https://doc.qt.io/qt-6.8/windows-building.html)
and [Qt for Python 6.8 Windows build instructions](https://doc.qt.io/qtforpython-6.8/building_from_source/windows.html).
For PySide, use a matching Qt installation and compatible MSVC/Python build
tools. The versioned source links identify the shipped libraries; a distributor
remains responsible for the corresponding-source obligations of its distribution.

## Replacing shared libraries

The standard Windows installer installs a directory bundle, with replaceable
libraries beside the application. Close EERSim and preserve a backup before
replacing files. Within the installation's `_internal` directory:

- Qt DLLs are in `PySide6`, including `Qt6Core.dll`, `Qt6Gui.dll`, and
  `Qt6Widgets.dll`; dependent Qt DLLs must remain compatible.
- Qt plugins retain their corresponding subdirectories under `PySide6/plugins`,
  such as `platforms/qwindows.dll`.
- PySide extension modules and `pyside6.abi3.dll` are in `PySide6`;
  `shiboken6.abi3.dll` and its Python extension are in `shiboken6`.

Use interface-compatible 64-bit Windows libraries matching the installed Qt,
PySide and Python ABI, keeping the existing relative paths and required plugin
layout. Start the application normally with those replacement files. EERSim
does not enforce a signature or hash restriction on these Qt components.
In a source installation, install the replacement PySide/Shiboken wheels into
the Python environment used to launch EERSim. EERSim's build scripts can also
create a directory bundle from that environment. Custom one-file builds should
be rebuilt with the modified libraries; the installer directory layout avoids
having to replace files in a temporary extraction directory.

For the library terms and obligations, see the included license texts and
[Qt's official LGPL guidance](https://www.qt.io/development/open-source-lgpl-obligations).
