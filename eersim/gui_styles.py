"""A restrained WVU-inspired desktop palette shared by the simulator widgets."""

NAVY = "#081320"
PANEL = "#101e2d"
GOLD = "#edbd50"
TEXT = "#edf3f8"
MUTED = "#899aaf"

STYLESHEET = """
QMainWindow, QWidget#Root { background: #081320; color: #edf3f8; }
QWidget { color: #edf3f8; font-family: 'Segoe UI', sans-serif; font-size: 12px; }
QFrame#Header { background: #0c1928; border-bottom: 1px solid #223348; }
QFrame#Panel, QFrame#ControlBar { background: #101e2d; border: 1px solid #25354a; border-radius: 10px; }
QFrame#Metric { background: #101e2d; border: 1px solid #223348; border-radius: 8px; }
QLabel#Brand { font-size: 23px; font-weight: 800; letter-spacing: 2px; color: #edbd50; }
QLabel#BrandSub { font-size: 10px; color: #8a9eb5; letter-spacing: 2px; }
QLabel#Title { font-size: 15px; font-weight: 650; }
QLabel#Muted { color: #91a4ba; }
QLabel#Section { color: #edbd50; font-size: 10px; font-weight: 700; letter-spacing: 1px; }
QLabel#Badge { background: #182c35; color: #87d6bd; padding: 6px 11px; border-radius: 5px; font-weight: 650; }
QLabel#MetricValue { font-family: 'Cascadia Mono', 'Consolas', monospace; font-size: 21px; font-weight: 650; }
QLabel#MetricLabel { color: #91a4ba; font-size: 10px; letter-spacing: 1px; }
QPushButton, QToolButton { background: #1b2c40; border: 1px solid #30445c; border-radius: 6px; padding: 8px 11px; font-weight: 600; }
QPushButton:hover, QToolButton:hover { background: #273e55; border-color: #59718b; }
QPushButton:pressed, QToolButton:pressed { background: #304963; }
QPushButton:disabled { color: #63758c; background: #142335; border-color: #24364c; }
QPushButton#Primary { background: #edbd50; color: #152233; border-color: #edbd50; font-weight: 750; }
QPushButton#Primary:hover { background: #f5cb70; }
QPushButton#Primary:disabled { background: #625637; color: #a59e8c; border-color: #625637; }
QPushButton#Danger { color: #f0a4a4; }
QPushButton#Mode { text-align: left; padding: 10px 13px; background: #142335; border: 1px solid #2b3d53; }
QPushButton#Mode:checked { background: #303023; color: #f4ce76; border: 1px solid #edbd50; }
QPushButton#Digital:pressed, QPushButton#Digital:checked { background: #edbd50; color: #152233; }
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background: #0c1726; border: 1px solid #2a3d53; border-radius: 5px; padding: 6px 8px; selection-background-color: #826d34; }
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus { border-color: #edbd50; }
QComboBox::drop-down { width: 22px; border: none; }
QComboBox QAbstractItemView { background: #142335; selection-background-color: #32465c; }
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button { width: 16px; background: #21344a; border: none; }
QTabWidget::pane { border: 1px solid #25354a; background: #101e2d; border-radius: 7px; top: -1px; }
QTabBar::tab { color: #91a4ba; background: #0c1928; padding: 12px 8px; border-bottom: 2px solid transparent; font-size: 11px; }
QTabBar::tab:selected { color: #edbd50; border-bottom: 2px solid #edbd50; background: #101e2d; }
QTabBar::tab:hover { color: #edf3f8; }
QScrollArea { border: none; background: transparent; }
QScrollArea > QWidget > QWidget { background: #101e2d; }
QScrollBar:vertical { background: #101e2d; width: 8px; margin: 2px; }
QScrollBar::handle:vertical { background: #33485f; border-radius: 3px; min-height: 24px; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal { background: #101e2d; height: 8px; }
QScrollBar::handle:horizontal { background: #33485f; border-radius: 3px; min-width: 24px; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }
QGroupBox { border: 1px solid #2a3d53; border-radius: 6px; margin-top: 13px; padding-top: 12px; }
QGroupBox::title { subcontrol-origin: margin; left: 10px; color: #a6b6c8; }
QCheckBox { spacing: 7px; }
QCheckBox::indicator { width: 15px; height: 15px; border: 1px solid #405773; border-radius: 3px; background: #0c1726; }
QCheckBox::indicator:checked { background: #edbd50; border-color: #edbd50; }
QPlainTextEdit { background: #091522; border: 1px solid #25394f; border-radius: 5px; font-family: 'Cascadia Mono', 'Consolas', monospace; font-size: 11px; selection-background-color: #324b67; }
QTableWidget { background: #0c1726; alternate-background-color: #122135; border: 1px solid #25394f; gridline-color: #23364c; selection-background-color: #30465d; }
QHeaderView::section { background: #1a2b3e; color: #9eb1c8; padding: 6px 4px; border: none; font-size: 10px; }
QTableWidget QLineEdit { padding: 2px; }
QProgressBar { background: #081320; border: none; border-radius: 3px; min-height: 5px; max-height: 5px; }
QProgressBar::chunk { background: #edbd50; border-radius: 3px; }
QSplitter::handle { background: #081320; width: 10px; }
QStatusBar { background: #081320; color: #8296ad; font-size: 11px; }
QToolTip { background: #26384b; color: #edf3f8; border: 1px solid #526880; padding: 6px; }
QMessageBox, QFileDialog { background: #101e2d; }
"""
