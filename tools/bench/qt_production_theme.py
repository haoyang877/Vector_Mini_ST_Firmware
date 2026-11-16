"""统一Qt浅色调色板，避免Windows主题与自定义控件边框混用。"""

from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication

THEME = """
QMainWindow,QDialog { background:#f2f5f9; }
QWidget { font-family:'Microsoft YaHei UI'; font-size:13px; color:#24354b; }
QGroupBox { background:white; border:1px solid #dce4ed; border-radius:10px; margin-top:12px; padding:14px 12px 10px; }
QGroupBox::title { subcontrol-origin:margin; left:14px; padding:0 5px; color:#52667e; font-weight:600; }
QLabel#title { font-size:23px; font-weight:700; color:#132e49; }
QLabel#subtitle { color:#60758b; font-size:12px; }
QLabel { background:transparent; border:0; }
QLabel#statusValue { color:#253f58; font-weight:600; }
QLabel#metric { font-size:19px; font-weight:600; color:#126c87; }
QLabel#metric[unavailable="true"] { color:#8798a9; }
QWidget#telemetryTile { background:#f3f7fa; border:1px solid #e3ebf2; border-radius:7px; }
QLabel#sectionTitle { font-size:20px; font-weight:600; color:#1c3d59; }
QListWidget#navigation { background:#e8eef5; color:#334b62; border:0; border-radius:10px; padding:10px 6px; outline:0; }
QListWidget#navigation::item { height:48px; padding-left:14px; border-radius:7px; margin:3px; }
QListWidget#navigation::item:selected { background:#d4e7f3; color:#125e82; }
QListWidget#navigation::item:hover { background:#dce7f1; }
QPushButton { background:white; border:1px solid #cdd8e4; border-radius:6px; padding:7px 13px; min-height:19px; }
QPushButton:hover { background:#edf5fb; border-color:#5095b5; }
QPushButton:pressed { background:#dbeaf5; }
QPushButton:disabled { color:#97a5b3; background:#f4f6f8; border-color:#e2e7ed; }
QPushButton#primary { color:white; background:#126f94; border-color:#126f94; font-weight:600; }
QPushButton#primary:disabled { color:#a5b9c3; background:#dce6eb; border-color:#dce6eb; }
QPushButton#stop { color:#b33b45; border-color:#e4b5b9; background:#fff7f7; }
QPushButton#stop:disabled { color:#97a5b3; border-color:#e2e7ed; background:#f4f6f8; }
QLineEdit,QComboBox { background:white; border:1px solid #cdd8e4; border-radius:5px; padding:6px 8px; min-height:20px; }
QLineEdit:focus,QComboBox:focus { border:1px solid #1680a8; }
QLineEdit:read-only { background:#f3f6fa; color:#62778d; }
QTabWidget::pane { border:0; background:white; }
QTabBar::tab { padding:9px 17px; background:#e9eef4; border:0; margin-right:3px; color:#60748b; }
QTabBar::tab:selected { background:white; color:#126f94; border-bottom:2px solid #126f94; }
QTableWidget { border:1px solid #e0e6ee; background:white; gridline-color:#edf1f6; alternate-background-color:#f8fafc; selection-background-color:#dceef8; selection-color:#12384d; }
QHeaderView::section { background:#f0f4f8; padding:8px; border:0; border-bottom:1px solid #dde5ee; font-weight:600; color:#52667e; }
QPlainTextEdit { background:white; border:1px solid #dce4ed; border-radius:6px; padding:6px; }
QProgressBar { border:0; border-radius:4px; background:#e3eaf1; height:8px; }
QProgressBar::chunk { background:#169e99; border-radius:4px; }
QScrollArea { border:0; background:#f2f5f9; }
QComboBox::drop-down { border:0; background:transparent; width:20px; }
QComboBox QAbstractItemView { background:white; color:#24354b; selection-background-color:#d4e7f3; selection-color:#125e82; border:1px solid #cdd8e4; outline:0; }
QScrollBar:vertical { background:#edf2f7; width:10px; margin:0; border:0; }
QScrollBar:horizontal { background:#edf2f7; height:10px; margin:0; border:0; }
QScrollBar::handle { background:#b8c8d8; border-radius:4px; min-height:24px; min-width:24px; }
QScrollBar::add-line,QScrollBar::sub-line { width:0; height:0; border:0; background:transparent; }
QScrollBar::add-page,QScrollBar::sub-page { background:transparent; }
QTableCornerButton::section { background:#f0f4f8; border:0; }
QToolTip { background:white; color:#24354b; border:1px solid #cdd8e4; }
QSplitter::handle { background:#e0e7ef; }
"""


def apply_theme(app: QApplication) -> None:
    """整个进程统一Fusion与浅色语义色，包括弹窗和原生子控件。"""
    if app.property("production_theme") == "light-v2.0.1":
        return
    app.setStyle("Fusion")
    palette = QPalette()
    colors = {
        QPalette.ColorRole.Window: "#f2f5f9",
        QPalette.ColorRole.WindowText: "#24354b",
        QPalette.ColorRole.Base: "#ffffff",
        QPalette.ColorRole.AlternateBase: "#f8fafc",
        QPalette.ColorRole.Text: "#24354b",
        QPalette.ColorRole.Button: "#ffffff",
        QPalette.ColorRole.ButtonText: "#24354b",
        QPalette.ColorRole.Highlight: "#d4e7f3",
        QPalette.ColorRole.HighlightedText: "#125e82",
        QPalette.ColorRole.ToolTipBase: "#ffffff",
        QPalette.ColorRole.ToolTipText: "#24354b",
        QPalette.ColorRole.Light: "#ffffff",
        QPalette.ColorRole.Midlight: "#e8eef5",
        QPalette.ColorRole.Mid: "#cdd8e4",
        QPalette.ColorRole.Dark: "#9aafc2",
        QPalette.ColorRole.Shadow: "#cdd8e4",
        QPalette.ColorRole.PlaceholderText: "#73879b",
    }
    for role, value in colors.items():
        palette.setColor(role, QColor(value))
    for role in (
        QPalette.ColorRole.Text,
        QPalette.ColorRole.ButtonText,
        QPalette.ColorRole.WindowText,
    ):
        palette.setColor(QPalette.ColorGroup.Disabled, role, QColor("#8798a9"))
    app.setPalette(palette)
    app.setStyleSheet(THEME)
    app.setProperty("production_theme", "light-v2.0.1")
