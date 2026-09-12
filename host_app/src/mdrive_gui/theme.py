"""Dark Qt theme matching docs/mockups/ui_reference.html palette tokens."""

from __future__ import annotations

from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import QApplication

# Palette tokens (single source of truth; mirrors the mockup CSS variables).
BG = "#1e1e1e"
PANEL = "#252526"
PANEL2 = "#2d2d30"
PANEL3 = "#333337"
BORDER = "#3c3c3c"
BORDER2 = "#4a4a4f"
TEXT = "#e0e0e0"
MUTED = "#9e9e9e"
FAINT = "#6e6e6e"
ACCENT = "#4FC3F7"
OK = "#66BB6A"
ERROR = "#EF5350"
WARN = "#FFB74D"
INPUT_BG = "#1b1b1c"
SEL_BG = "#123c4e"
SEL_BORDER = "#1d6a86"
SEL_TEXT = "#bfeaff"
DIRTY_BG = "#3a3418"
ERR_BG = "#3a1e1e"
PLOT_BG = "#111213"

SANS = '"Microsoft YaHei", "Segoe UI", "SimHei", sans-serif'
MONO = '"Consolas", "Cascadia Mono", monospace'


def color(token: str) -> QColor:
    """Return a QColor for a palette token, for model/delegate roles."""
    return QColor(token)


QSS = f"""
* {{ font-family: {SANS}; font-size: 12px; }}
QMainWindow, QDialog {{ background: {BG}; }}
QWidget {{ background: {BG}; color: {TEXT}; }}
QFrame#card {{ background: {PANEL}; border: 1px solid {BORDER}; border-radius: 6px; }}
QLabel#cardTitle {{ color: {TEXT}; font-weight: 600; padding: 2px 0; }}
QLabel {{ background: transparent; }}
QLabel[muted="true"] {{ color: {MUTED}; }}
QLabel[faint="true"] {{ color: {FAINT}; }}
QLabel[mono="true"] {{ font-family: {MONO}; }}
QLabel#linkBadge {{ padding: 2px 8px; border: 1px solid {BORDER2}; border-radius: 4px; background: {PANEL2}; }}

QMenuBar {{ background: {PANEL}; border-bottom: 1px solid {BORDER}; }}
QMenuBar::item {{ padding: 4px 10px; background: transparent; }}
QMenuBar::item:selected {{ background: {PANEL3}; border-radius: 4px; }}
QMenu {{ background: {PANEL2}; border: 1px solid {BORDER2}; }}
QMenu::item {{ padding: 5px 22px; }}
QMenu::item:selected {{ background: {SEL_BG}; color: {SEL_TEXT}; }}

QToolBar {{ background: {PANEL}; border-bottom: 1px solid {BORDER}; spacing: 6px; padding: 4px 8px; }}
QToolBar::separator {{ background: {BORDER2}; width: 1px; margin: 4px; }}

QPushButton, QToolButton {{
    background: {PANEL2}; color: {TEXT};
    border: 1px solid {BORDER2}; border-radius: 4px; padding: 4px 12px;
}}
QPushButton:hover, QToolButton:hover {{ background: {PANEL3}; }}
QPushButton:disabled, QToolButton:disabled {{ color: {FAINT}; background: {PANEL}; border-color: {BORDER}; }}
QPushButton[variant="primary"], QToolButton[variant="primary"] {{
    background: {SEL_BG}; border-color: {SEL_BORDER}; color: {SEL_TEXT};
}}
QPushButton[variant="primary"]:hover, QToolButton[variant="primary"]:hover {{ background: #16495f; }}
QPushButton[variant="danger"] {{ background: #3f1f1f; border-color: #7a3333; color: #ffd2d2; }}
QPushButton[variant="danger"]:hover {{ background: #4d2626; }}
QToolButton::menu-indicator {{ image: none; width: 0; }}

QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {{
    background: {INPUT_BG}; color: {TEXT};
    border: 1px solid {BORDER2}; border-radius: 4px; padding: 3px 8px;
    selection-background-color: {SEL_BG};
}}
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {{ border-color: {SEL_BORDER}; }}
QLineEdit:disabled, QComboBox:disabled {{ color: {FAINT}; }}
QComboBox QAbstractItemView {{ background: {PANEL2}; border: 1px solid {BORDER2}; selection-background-color: {SEL_BG}; }}
QComboBox::drop-down {{ border: none; width: 20px; }}

QTabWidget::pane {{ border: 1px solid {BORDER}; top: -1px; background: {BG}; }}
QTabBar::tab {{
    background: transparent; color: {MUTED};
    padding: 7px 18px; border-bottom: 2px solid transparent;
}}
QTabBar::tab:hover {{ color: {TEXT}; }}
QTabBar::tab:selected {{ color: {ACCENT}; border-bottom-color: {ACCENT}; }}

QTableView, QTableWidget, QTreeWidget, QListWidget, QPlainTextEdit, QTextEdit {{
    background: {INPUT_BG}; color: {TEXT};
    border: 1px solid {BORDER}; border-radius: 4px;
    gridline-color: #2f2f31;
    alternate-background-color: #28282a;
    selection-background-color: {SEL_BG}; selection-color: {SEL_TEXT};
}}
QTableView::item, QTableWidget::item {{ padding: 3px 6px; border: none; }}
QHeaderView::section {{
    background: {PANEL}; color: {MUTED}; font-weight: 500;
    border: none; border-bottom: 1px solid {BORDER2}; padding: 5px 8px;
}}
QTableCornerButton::section {{ background: {PANEL}; border: none; }}
QTreeWidget::item {{ padding: 3px 4px; }}
QTreeWidget::item:selected {{ background: {SEL_BG}; color: {SEL_TEXT}; }}
QTreeWidget::item:hover {{ background: {PANEL3}; }}

QGroupBox {{
    background: {PANEL}; border: 1px solid {BORDER}; border-radius: 6px;
    margin-top: 14px; padding-top: 8px;
}}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; top: 2px; color: {TEXT}; font-weight: 600; }}

QProgressBar {{
    background: {INPUT_BG}; border: 1px solid {BORDER2}; border-radius: 7px;
    height: 12px; text-align: center; color: {MUTED}; font-size: 10px;
}}
QProgressBar::chunk {{ background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 {SEL_BORDER}, stop:1 {ACCENT}); border-radius: 6px; }}

QCheckBox {{ spacing: 6px; background: transparent; }}
QCheckBox::indicator {{ width: 14px; height: 14px; border: 1px solid {BORDER2}; border-radius: 3px; background: {INPUT_BG}; }}
QCheckBox::indicator:checked {{ background: {SEL_BG}; border-color: {SEL_BORDER}; }}
QRadioButton {{ background: transparent; }}

QStatusBar {{ background: {PANEL}; border-top: 1px solid {BORDER}; color: {MUTED}; }}
QStatusBar::item {{ border: none; }}
QStatusBar QLabel {{ color: {MUTED}; padding: 0 6px; }}
QStatusBar QLabel[value="true"] {{ color: {TEXT}; font-family: {MONO}; }}

QSplitter::handle {{ background: {BORDER}; }}
QSplitter::handle:horizontal {{ width: 2px; }}
QSplitter::handle:vertical {{ height: 2px; }}

QScrollBar:vertical {{ background: {BG}; width: 10px; margin: 0; }}
QScrollBar::handle:vertical {{ background: {BORDER2}; min-height: 24px; border-radius: 5px; }}
QScrollBar::handle:vertical:hover {{ background: {FAINT}; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QScrollBar:horizontal {{ background: {BG}; height: 10px; margin: 0; }}
QScrollBar::handle:horizontal {{ background: {BORDER2}; min-width: 24px; border-radius: 5px; }}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{ width: 0; }}

QToolTip {{ background: {PANEL2}; color: {TEXT}; border: 1px solid {BORDER2}; padding: 4px 6px; }}
QFileDialog, QMessageBox {{ background: {BG}; }}
"""


def apply_theme(app: QApplication) -> None:
    """Apply the Fusion base style, dark palette, and the app stylesheet."""
    app.setStyle("Fusion")
    app.setFont(QFont("Microsoft YaHei", 9))
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, color(BG))
    palette.setColor(QPalette.ColorRole.WindowText, color(TEXT))
    palette.setColor(QPalette.ColorRole.Base, color(INPUT_BG))
    palette.setColor(QPalette.ColorRole.AlternateBase, color("#28282a"))
    palette.setColor(QPalette.ColorRole.Text, color(TEXT))
    palette.setColor(QPalette.ColorRole.Button, color(PANEL2))
    palette.setColor(QPalette.ColorRole.ButtonText, color(TEXT))
    palette.setColor(QPalette.ColorRole.Highlight, color(SEL_BG))
    palette.setColor(QPalette.ColorRole.HighlightedText, color(SEL_TEXT))
    palette.setColor(QPalette.ColorRole.PlaceholderText, color(FAINT))
    palette.setColor(QPalette.ColorRole.ToolTipBase, color(PANEL2))
    palette.setColor(QPalette.ColorRole.ToolTipText, color(TEXT))
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, color(FAINT))
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, color(FAINT))
    app.setPalette(palette)
    app.setStyleSheet(QSS)
