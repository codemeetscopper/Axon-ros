from __future__ import annotations


def build_stylesheet() -> str:
    return """
    QWidget {
        font-family: "Inter", "Segoe UI", "Helvetica", sans-serif;
        font-size: 12px;
        color: #e6e7eb;
    }
    QMainWindow {
        background-color: #111318;
    }
    QListWidget {
        background-color: #0c0e12;
        border: none;
        padding: 8px;
    }
    QListWidget::item {
        padding: 8px 10px;
        border-radius: 8px;
    }
    QListWidget::item:selected {
        background-color: #1f2430;
    }
    QGroupBox {
        border: 1px solid #1f2430;
        border-radius: 10px;
        margin-top: 12px;
        padding: 12px;
        background-color: #151922;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 6px;
        color: #9aa3b2;
    }
    QLineEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox {
        background-color: #0f1218;
        border: 1px solid #1f2430;
        border-radius: 6px;
        padding: 6px;
    }
    QPushButton {
        background-color: #2b3242;
        border: none;
        border-radius: 8px;
        padding: 6px 12px;
    }
    QPushButton:hover {
        background-color: #364057;
    }
    QPushButton:disabled {
        background-color: #1b202b;
        color: #6c758a;
    }
    QLabel#SectionTitle {
        font-size: 16px;
        font-weight: 600;
    }
    QLabel#Muted {
        color: #9aa3b2;
    }
    """
