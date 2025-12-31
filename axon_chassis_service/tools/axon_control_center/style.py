from __future__ import annotations


def build_stylesheet() -> str:
    return """
    QWidget {
        font-family: "Inter", "Segoe UI", "Helvetica", sans-serif;
        font-size: 12px;
        color: #1f2430;
    }
    QMainWindow {
        background-color: #f6f7fb;
    }
    QListWidget {
        background-color: #ffffff;
        border: 1px solid #e5e7ef;
        padding: 8px;
    }
    QListWidget::item {
        padding: 8px 10px;
        border-radius: 8px;
    }
    QListWidget::item:selected {
        background-color: #e8ecf7;
    }
    QGroupBox {
        border: 1px solid #d9dfea;
        border-radius: 10px;
        margin-top: 12px;
        padding: 12px;
        background-color: #ffffff;
    }
    QGroupBox::title {
        subcontrol-origin: margin;
        left: 10px;
        padding: 0 6px;
        color: #6b7385;
    }
    QLineEdit, QPlainTextEdit, QSpinBox, QDoubleSpinBox {
        background-color: #f4f6fb;
        border: 1px solid #d9dfea;
        border-radius: 6px;
        padding: 6px;
    }
    QPushButton {
        background-color: #3b6ef5;
        border: none;
        border-radius: 8px;
        padding: 6px 12px;
        color: #ffffff;
    }
    QPushButton:hover {
        background-color: #335fde;
    }
    QPushButton:disabled {
        background-color: #d9dfea;
        color: #8c93a3;
    }
    QLabel#SectionTitle {
        font-size: 16px;
        font-weight: 600;
    }
    QLabel#Muted {
        color: #6b7385;
    }
    """
