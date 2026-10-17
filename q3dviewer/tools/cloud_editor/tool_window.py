"""Shared window primitives for cloud-editor tools."""

from pathlib import Path

from q3dviewer.Qt.QtCore import QSize, Signal
from q3dviewer.Qt.QtGui import QIcon
from q3dviewer.Qt.QtWidgets import (
    QHBoxLayout, QLabel, QMdiSubWindow, QToolButton, QVBoxLayout, QWidget)


ICON_DIR = Path(__file__).resolve().parent / 'icons'


def make_icon_button(parent, icon_name, tooltip):
    button = QToolButton(parent)
    button.setIcon(QIcon(str(ICON_DIR / f'{icon_name}.svg')))
    button.setIconSize(QSize(28, 28))
    button.setFixedSize(38, 38)
    button.setAutoRaise(True)
    button.setToolTip(tooltip)
    return button


class ToolWindow(QMdiSubWindow):
    """Common tool window with undo, apply and cancel controls."""

    apply_requested = Signal()
    undo_requested = Signal()
    cancel_requested = Signal()
    closed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._placed_initially = False

        content = QWidget(self)
        self.setWidget(content)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(8, 6, 8, 6)
        tool_layout = QHBoxLayout()
        self.content_layout = QHBoxLayout()
        tool_layout.addLayout(self.content_layout)

        self.btn_undo = make_icon_button(
            self, 'undo', 'Undo last pending operation')
        self.btn_undo.clicked.connect(self.undo_requested)
        tool_layout.addWidget(self.btn_undo)

        self.btn_apply = make_icon_button(
            self, 'check', 'Apply pending operations')
        self.btn_apply.clicked.connect(self.apply_requested)
        tool_layout.addWidget(self.btn_apply)

        self.btn_cancel = make_icon_button(
            self, 'close', 'Cancel pending operations')
        self.btn_cancel.clicked.connect(self.cancel_requested)
        self.cancel_requested.connect(self.close)
        tool_layout.addWidget(self.btn_cancel)

        layout.addLayout(tool_layout)
        self.hint_label = QLabel()
        self.hint_label.setStyleSheet('color: gray;')
        layout.addWidget(self.hint_label)

    def set_hint(self, text):
        self.hint_label.setText(text)
        self.widget().layout().activate()
        required_width = self.sizeHint().width()
        if required_width > self.width():
            self.resize(required_width, self.height())

    def showEvent(self, event):
        super().showEvent(event)
        if not self._placed_initially:
            parent = self.parentWidget()
            right = max(0, parent.width() - self.width())
            self.move(right, 0)
            self._placed_initially = True

    def closeEvent(self, event):
        event.accept()
        self.closed.emit()
