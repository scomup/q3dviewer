#!/usr/bin/env python3
"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.glwidget import *
import signal
from PySide6.QtWidgets import QMainWindow, QApplication


def handler(signal, frame):
    QApplication.quit()
    print("Force close by Ctrl+C")


class Viewer(QMainWindow):
    def __init__(self, name='Viewer', win_size=[1920, 1080]):
        signal.signal(signal.SIGINT, handler)
        super(Viewer, self).__init__()
        self.setGeometry(0, 0, win_size[0], win_size[1])
        self.init_ui()
        self.setWindowTitle(name)

    def init_ui(self):
        center_widget = QWidget()
        self.setCentralWidget(center_widget)
        self.layout = QHBoxLayout()
        center_widget.setLayout(self.layout)
        self.glwidget = GLWidget()
        self.layout.addWidget(self.glwidget, 1)
        timer = QtCore.QTimer(self)
        timer.setInterval(20)  # period, in milliseconds
        timer.timeout.connect(self.update)
        timer.start()

    def add_items(self, named_items: dict):
        for name, item in named_items.items():
            self.glwidget.add_item_with_name(name, item)

    def __getitem__(self, name: str):
        if name in self.glwidget.named_items:
            return self.glwidget.named_items[name]
        else:
            return None

    def update(self):
        # force update by timer
        self.glwidget.update()

    def closeEvent(self, event):
        event.accept()
        QApplication.quit()

    def show(self):
        self.glwidget.setting_window.add_setting(
            "main win", self.glwidget)
        super().show()
