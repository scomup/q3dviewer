#!/usr/bin/env python3
"""
Copyright 2024  Liu Yang
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.glv_widget import *
import signal
import sys


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
        layout = QVBoxLayout()
        center_widget.setLayout(layout)
        self.glv_widget = GLVWidget()
        layout.addWidget(self.glv_widget, 1)
        timer = QtCore.QTimer(self)
        timer.setInterval(20)  # period, in milliseconds
        timer.timeout.connect(self.update)
        self.glv_widget.setCameraPosition(distance=40)
        timer.start()

    def add_items(self, named_items: dict):
        for name, item in named_items.items():
            self.glv_widget.add_item(name, item)

    def __getitem__(self, name: str):
        if name in self.glv_widget.named_items:
            return self.glv_widget.named_items[name]
        else:
            return None

    def update(self):
        # force update by timer
        self.glv_widget.update()

    def closeEvent(self, event):
        event.accept()
        QApplication.quit()

    def show(self):
        self.glv_widget.setting_window.add_setting(
            "main win", self.glv_widget)
        super().show()
