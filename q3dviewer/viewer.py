#!/usr/bin/env python3
"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.glwidget import *
import signal
from PySide6.QtWidgets import QMainWindow, QApplication, QHBoxLayout


def handler(signal, frame):
    QApplication.quit()
    print("Force close by Ctrl+C")


class Viewer(QMainWindow):
    def __init__(self, name='Viewer', win_size=[1920, 1080]):
        signal.signal(signal.SIGINT, handler)
        super(Viewer, self).__init__()
        self.setGeometry(0, 0, win_size[0], win_size[1])
        self.init_ui()
        self.add_update_timer()
        self.setWindowTitle(name)

    def init_ui(self):
        center_widget = QWidget()
        self.setCentralWidget(center_widget)
        main_layout = QHBoxLayout()
        center_widget.setLayout(main_layout)
        self.glwidget = GLWidget()
        main_layout.addWidget(self.glwidget, 1)
        self.add_control_panel(main_layout)

    def add_control_panel(self, main_layout):
        """
        Override this function to add your own control panel to 
        the right of the main window.
        Don't forget add your own layout to the main_layout.
        """
        pass

    def add_update_timer(self):
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
