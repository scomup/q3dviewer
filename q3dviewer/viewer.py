#!/usr/bin/env python3
"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.glwidget import *
import signal
from pathlib import Path
from q3dviewer.Qt.QtWidgets import QMainWindow, QApplication, QHBoxLayout


class Viewer(QMainWindow):
    def __init__(self, name='Viewer', win_size=[1920, 1080], 
                 gl_widget_class=GLWidget, update_interval=20):
        self.set_quit_handler()
        super(Viewer, self).__init__()
        self.viewer_name = name
        self.setGeometry(0, 0, win_size[0], win_size[1])
        self.gl_widget_class = gl_widget_class
        self.init_ui()
        self.update_interval = update_interval
        self.add_update_timer()
        self.setWindowTitle(name)
        self.installEventFilter(self)

    def set_quit_handler(self, handler=None):
        if handler is None:
            handler = signal.SIG_DFL
        signal.signal(signal.SIGINT, handler)


    def init_ui(self):
        center_widget = QWidget()
        self.setCentralWidget(center_widget)
        main_layout = QHBoxLayout()
        self.add_control_panel(main_layout)
        center_widget.setLayout(main_layout)
        self.glwidget = self.gl_widget_class()
        setting_name = self.viewer_name.lower().replace(' ', '_')
        self.glwidget.set_setting_path(
            Path.home() / '.config' / 'q3dviewer' / setting_name / 'setting.json')
        main_layout.addWidget(self.glwidget, 1)

    def add_control_panel(self, main_layout):
        """
        Override this function to add your own control panel to 
        the left side of the main window.
        Don't forget add your own layout to the main_layout.
        """
        pass

    def set_setting_path(self, path):
        self.glwidget.set_setting_path(path)

    def add_update_timer(self):
        timer = QtCore.QTimer(self)
        timer.setInterval(self.update_interval)  # period, in milliseconds
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
        self.glwidget.load_setting()
        self.glwidget.setting_window.add_setting(
            "main_win", self.glwidget)
        super().show()
