#!/usr/bin/env python3

from q3dviewer.viewer_widget import *
import signal
import sys


def handler(signal, frame):
    QApplication.quit()
    print("Force close by Ctrl+C")


class Viewer(QMainWindow):
    def __init__(self, name='Viewer', win_size=[1920, 1080], vw=ViewWidget):
        signal.signal(signal.SIGINT, handler)
        super(Viewer, self).__init__()
        self.vw = vw
        self.setGeometry(0, 0, win_size[0], win_size[1])
        self.init_ui()
        self.setWindowTitle(name)

    def init_ui(self):
        centerWidget = QWidget()
        self.setCentralWidget(centerWidget)
        layout = QVBoxLayout()
        centerWidget.setLayout(layout)
        self.viewerWidget = self.vw()
        layout.addWidget(self.viewerWidget, 1)
        timer = QtCore.QTimer(self)
        timer.setInterval(20)  # period, in milliseconds
        timer.timeout.connect(self.update)
        self.viewerWidget.setCameraPosition(distance=40)
        timer.start()

    def add_items(self, named_items: dict):
        for name, item in named_items.items():
            self.viewerWidget.addItem(name, item)

    def __getitem__(self, name: str):
        if name in self.viewerWidget.named_items:
            return self.viewerWidget.named_items[name]
        else:
            return None

    def update(self):
        # force update by timer
        self.viewerWidget.update()

    def closeEvent(self, event):
        event.accept()
        QApplication.quit()

    def show(self):
        self.viewerWidget.setting_window.add_setting(
            "main win", self.viewerWidget)
        super().show()
