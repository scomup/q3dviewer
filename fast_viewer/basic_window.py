#!/usr/bin/env python3

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QWidget, QComboBox, QVBoxLayout, QSizePolicy,\
      QSpacerItem, QMainWindow
from OpenGL.GL import *
from PyQt5.QtGui import QKeyEvent, QVector3D
from PyQt5.QtWidgets import QApplication, QWidget
import numpy as np
import signal
import sys


class SettingWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.combo_items = QComboBox()
        self.combo_items.currentIndexChanged.connect(self.onComboboxSelection)
        main_layout = QVBoxLayout()
        self.stretch = QSpacerItem(10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding)
        main_layout.addWidget(self.combo_items)
        self.layout = QVBoxLayout()
        self.layout.addItem(self.stretch)
        main_layout.addLayout(self.layout)
        self.setLayout(main_layout)
        self.setWindowTitle("Setting Window")
        self.setGeometry(200, 200, 300, 200)
        self.items = {}

    def addSetting(self, name, item):
        self.items.update({name: item})
        self.combo_items.addItem("%s(%s)" % (name, item.__class__.__name__))

    def clearSetting(self):
        while self.layout.count():
            child = self.layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def onComboboxSelection(self, index):
        self.layout.removeItem(self.stretch)

        # remove all setting of previous widget
        self.clearSetting()

        key = list(self.items.keys())
        try:
            item = self.items[key[index]]
            item.addSetting(self.layout)
            self.layout.addItem(self.stretch)
        except AttributeError:
            print("%s: No setting." % (item.__class__.__name__))


class ViewWidget(gl.GLViewWidget):
    def __init__(self):
        super(ViewWidget, self).__init__()
        self.setting_window = SettingWindow()
        self.named_items = {}

    def addItem(self, name, item):
        self.named_items.update({name: item})
        self.setting_window.addSetting(name, item)
        super().addItem(item)

    def mouseReleaseEvent(self, ev):
        if hasattr(self, 'mousePos'):
            delattr(self, 'mousePos')

    def mouseMoveEvent(self, ev):
        lpos = ev.localPos()
        if not hasattr(self, 'mousePos'):
            self.mousePos = lpos
        diff = lpos - self.mousePos
        self.mousePos = lpos
        if ev.buttons() == QtCore.Qt.MouseButton.RightButton:
            self.orbit(-diff.x(), diff.y())
        elif ev.buttons() == QtCore.Qt.MouseButton.LeftButton:
            pitch_abs = np.abs(self.opts['elevation'])
            camera_mode = 'view-upright'
            if(pitch_abs <= 45.0 or pitch_abs == 90):
                camera_mode = 'view'
            self.pan(diff.x(), diff.y(), 0, relative=camera_mode)

    def keyPressEvent(self, event: QKeyEvent):
        step = 10
        zoom_delta = 20
        self.projectionMatrix().data()

        pitch_abs = np.abs(self.opts['elevation'])
        camera_mode = 'view-upright'
        if(pitch_abs <= 45.0 or pitch_abs == 90):
            camera_mode = 'view'

        if event.key() == QtCore.Qt.Key_M:  # setting meun
            print("Open setting windows")
            self.openSettingWindow()
        elif event.key() == QtCore.Qt.Key_R:
            print("Clear viewer")
            for item in self.named_items.values():
                try:
                    item.clear()
                except:
                    pass
        elif event.key() == QtCore.Qt.Key_W:
            self.pan(0, +step, 0, relative=camera_mode)
        elif event.key() == QtCore.Qt.Key_S:
            self.pan(0, -step, 0, relative=camera_mode)
        elif event.key() == QtCore.Qt.Key_A:
            self.pan(+step, 0, 0, relative=camera_mode)
        elif event.key() == QtCore.Qt.Key_D:
            self.pan(-step, 0, 0, relative=camera_mode)
        elif event.key() == QtCore.Qt.Key_Z:
            self.opts['distance'] *= 0.999**(+zoom_delta)
        elif event.key() == QtCore.Qt.Key_X:
            self.opts['distance'] *= 0.999**(-zoom_delta)
        else:
            super().keyPressEvent(event)

    def openSettingWindow(self):
        if self.setting_window.isVisible():
            self.setting_window.raise_()

        else:
            self.setting_window.show()


class Viewer(QMainWindow):
    def __init__(self, name='Viewer', win_size=[1920, 1080], vw=ViewWidget):
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        super(Viewer, self).__init__()
        self.vw = vw
        self.setGeometry(0, 0, win_size[0], win_size[1])
        self.initUI()
        self.setWindowTitle(name)

    def initUI(self):
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

    def addItems(self, named_items: dict):
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

    def closeEvent(self, _):
        sys.exit(0)
