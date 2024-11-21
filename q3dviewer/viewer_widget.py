"""
Copyright 2024  Liu Yang
Distributed under MIT license. See LICENSE for more information.
"""

#!/usr/bin/env python3

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore
from PyQt5.QtWidgets import QWidget, QComboBox, QVBoxLayout, QHBoxLayout, \
    QSizePolicy, QSpacerItem, QMainWindow
from OpenGL.GL import *
from PyQt5.QtGui import QKeyEvent, QVector3D
from PyQt5.QtWidgets import QApplication, QWidget
import numpy as np
from PyQt5.QtWidgets import QLabel, QLineEdit, \
    QDoubleSpinBox, QSpinBox, QCheckBox


class SettingWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.combo_items = QComboBox()
        self.combo_items.currentIndexChanged.connect(self.onComboboxSelection)
        main_layout = QVBoxLayout()
        self.stretch = QSpacerItem(
            10, 10, QSizePolicy.Minimum, QSizePolicy.Expanding)
        main_layout.addWidget(self.combo_items)
        self.layout = QVBoxLayout()
        self.layout.addItem(self.stretch)
        main_layout.addLayout(self.layout)
        self.setLayout(main_layout)
        self.setWindowTitle("Setting Window")
        self.setGeometry(200, 200, 300, 200)
        self.items = {}

    def add_setting(self, name, item):
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
            item.add_setting(self.layout)
            self.layout.addItem(self.stretch)
        except AttributeError:
            print("%s: No setting." % (item.__class__.__name__))


class ViewWidget(gl.GLViewWidget):
    def __init__(self):
        self.followed_name = 'none'
        self.named_items = {}
        self.color = '#000000'
        self.followable_item_name = ['none']
        self.setting_window = SettingWindow()
        super(ViewWidget, self).__init__()

    def on_followable_selection(self, index):
        self.followed_name = self.followable_item_name[index]

    def update(self):
        if self.followed_name != 'none':
            pos = self.named_items[self.followed_name].T[:3, 3]
            self.opts['center'] = QVector3D(pos[0], pos[1], pos[2])
        super().update()

    def add_setting(self, layout):
        label1 = QLabel("Set background color:")
        label1.setToolTip("using '#xxxxxx', i.e. #FF4500")
        box1 = QLineEdit()
        box1.setToolTip("'using '#xxxxxx', i.e. #FF4500")
        box1.setText(str(self.color))
        box1.textChanged.connect(self.set_bk_color)
        layout.addWidget(label1)
        layout.addWidget(box1)
        label2 = QLabel("Set Focus:")
        combo2 = QComboBox()
        for name in self.followable_item_name:
            combo2.addItem(name)
        combo2.currentIndexChanged.connect(self.on_followable_selection)
        layout.addWidget(label2)
        layout.addWidget(combo2)

    def set_bk_color(self, color):
        if (type(color) != str):
            return
        if color.startswith("#"):
            try:
                self.setBackgroundColor(color)
                self.color = color
            except ValueError:
                return

    def add_item(self, name, item):
        self.named_items.update({name: item})
        if (item.__class__.__name__ == 'GLAxisItem'):
            self.followable_item_name.append(name)
        self.setting_window.add_setting(name, item)
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
            if (pitch_abs <= 45.0 or pitch_abs == 90):
                camera_mode = 'view'
            self.pan(diff.x(), diff.y(), 0, relative=camera_mode)

    def keyPressEvent(self, ev: QKeyEvent):
        step = 10
        zoom_delta = 20
        speed = 2
        self.projectionMatrix().data()

        pitch_abs = np.abs(self.opts['elevation'])
        camera_mode = 'view-upright'
        if (pitch_abs <= 45.0 or pitch_abs == 90):
            camera_mode = 'view'

        if ev.key() == QtCore.Qt.Key_M:  # setting meun
            print("Open setting windows")
            self.open_setting_window()
        elif ev.key() == QtCore.Qt.Key_R:
            print("Clear viewer")
            for item in self.named_items.values():
                try:
                    item.clear()
                except AttributeError:
                    pass
        elif ev.key() == QtCore.Qt.Key_Up:
            if ev.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier:
                self.pan(0, +step, 0, relative=camera_mode)
            else:
                self.orbit(azim=0, elev=-speed)
        elif ev.key() == QtCore.Qt.Key_Down:
            if ev.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier:
                self.pan(0, -step, 0, relative=camera_mode)
            else:
                self.orbit(azim=0, elev=speed)
        elif ev.key() == QtCore.Qt.Key_Left:
            if ev.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier:
                self.pan(+step, 0, 0, relative=camera_mode)
            else:
                self.orbit(azim=speed, elev=0)

        elif ev.key() == QtCore.Qt.Key_Right:
            if ev.modifiers() & QtCore.Qt.KeyboardModifier.ControlModifier:
                self.pan(-step, 0, 0, relative=camera_mode)
            else:
                self.orbit(azim=-speed, elev=0)

        elif ev.key() == QtCore.Qt.Key_Z:
            self.opts['distance'] *= 0.999**(+zoom_delta)
        elif ev.key() == QtCore.Qt.Key_X:
            self.opts['distance'] *= 0.999**(-zoom_delta)
        else:
            super().keyPressEvent(ev)

    def open_setting_window(self):
        if self.setting_window.isVisible():
            self.setting_window.raise_()

        else:
            self.setting_window.show()