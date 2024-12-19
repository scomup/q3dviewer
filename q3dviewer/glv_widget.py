"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from q3dviewer.base_item import BaseItem
from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QComboBox, QVBoxLayout, QHBoxLayout, \
    QSizePolicy, QSpacerItem, QMainWindow
from OpenGL.GL import *
from PyQt5.QtGui import QKeyEvent, QVector3D
from PyQt5.QtWidgets import QApplication, QWidget
import numpy as np
from PyQt5.QtWidgets import QLabel, QLineEdit, \
    QDoubleSpinBox, QSpinBox, QCheckBox
from PyQt5.QtCore import QRegularExpression
from PyQt5.QtGui import QRegularExpressionValidator
from q3dviewer.base_glwidget import BaseGLWidget

class SettingWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.combo_items = QComboBox()
        self.combo_items.currentIndexChanged.connect(self.onComboSelection)
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

    def clear_setting(self):
        while self.layout.count():
            child = self.layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def onComboSelection(self, index):
        self.layout.removeItem(self.stretch)
        # remove all setting of previous widget
        self.clear_setting()

        key = list(self.items.keys())
        item = self.items[key[index]]
        if hasattr(item, "add_setting"):
            item.add_setting(self.layout)
            self.layout.addItem(self.stretch)
        else:
            print("%s: No setting." % (item.__class__.__name__))


class GLVWidget(BaseGLWidget):
    def __init__(self):
        self.followed_name = 'none'
        self.named_items = {}
        self.color = '#000000'
        self.followable_item_name = ['none']
        self.setting_window = SettingWindow()
        super(GLVWidget, self).__init__()

    def keyPressEvent(self, ev: QKeyEvent):
        if ev.key() == QtCore.Qt.Key_M:  # setting meun
            print("Open setting windows")
            self.open_setting_window()
        super().keyPressEvent(ev)

    def on_followable_selection(self, index):
        self.followed_name = self.followable_item_name[index]

    def update(self):
        if self.followed_name != 'none':
            pos = self.named_items[self.followed_name].T[:3, 3]
            self.opts['center'] = QVector3D(pos[0], pos[1], pos[2])
        super().update()

    def add_setting(self, layout):
        label_color = QLabel("Set background color:")
        layout.addWidget(label_color)
        color_edit = QLineEdit()
        color_edit.setToolTip("'using hex color, i.e. #FF4500")
        color_edit.setText(self.color)
        color_edit.textChanged.connect(self.set_backgroud_color)
        regex = QRegularExpression(r"^#[0-9A-Fa-f]{6}$")
        validator = QRegularExpressionValidator(regex)
        color_edit.setValidator(validator)
        layout.addWidget(color_edit)
        label_focus = QLabel("Set Focus:")
        combo_focus = QComboBox()
        for name in self.followable_item_name:
            combo_focus.addItem(name)
        combo_focus.currentIndexChanged.connect(self.on_followable_selection)
        layout.addWidget(label_focus)
        layout.addWidget(combo_focus)

    def set_backgroud_color(self, color):
        try:
            self.set_color(color)
            self.color = color
        except ValueError:
            return

    def add_item_with_name(self, name, item):
        self.named_items.update({name: item})
        if (item.__class__.__name__ == 'GLAxisItem'):
            self.followable_item_name.append(name)
        self.setting_window.add_setting(name, item)
        super().add_item(item)

    def open_setting_window(self):
        if self.setting_window.isVisible():
            self.setting_window.raise_()

        else:
            self.setting_window.show()
