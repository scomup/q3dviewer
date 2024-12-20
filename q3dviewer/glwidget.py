"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from PyQt5 import QtCore
from PyQt5.QtWidgets import QWidget, QComboBox, QVBoxLayout, QHBoxLayout, \
    QSizePolicy, QSpacerItem
from OpenGL.GL import *
from PyQt5.QtGui import QKeyEvent, QVector3D
import numpy as np
from PyQt5.QtWidgets import QLabel, QLineEdit
from PyQt5.QtCore import QRegularExpression
from PyQt5.QtGui import QRegularExpressionValidator
from q3dviewer.base_glwidget import BaseGLWidget
from PyQt5.QtWidgets import QCheckBox

class SettingWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.combo_items = QComboBox()
        self.combo_items.currentIndexChanged.connect(self.on_combo_selection)
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

    def on_combo_selection(self, index):
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


class GLWidget(BaseGLWidget):
    def __init__(self):
        self.followed_name = 'none'
        self.named_items = {}
        self.color_str = '#000000'
        self.followable_item_name = ['none']
        self.setting_window = SettingWindow()
        self.enable_show_center = True
        super(GLWidget, self).__init__()

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
        color_edit.setText(self.color_str)
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

        checkbox_show_center = QCheckBox("Show Center Point")
        checkbox_show_center.setChecked(self.enable_show_center)
        checkbox_show_center.stateChanged.connect(self.change_show_center)
        layout.addWidget(checkbox_show_center)

    def set_backgroud_color(self, color_str):
        try:
            color_flat = int(color_str[1:], 16)
            red = (color_flat >> 16) & 0xFF
            green = (color_flat >> 8) & 0xFF
            blue = color_flat & 0xFF
            self.color_str = color_str
            self.set_color([red, green, blue, 0])
        except ValueError:
            return

    def add_item_with_name(self, name, item):
        self.named_items.update({name: item})
        if (item.__class__.__name__ == 'AxisItem'):
            self.followable_item_name.append(name)
        self.setting_window.add_setting(name, item)
        super().add_item(item)

    def open_setting_window(self):
        if self.setting_window.isVisible():
            self.setting_window.raise_()

        else:
            self.setting_window.show()

    def change_show_center(self, state):
        self.enable_show_center = state == QtCore.Qt.Checked