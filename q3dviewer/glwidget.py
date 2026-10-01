"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.Qt import QtCore
from q3dviewer.Qt.QtWidgets import QWidget, QComboBox, QVBoxLayout, QLabel, QLineEdit, QCheckBox, QGroupBox
from q3dviewer.Qt.QtGui import QKeyEvent
from q3dviewer.base_glwidget import BaseGLWidget
from q3dviewer.utils import text_to_rgba
import numpy as np
import json
import numpy as np
from pathlib import Path


SETTING_PATH = Path.home() / ".config" / "q3dviewer" / "cloud_viewer" / "setting.json"


class SettingWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.combo_items = QComboBox()
        self.combo_items.currentIndexChanged.connect(self.on_combo_selection)
        main_layout = QVBoxLayout()
        main_layout.setAlignment(QtCore.Qt.AlignTop)
        main_layout.addWidget(self.combo_items)
        self.layout = QVBoxLayout()
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
        # remove all setting of previous widget
        self.clear_setting()
        key = list(self.items.keys())
        item = self.items[key[index]]
        group_box = QGroupBox()
        group_layout = QVBoxLayout()
        item.add_setting(group_layout)
        group_box.setLayout(group_layout)
        self.layout.addWidget(group_box)


class GLWidget(BaseGLWidget):
    def __init__(self):
        self.followed_name = 'none'
        self.named_items = {}
        self.color_str = 'black'
        self.followable_item_name = None
        self.setting_window = SettingWindow()
        self.enable_show_center = True
        self.old_center = None
        super(GLWidget, self).__init__()

    def keyPressEvent(self, ev: QKeyEvent):
        if ev.modifiers() & QtCore.Qt.ControlModifier:
            if ev.key() == QtCore.Qt.Key_S:
                self.save_setting()
                ev.accept()
                return
            if ev.key() == QtCore.Qt.Key_L:
                self.load_setting()
                ev.accept()
                return

        if ev.key() == QtCore.Qt.Key_M:  # setting menu
            print("Open setting windows")
            self.open_setting_window()            
        else:
            super().keyPressEvent(ev)
        if ev.key() == QtCore.Qt.Key_F:  # reset follow
            if self.followable_item_name is None:
                self.initial_followable()

            if self.followed_name != 'none':
                self.followed_name = 'none'
                print("Reset follow.")
            elif len(self.followable_item_name) > 1:
                self.followed_name = self.followable_item_name[1]
                print("Set follow to ", self.followed_name)
            else:
                pass # do nothing

    def on_followable_selection(self, index):
        self.followed_name = self.followable_item_name[index]

    def mouseDoubleClickEvent(self, event):
        """Double click to set center."""
        p = self.get_point(event.x(), event.y())
        if p is not None:
            self.set_center(p)
        super().mouseDoubleClickEvent(event)

    def follow_odom(self):
        if self.followed_name != 'none':
            new_center = self.named_items[self.followed_name].T[:3, 3]
            if self.old_center is None:
                self.old_center = new_center.copy()
            else:
                delta = new_center - self.old_center
                self.set_center(self.center + delta)
                self.old_center = new_center.copy()

    def update(self):
        self.follow_odom()
        super().update()

    def add_setting(self, layout):
        label_color = QLabel("Set background color:")
        layout.addWidget(label_color)
        color_edit = QLineEdit()
        color_edit.setToolTip("'using hex color, i.e. #FF4500")
        color_edit.setText(self.color_str)
        color_edit.textChanged.connect(self.set_bg_color)
        layout.addWidget(color_edit)
        
        label_focus = QLabel("Set Focus:")
        combo_focus = QComboBox()
    
        if self.followable_item_name is None:
            self.initial_followable()

        for name in self.followable_item_name:
            combo_focus.addItem(name)
        combo_focus.currentIndexChanged.connect(self.on_followable_selection)
        layout.addWidget(label_focus)
        layout.addWidget(combo_focus)

        checkbox_show_center = QCheckBox("Show Center Point")
        checkbox_show_center.setChecked(self.enable_show_center)
        checkbox_show_center.stateChanged.connect(self.change_show_center)
        layout.addWidget(checkbox_show_center)

    def initial_followable(self):
        self.followable_item_name = ['none']
        for name, item in self.named_items.items():
            if item.__class__.__name__ == 'AxisItem' and not item._disable_setting:
                self.followable_item_name.append(name)

    def set_bg_color(self, color):
        try:
            self.color_str = color
            red, green, blue, alpha = text_to_rgba(color)
            self.set_color([red, green, blue, alpha])
            self.need_force_update = True
            print(f"Background color set to {color}")
        except ValueError:
            print("Invalid color format. Use mathplotlib color format.")

    def add_item_with_name(self, name, item):
        self.named_items.update({name: item})
        if not item._disable_setting:
            self.setting_window.add_setting(name, item)
        super().add_item(item)

    def open_setting_window(self):
        if self.setting_window.isVisible():
            self.setting_window.raise_()

        else:
            self.setting_window.show()

    def change_show_center(self, state):
        self.enable_show_center = state

    def save_setting(self):
        print("Saving settings...")
        SETTING_PATH.parent.mkdir(parents=True, exist_ok=True)
        setting = {
            'main_win': self.get_camera_pose(),
            'items': {
                name: item.save_setting()
                for name, item in self.named_items.items()
            },
        }
        with SETTING_PATH.open('w', encoding='utf-8') as file:
            json.dump(setting, file, indent=2)

    def load_setting(self):
        if not SETTING_PATH.exists():
            return
        print("Loading settings...")
        with SETTING_PATH.open('r', encoding='utf-8') as file:
            setting = json.load(file)

        main_win = setting.get('main_win')
        if main_win is not None:
            self.set_camera_pose(main_win)

        for name, item_setting in setting.get('items', {}).items():
            item = self.named_items.get(name)
            if item is not None:
                item.load_setting(item_setting)

    def get_camera_pose(self):
        """Get current camera pose parameters"""
        camera_pose = {
            'center': self.center.tolist(),
            'euler': self.euler.tolist(),
            'distance': float(self.dist),
        }
        return camera_pose

    def set_camera_pose(self, config):
        """Set camera pose from parameters"""
        if 'center' in config and 'euler' in config and 'distance' in config:
            self.set_center(np.asarray(config['center'], dtype=float))
            self.set_euler(np.asarray(config['euler'], dtype=float))
            self.set_dist(config['distance'])
        else:
            print("Invalid camera pose config")