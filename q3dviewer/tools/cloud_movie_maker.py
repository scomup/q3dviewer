#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QVBoxLayout, QLabel, QListWidget, QListWidgetItem, QPushButton, QWidget
from PySide6.QtCore import QThread, Signal
from cloud_viewer import ProgressDialog,  FileLoaderThread
from PySide6 import QtCore
from PySide6.QtGui import QKeyEvent
from q3dviewer import GLWidget


class Frame:
    def __init__(self, id, Twc, velociy):
        self.id = id
        self.Twc = Twc # from world to camera
        self.velociy = velociy
        self.stop_time = 0


class CustomGLWidget(GLWidget):
    def __init__(self, viewer):
        super().__init__()
        self.viewer = viewer  # Add a viewer handle

    def keyPressEvent(self, ev: QKeyEvent):
        if ev.key() == QtCore.Qt.Key_Space:
            self.viewer.add_key_frame()
        elif ev.key() == QtCore.Qt.Key_Backspace:
            self.viewer.del_key_frame()
        super().keyPressEvent(ev)

class Viewer2(q3d.Viewer):
    def __init__(self, **kwargs):
        self.frames = []
        super().__init__(**kwargs, gl_widget_class=lambda: CustomGLWidget(self))

    def add_control_panel(self, main_layout):
        # Create a vertical layout for the settings
        setting_layout = QVBoxLayout()

        # Buttons to add and delete key frames
        add_button = QPushButton("Add Key Frame")
        add_button.clicked.connect(self.add_key_frame)
        setting_layout.addWidget(add_button)
        del_button = QPushButton("Delete Key Frame")
        del_button.clicked.connect(self.del_key_frame)
        setting_layout.addWidget(del_button)

        # Add a list of key frames
        self.frame_list = QListWidget()
        setting_layout.addWidget(self.frame_list)
        
        setting_layout.setAlignment(QtCore.Qt.AlignTop)
        main_layout.addLayout(setting_layout)

    def add_key_frame(self):
        view_matrix = self.glwidget.get_view_matrix()
        current_index = self.frame_list.currentRow()
        frame = Frame(len(self.frames), view_matrix, 0)
        
        if current_index >= 0:
            self.frames.insert(current_index + 1, frame)
            item = QListWidgetItem(f"Frame {current_index + 2}")
            self.frame_list.insertItem(current_index + 1, item)
            self.frame_list.setCurrentRow(current_index + 1)
        else:
            self.frames.append(frame)
            item = QListWidgetItem(f"Frame {len(self.frames)}")
            self.frame_list.addItem(item)
            self.frame_list.setCurrentRow(len(self.frames) - 1)  # Set to the newly added one

        # Update frame labels
        for i in range(len(self.frames)):
            self.frame_list.item(i).setText(f"Frame {i + 1}")

    def del_key_frame(self):
        current_index = self.frame_list.currentRow()
        if current_index >= 0:
            self.frames.pop(current_index)
            self.frame_list.takeItem(current_index)

        # Update frame labels
        for i in range(len(self.frames)):
            self.frame_list.item(i).setText(f"Frame {i + 1}")


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="the cloud file path")
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Viewer'])
    viewer = Viewer2(name='Cloud Viewer')
    cloud_item = q3d.CloudIOItem(size=1, alpha=0.1)
    axis_item = q3d.AxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=1000, spacing=20)

    viewer.add_items(
        {'cloud': cloud_item, 'grid': grid_item, 'axis': axis_item})

    if args.path:
        pcd_fn = args.path
        viewer.open_cloud_file(pcd_fn)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
