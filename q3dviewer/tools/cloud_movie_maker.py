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
    def __init__(self, Tcw, velociy):
        self.Twc = Tcw # from world to camera
        self.velociy = velociy
        self.stop_time = 0


class CustomGLWidget(GLWidget):
    def __init__(self, viewer):
        super().__init__()
        self.viewer = viewer  # Add a viewer handle

    def keyPressEvent(self, ev: QKeyEvent):
        if ev.key() == QtCore.Qt.Key_Space:
            self.viewer.add_key_frame()
        elif ev.key() == QtCore.Qt.Key_Delete:
            self.viewer.del_key_frame()
        super().keyPressEvent(ev)

class Viewer2(q3d.Viewer):
    def __init__(self, **kwargs):
        self.frames = []
        self.frames_item = []
        super().__init__(**kwargs, gl_widget_class=lambda: CustomGLWidget(self))
        self.frame_list.itemSelectionChanged.connect(self.change_frame_color)
        self.installEventFilter(self)

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
        view_matrix = self.glwidget.view_matrix

        # Get camera pose in world frame
        Twc = np.linalg.inv(view_matrix)
        # add a FrameItem to the scene
        frame_item = q3d.FrameItem(Twc, width=3, img=None, color=(0, 0, 1))
        self.glwidget.add_item(frame_item)
        self.frames_item.append(frame_item)
        # move the camera back to 0.5 meter, let the user see the frame
        self.glwidget.update_dist(0.5)

        # Add the key frame to the list
        current_index = self.frame_list.currentRow()
        frame = Frame(Twc, 0)
        self.frames.insert(current_index + 1, frame)
        item = QListWidgetItem(f"Frame {current_index + 2}")
        self.frame_list.insertItem(current_index + 1, item)
        self.frame_list.setCurrentRow(current_index + 1)
        # Update frame labels
        for i in range(len(self.frames)):
            self.frame_list.item(i).setText(f"Frame {i + 1}")

    def del_key_frame(self):
        current_index = self.frame_list.currentRow()
        if current_index >= 0:
            self.frames.pop(current_index)
            self.glwidget.remove_item(self.frames_item[current_index])
            self.frames_item.pop(current_index)
            self.frame_list.itemSelectionChanged.disconnect(self.change_frame_color)
            self.frame_list.takeItem(current_index)
            self.frame_list.itemSelectionChanged.connect(self.change_frame_color)
            self.change_frame_color()
        # Update frame labels
        for i in range(len(self.frames)):
            self.frame_list.item(i).setText(f"Frame {i + 1}")
    

    def change_frame_color(self):
        current = self.frame_list.currentRow()
        for i, item in enumerate(self.frames_item):
            if i == current:
                item.set_color([1.0, 0.0, 0.0])
                item.set_line_width(5)
            else:
                item.set_color([0.0, 0.0, 1.0])
                item.set_line_width(3)

    def eventFilter(self, obj, event):
        if event.type() == QtCore.QEvent.KeyPress:
            if event.key() == QtCore.Qt.Key_Delete:
                self.del_key_frame()
                return True
        return super().eventFilter(obj, event)


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
