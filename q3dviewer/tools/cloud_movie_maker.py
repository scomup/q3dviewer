#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QVBoxLayout, QLabel, QListWidget, QListWidgetItem, QPushButton, QWidget, QDoubleSpinBox
from PySide6.QtCore import QThread, Signal, QTimer
from cloud_viewer import ProgressDialog,  FileLoaderThread
from PySide6 import QtCore
from PySide6.QtGui import QKeyEvent
from q3dviewer import GLWidget


class Frame:
    def __init__(self, Tcw, item):
        self.Twc = Tcw # from world to camera
        self.velociy = 1
        self.stop_time = 0
        self.item = item


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
        self.key_frames = []
        super().__init__(**kwargs, gl_widget_class=lambda: CustomGLWidget(self))
        self.frame_list.itemSelectionChanged.connect(self.on_select_frame)
        self.installEventFilter(self)
        self.timer = QTimer()
        self.timer.timeout.connect(self.play_frames)
        self.current_frame_index = 0
        self.interpolation_steps = 0
        self.current_interpolation_step = 0
        self.is_playing = False

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

        # Add play/stop button
        self.play_button = QPushButton("Play")
        self.play_button.clicked.connect(self.toggle_playback)
        setting_layout.addWidget(self.play_button)

        # Add a list of key frames
        self.frame_list = QListWidget()
        setting_layout.addWidget(self.frame_list)

        # Add double spin boxes for velocity and stop time
        self.velocity_spinbox = QDoubleSpinBox()
        self.velocity_spinbox.setPrefix("Velocity: ")
        self.velocity_spinbox.setRange(0, 100)
        self.velocity_spinbox.valueChanged.connect(self.set_frame_velocity)
        setting_layout.addWidget(self.velocity_spinbox)

        self.stop_time_spinbox = QDoubleSpinBox()
        self.stop_time_spinbox.setPrefix("Stop Time: ")
        self.stop_time_spinbox.setRange(0, 100)
        self.stop_time_spinbox.valueChanged.connect(self.set_frame_stop_time)
        setting_layout.addWidget(self.stop_time_spinbox)
        
        setting_layout.setAlignment(QtCore.Qt.AlignTop)
        main_layout.addLayout(setting_layout)

    def add_key_frame(self):
        view_matrix = self.glwidget.view_matrix

        # Get camera pose in world frame
        Twc = np.linalg.inv(view_matrix)
        # add a FrameItem to the scene
        frame_item = q3d.FrameItem(Twc, width=3, color='#0000FF')
        self.glwidget.add_item(frame_item)
        # move the camera back to 0.5 meter, let the user see the frame
        self.glwidget.update_dist(0.5)

        # Add the key frame to the list
        current_index = self.frame_list.currentRow()
        frame = Frame(Twc, frame_item)
        self.key_frames.insert(current_index + 1, frame)
        item = QListWidgetItem(f"Frame {current_index + 2}")
        self.frame_list.insertItem(current_index + 1, item)
        self.frame_list.setCurrentRow(current_index + 1)
        # Update frame labels
        for i in range(len(self.key_frames)):
            self.frame_list.item(i).setText(f"Frame {i + 1}")

    def del_key_frame(self):
        current_index = self.frame_list.currentRow()
        if current_index >= 0:
            self.glwidget.remove_item(self.key_frames[current_index].item)
            self.key_frames.pop(current_index)
            self.frame_list.itemSelectionChanged.disconnect(self.on_select_frame)
            self.frame_list.takeItem(current_index)
            self.frame_list.itemSelectionChanged.connect(self.on_select_frame)
            self.on_select_frame()
        # Update frame labels
        for i in range(len(self.key_frames)):
            self.frame_list.item(i).setText(f"Frame {i + 1}")
    
    def on_select_frame(self):
        current = self.frame_list.currentRow()
        for i, frame in enumerate(self.key_frames):
            if i == current:
                frame.item.set_color('#FF0000')
                frame.item.set_line_width(5)
                self.velocity_spinbox.setValue(frame.velociy)
                self.stop_time_spinbox.setValue(frame.stop_time)
                self.velocity_spinbox.setValue(frame.velociy)
                self.stop_time_spinbox.setValue(frame.stop_time)
            else:
                frame.item.set_color('#0000FF')
                frame.item.set_line_width(3)

    def set_frame_velocity(self, value):
        current_index = self.frame_list.currentRow()
        if current_index >= 0:
            self.key_frames[current_index].velociy = value

    def set_frame_stop_time(self, value):
        current_index = self.frame_list.currentRow()
        if current_index >= 0:
            self.key_frames[current_index].stop_time = value

    def toggle_playback(self):
        if self.is_playing:
            self.stop_playback()
        else:
            self.start_playback()

    def start_playback(self):
        if self.key_frames:
            self.current_frame_index = 0
            self.current_interpolation_step = 0
            self.timer.start(100)  # Adjust the interval as needed
            self.is_playing = True
            self.play_button.setText("Stop")

    def stop_playback(self):
        self.timer.stop()
        self.is_playing = False
        self.play_button.setText("Play")

    def play_frames(self):
        if self.current_frame_index < len(self.key_frames) - 1:
            current_frame = self.key_frames[self.current_frame_index]
            next_frame = self.key_frames[self.current_frame_index + 1]
            self.interpolation_steps = int(100 / current_frame.velociy)  # Adjust the divisor as needed

            if self.current_interpolation_step < self.interpolation_steps:
                alpha = self.current_interpolation_step / self.interpolation_steps
                interpolated_Twc = self.interpolate_matrices(current_frame.Twc, next_frame.Twc, alpha)
                self.glwidget.set_view_matrix(np.linalg.inv(interpolated_Twc))
                self.current_interpolation_step += 1
            else:
                self.current_frame_index += 1
                self.current_interpolation_step = 0
        else:
            self.timer.stop()

    def interpolate_matrices(self, Twc1, Twc2, alpha):
        return (1 - alpha) * Twc1 + alpha * Twc2

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
