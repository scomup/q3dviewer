#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QVBoxLayout, QListWidget, QListWidgetItem, QPushButton, QDoubleSpinBox, QCheckBox, QLineEdit, QMessageBox, QLabel, QHBoxLayout
from PySide6.QtCore import QTimer
from cloud_viewer import ProgressDialog,  FileLoaderThread
from PySide6 import QtCore
from PySide6.QtGui import QKeyEvent
from q3dviewer import GLWidget
import imageio.v2 as imageio
import os


class Frame:
    def __init__(self, Tcw, item):
        self.Twc = Tcw # from world to camera
        self.linear_velociy = 10
        self.angular_velocity = 1
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

class CMMViewer(q3d.Viewer):
    """
    This class is a subclass of Viewer, which is used to create a cloud movie maker.
    """
    def __init__(self, **kwargs):
        self.key_frames = []
        self.video_path = os.path.join(os.path.expanduser("~"), "output.mp4")
        super().__init__(**kwargs, gl_widget_class=lambda: CustomGLWidget(self))
        self.timer = QTimer()
        self.timer.timeout.connect(self.play_frames)
        self.current_frame_index = 0
        self.is_playing = False
        self.is_recording = False
        self.setAcceptDrops(True)

    def add_control_panel(self, main_layout):
        """
        Add a control panel to the viewer.
        """
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

        # Add record checkbox
        self.record_checkbox = QCheckBox("Record")
        self.record_checkbox.stateChanged.connect(self.toggle_recording)
        setting_layout.addWidget(self.record_checkbox)

        # Add video path setting
        video_path_layout = QHBoxLayout()
        label_video_path = QLabel("Video Path:")
        video_path_layout.addWidget(label_video_path)
        self.video_path_edit = QLineEdit()
        self.video_path_edit.setText(self.video_path)
        self.video_path_edit.textChanged.connect(self.update_video_path)
        video_path_layout.addWidget(self.video_path_edit)
        setting_layout.addLayout(video_path_layout)

        # Add a list of key frames
        self.frame_list = QListWidget()
        setting_layout.addWidget(self.frame_list)
        self.frame_list.itemSelectionChanged.connect(self.on_select_frame)
        self.installEventFilter(self)

        # Add spin boxes for linear / angular velocity and stop time
        self.lin_vel_spinbox = QDoubleSpinBox()
        self.lin_vel_spinbox.setPrefix("Linear Velocity (m/s): ")
        self.lin_vel_spinbox.setRange(0, 100)
        self.lin_vel_spinbox.valueChanged.connect(self.set_frame_lin_vel)
        setting_layout.addWidget(self.lin_vel_spinbox)

        self.lin_ang_spinbox = QDoubleSpinBox()
        self.lin_ang_spinbox.setPrefix("Angular Velocity (rad/s): ")
        self.lin_ang_spinbox.setRange(0, 100)
        self.lin_ang_spinbox.valueChanged.connect(self.set_frame_ang_vel)
        setting_layout.addWidget(self.lin_ang_spinbox)

        self.stop_time_spinbox = QDoubleSpinBox()
        self.stop_time_spinbox.setPrefix("Stop Time: ")
        self.stop_time_spinbox.setRange(0, 100)
        self.stop_time_spinbox.valueChanged.connect(self.set_frame_stop_time)
        setting_layout.addWidget(self.stop_time_spinbox)
        
        setting_layout.setAlignment(QtCore.Qt.AlignTop)
        main_layout.addLayout(setting_layout)

    def update_video_path(self, path):
        self.video_path = path

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
                self.lin_vel_spinbox.setValue(frame.linear_velociy)
                self.lin_ang_spinbox.setValue(frame.angular_velocity)
                self.stop_time_spinbox.setValue(frame.stop_time)
            else:
                frame.item.set_color('#0000FF')
                frame.item.set_line_width(3)

    def set_frame_lin_vel(self, value):
        current_index = self.frame_list.currentRow()
        if current_index >= 0:
            self.key_frames[current_index].linear_velociy = value

    def set_frame_ang_vel(self, value):
        current_index = self.frame_list.currentRow()
        if current_index >= 0:
            self.key_frames[current_index].angular_velocity = value

    def set_frame_stop_time(self, value):
        current_index = self.frame_list.currentRow()
        if current_index >= 0:
            self.key_frames[current_index].stop_time = value

    def create_frames(self):
        self.frames = []
        dt = 1 / float(self.update_interval)
        for i in range(len(self.key_frames) - 1):
            current_frame = self.key_frames[i]
            next_frame = self.key_frames[i + 1]
            Ts = q3d.interpolate_pose(current_frame.Twc, next_frame.Twc,
                                      current_frame.linear_velociy,
                                      current_frame.angular_velocity,
                                      dt)
            self.frames.extend(Ts)

    def toggle_playback(self):
        if self.is_playing:
            self.stop_playback()
        else:
            self.start_playback()

    def start_playback(self):
        if self.key_frames:
            self.create_frames()
            self.current_frame_index = 0
            self.timer.start(self.update_interval)  # Adjust the interval as needed
            self.is_playing = True
            self.play_button.setStyleSheet("")
            self.play_button.setText("Stop")
            self.record_checkbox.setEnabled(False)

    def stop_playback(self):
        self.timer.stop()
        self.is_playing = False
        self.play_button.setStyleSheet("")
        self.play_button.setText("Play")
        self.record_checkbox.setEnabled(True)

    def play_frames(self):
        if self.is_recording is True and self.current_frame_index == 0:
            self.start_recording()

        if self.current_frame_index < len(self.frames):
            self.glwidget.set_view_matrix(np.linalg.inv(self.frames[self.current_frame_index]))
            self.current_frame_index += 1
            if self.is_recording:
                self.record_frame()
        else:
            self.timer.stop()
            self.is_playing = False
            self.record_checkbox.setEnabled(True)
            if self.is_recording:
                self.stop_recording()

    def toggle_recording(self, state):
        if state == 2:
            self.is_recording = True
        else:
            self.is_recording = False

    def start_recording(self):
        self.is_recording = True
        self.frames_to_record = []
        video_path = self.video_path_edit.text()
        self.play_button.setStyleSheet("background-color: red")
        self.play_button.setText("Recording")
        self.writer = imageio.get_writer(video_path, fps=self.update_interval,
                                         codec="libx264", bitrate="5M", quality=10)

    def stop_recording(self):
        self.is_recording = False
        self.play_button.setStyleSheet("")
        self.play_button.setText("Play")
        self.record_checkbox.setChecked(False)
        if hasattr(self, 'writer'):
            self.writer.close()
            self.show_save_message()

    def show_save_message(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setWindowTitle("Video Saved")
        msg_box.setText(f"Video saved to {self.video_path_edit.text()}")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec()

    def record_frame(self):
        frame = self.glwidget.capture_frame()
        self.frames_to_record.append(frame)
        print(f"Recorded frame {len(self.frames_to_record)}")
        self.writer.append_data(frame)

    def eventFilter(self, obj, event):
        if event.type() == QtCore.QEvent.KeyPress:
            if event.key() == QtCore.Qt.Key_Delete:
                self.del_key_frame()
                return True
        return super().eventFilter(obj, event)

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        """
        Overwrite the drop event to open the cloud file.
        """
        self.progress_dialog = ProgressDialog(self)
        self.progress_dialog.show()
        files = event.mimeData().urls()
        self.progress_thread = FileLoaderThread(self, files)
        self['cloud'].load(files[0].toLocalFile(), append=False)
        self.progress_thread.progress.connect(self.file_loading_progress)
        self.progress_thread.finished.connect(self.file_loading_finished)
        self.progress_thread.start()

    def file_loading_progress(self, value):
        self.progress_dialog.set_value(value)

    def file_loading_finished(self):
        self.progress_dialog.close()

    def open_cloud_file(self, file, append=False):
        cloud_item = self['cloud']
        if cloud_item is None:
            print("Can't find clouditem.")
            return
        cloud = cloud_item.load(file, append=append)
        center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
        self.glwidget.set_cam_position(pos=center)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="the cloud file path")
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Movie Maker'])
    viewer = CMMViewer(name='Cloud Movie Maker', update_interval=30)
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
