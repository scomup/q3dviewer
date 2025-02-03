#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QVBoxLayout, QListWidget, QListWidgetItem, QPushButton, QDoubleSpinBox, QCheckBox, QLineEdit, QMessageBox, QLabel, QHBoxLayout, QDockWidget, QWidget, QComboBox
from PySide6.QtCore import QTimer
from q3dviewer.tools.cloud_viewer import ProgressDialog, FileLoaderThread
from PySide6 import QtCore
from PySide6.QtGui import QKeyEvent
from q3dviewer import GLWidget
import imageio.v2 as imageio
import os


def recover_center_euler(Twc, dist):
    Rwc = Twc[:3, :3]  # Extract rotation
    twc = Twc[:3, 3]   # Extract translation
    tco = np.array([0, 0, dist])  # Camera frame origin
    two = twc - Rwc @ tco  # Compute center
    euler = q3d.matrix_to_euler(Rwc)
    return two, euler


class KeyFrame:
    def __init__(self, Twc, lin_vel=10, ang_vel=np.pi/3, stop_time=0):
        self.Twc = Twc
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel  # rad/s
        self.stop_time = stop_time
        self.item = q3d.FrameItem(Twc, width=3, color='#0000FF')


class CustomGLWidget(GLWidget):
    def __init__(self, viewer):
        super().__init__()
        self.viewer = viewer  # Add a viewer handle

    def keyPressEvent(self, ev: QKeyEvent):
        if ev.key() == QtCore.Qt.Key_Space:
            self.viewer.add_key_frame()
        elif ev.key() == QtCore.Qt.Key_Delete:
            self.viewer.del_key_frame()
        elif ev.key() == QtCore.Qt.Key_C:
            self.viewer.dock.show()
        super().keyPressEvent(ev)

class CMMViewer(q3d.Viewer):
    """
    This class is a subclass of Viewer, which is used to create a cloud movie maker.
    """
    def __init__(self, **kwargs):
        self.key_frames = []
        self.video_path = os.path.join(os.path.expanduser("~"), "output.mp4")
        super().__init__(**kwargs, gl_widget_class=lambda: CustomGLWidget(self))
        # for drop cloud file
        self.setAcceptDrops(True)

    def add_control_panel(self, main_layout):
        """
        Add a control panel to the viewer.
        """
        # Create a vertical layout for the settings
        setting_layout = QVBoxLayout()

        # Buttons to add and delete key frames
        add_button = QPushButton("Add Key Frame (Key Space)")
        add_button.clicked.connect(self.add_key_frame)
        setting_layout.addWidget(add_button)
        del_button = QPushButton("Delete Key Frame (Key Delete)")
        del_button.clicked.connect(self.del_key_frame)
        setting_layout.addWidget(del_button)

        # Add play/stop button
        self.play_button = QPushButton("Play")
        self.play_button.clicked.connect(self.toggle_playback)
        setting_layout.addWidget(self.play_button)

        # add a timer to play the frames
        self.timer = QTimer()
        self.timer.timeout.connect(self.play_frames)
        self.current_frame_index = 0
        self.is_playing = False
        self.is_recording = False

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

        # Add codec setting
        codec_layout = QHBoxLayout()
        label_codec = QLabel("Codec:")
        codec_layout.addWidget(label_codec)
        self.codec_combo = QComboBox()
        self.codec_combo.addItems(["libx264", "mjpeg", "mpeg4", "libx265"])
        codec_layout.addWidget(self.codec_combo)
        setting_layout.addLayout(codec_layout)

        # Add a list of key frames
        self.frame_list = QListWidget()
        setting_layout.addWidget(self.frame_list)
        self.frame_list.itemSelectionChanged.connect(self.on_select_frame)
        self.frame_list.itemDoubleClicked.connect(self.on_double_click_frame)
        self.installEventFilter(self)

        # Add spin boxes for linear / angular velocity and stop time
        self.lin_vel_spinbox = QDoubleSpinBox()
        self.lin_vel_spinbox.setPrefix("Linear Velocity (m/s): ")
        self.lin_vel_spinbox.setRange(0, 1000)
        self.lin_vel_spinbox.valueChanged.connect(self.set_frame_lin_vel)
        setting_layout.addWidget(self.lin_vel_spinbox)

        self.lin_ang_spinbox = QDoubleSpinBox()
        self.lin_ang_spinbox.setPrefix("Angular Velocity (deg/s): ")
        self.lin_ang_spinbox.setRange(0, 360)
        self.lin_ang_spinbox.valueChanged.connect(self.set_frame_ang_vel)
        setting_layout.addWidget(self.lin_ang_spinbox)

        self.stop_time_spinbox = QDoubleSpinBox()
        self.stop_time_spinbox.setPrefix("Stop Time: ")
        self.stop_time_spinbox.setRange(0, 100)
        self.stop_time_spinbox.valueChanged.connect(self.set_frame_stop_time)
        setting_layout.addWidget(self.stop_time_spinbox)
        
        setting_layout.setAlignment(QtCore.Qt.AlignTop)

        # Create a dock widget for the settings
        dock_widget = QDockWidget("Settings", self)
        dock_widget.setWidget(QWidget())
        dock_widget.widget().setLayout(setting_layout)
        # Hide close and undock buttons, I don't want the user to close the dock
        dock_widget.setFeatures(QDockWidget.DockWidgetMovable)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock_widget)

        # Add the dock widget to the main layout
        main_layout.addWidget(dock_widget)
        self.dock = dock_widget


    def update_video_path(self, path):
        self.video_path = path

    def add_key_frame(self):
        view_matrix = self.glwidget.view_matrix
        # Get camera pose in world frame
        Twc = np.linalg.inv(view_matrix)
        if self.key_frames:
            prev = self.key_frames[-1]
            key_frame = KeyFrame(Twc,
                                 lin_vel=prev.lin_vel, 
                                 ang_vel=prev.ang_vel,
                                 stop_time=prev.stop_time)
        else:
            key_frame = KeyFrame(Twc)
        self.key_frames.append(key_frame)
        # visualize this key frame using FrameItem
        self.glwidget.add_item(key_frame.item)
        # move the camera back to 0.5 meter, let the user see the frame
        # self.glwidget.update_dist(0.5)
        # Add the key frame to the Qt ListWidget
        item = QListWidgetItem(f"Frame {len(self.key_frames)}")
        self.frame_list.addItem(item)
        self.frame_list.setCurrentRow(len(self.key_frames) - 1)

    def del_key_frame(self):
        current_index = self.frame_list.currentRow()
        if current_index < 0:
            return
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
        if current < 0:
            return
        for i, frame in enumerate(self.key_frames):
            if i == current:
                # Highlight the selected frame
                frame.item.set_color('#FF0000')
                frame.item.set_line_width(5)
                # show current frame's parameters in the spinboxes
                self.lin_vel_spinbox.setValue(frame.lin_vel)
                self.lin_ang_spinbox.setValue(np.rad2deg(frame.ang_vel))
                self.stop_time_spinbox.setValue(frame.stop_time)
            else:
                frame.item.set_color('#0000FF')
                frame.item.set_line_width(3)

    def set_frame_lin_vel(self, value):
        current_index = self.frame_list.currentRow()
        if current_index < 0:
            return
        self.key_frames[current_index].lin_vel = value

    def set_frame_ang_vel(self, value):
        current_index = self.frame_list.currentRow()
        if current_index < 0:
            return
        self.key_frames[current_index].ang_vel = np.deg2rad(value)

    def set_frame_stop_time(self, value):
        current_index = self.frame_list.currentRow()
        if current_index < 0:
            return
        self.key_frames[current_index].stop_time = value

    def on_double_click_frame(self, item):
        current_index = self.frame_list.row(item)
        if current_index < 0:
            return
        Twc = self.key_frames[current_index].Twc
        center, euler = recover_center_euler(Twc, self.glwidget.dist)
        self.glwidget.set_cam_position(center=center,
                                       euler=euler)


    def create_frames(self):
        """
        Create the frames for playback by interpolating between key frames.
        """
        self.frames = []
        dt = 1 / float(self.update_interval)
        for i in range(len(self.key_frames) - 1):
            current_frame = self.key_frames[i]
            if current_frame.stop_time > 0:
                num_steps = int(current_frame.stop_time / dt)
                for j in range(num_steps):
                    self.frames.append([i, current_frame.Twc])
            next_frame = self.key_frames[i + 1]
            Ts = q3d.interpolate_pose(current_frame.Twc, next_frame.Twc,
                                      current_frame.lin_vel,
                                      current_frame.ang_vel,
                                      dt)
            for T in Ts:
                self.frames.append([i, T])
        
        print(f"Total frames: {len(self.frames)}")
        print(f"Total time: {len(self.frames) * dt:.2f} seconds")

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
            self.play_button.setStyleSheet("background-color: red")
            self.play_button.setText("Playing")
            self.record_checkbox.setEnabled(False)
            if self.is_recording is True:
                self.start_recording()

    def stop_playback(self):
        self.timer.stop()
        self.is_playing = False
        self.play_button.setStyleSheet("")
        self.play_button.setText("Play")
        self.record_checkbox.setEnabled(True)
        self.frame_list.setCurrentRow(len(self.key_frames) - 1)
        if self.is_recording:
            self.stop_recording()

    def play_frames(self):
        """
        callback function for the timer to play the frames
        """
        # play the frames
        if self.current_frame_index < len(self.frames):
            key_id, Tcw = self.frames[self.current_frame_index]
            self.glwidget.set_view_matrix(np.linalg.inv(Tcw))
            self.frame_list.setCurrentRow(key_id)
            self.current_frame_index += 1
            if self.is_recording:
                self.record_frame()
        else:
            self.stop_playback()

    def toggle_recording(self, state):
        if state == 2:
            self.is_recording = True
        else:
            self.is_recording = False

    def start_recording(self):
        self.is_recording = True
        self.prv_frame_shape = None
        video_path = self.video_path_edit.text()
        codec = self.codec_combo.currentText()
        self.play_button.setStyleSheet("background-color: red")
        self.play_button.setText("Recording")
        self.writer = imageio.get_writer(video_path, 
                                         fps=self.update_interval,
                                         codec=codec,
                                         quality=10,
                                         pixelformat='yuvj420p')
        # disable the all the frame_item while recording
        for frame in self.key_frames:
            frame.item.hide()
        self.dock.hide()  # Hide the dock while recording

    def stop_recording(self, save_movie=True):
        self.is_recording = False
        self.prv_frame_shape = None
        self.record_checkbox.setChecked(False)
        # enable the all the frame_item after recording
        for frame in self.key_frames:
            frame.item.show()
        if hasattr(self, 'writer') and save_movie:
            self.writer.close()
            self.show_save_message()
        self.dock.show()  # Show the dock when recording stops

    def show_save_message(self):
        msg_box = QMessageBox()
        msg_box.setIcon(QMessageBox.Information)
        msg_box.setWindowTitle("Video Saved")
        msg_box.setText(f"Video saved to {self.video_path_edit.text()}")
        msg_box.setStandardButtons(QMessageBox.Ok)
        msg_box.exec()

    def record_frame(self):
        frame = self.glwidget.capture_frame()
        
        # restart recording if the window size changes
        if self.prv_frame_shape is not None and frame.shape != self.prv_frame_shape:
            self.writer.close()
            self.start_recording()
            return
        self.prv_frame_shape = frame.shape 

        height, width, _ = frame.shape
        # Adjust frame dimensions to be multiples of 16
        new_height = height - (height % 16)
        new_width = width - (width % 16)
        frame = frame[:new_height, :new_width, :]
        frame = np.ascontiguousarray(frame)
        try:
            self.writer.append_data(frame)
        except Exception as e:
            # unexpected error, stop recording without saving
            print("Error while recording:")
            print(e)
            self.stop_recording(False)
            self.stop_playback()

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
        self.glwidget.set_cam_position(center=center)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="the cloud file path")
    args = parser.parse_args()
    app = q3d.QApplication(['Film Maker'])
    viewer = CMMViewer(name='Film Maker', update_interval=30)
    cloud_item = q3d.CloudIOItem(size=0.1, point_type='SPHERE', alpha=0.5, depth_test=True)
    grid_item = q3d.GridItem(size=1000, spacing=20)

    viewer.add_items(
        {'cloud': cloud_item, 'grid': grid_item})

    if args.path:
        pcd_fn = args.path
        viewer.open_cloud_file(pcd_fn)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
