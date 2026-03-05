#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from sensor_msgs.msg import PointCloud2
import rospy
import numpy as np
import argparse
import q3dviewer as q3d
from q3dviewer.Qt.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, QSpinBox, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QGroupBox
from q3dviewer.Qt import QtCore
from q3dviewer.utils.convert_ros_msg import convert_pointcloud2_msg
from q3dviewer.utils.maths import matrix_to_quaternion, euler_to_matrix, matrix_to_euler
from q3dviewer.utils.cloud_registration import matching


try:
    import open3d as o3d
except ImportError:
    print("\033[91mWarning: open3d is not installed. Please install it to use this tool.\033[0m")
    print("\033[93mYou can install it using: pip install open3d\033[0m")
    exit(1)


viewer = None
cloud0_accum = None
clouds0 = []
cloud1_accum = None
clouds1 = []


class CustomDoubleSpinBox(QDoubleSpinBox):
    def __init__(self, decimals=4, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.decimals = decimals
        self.setDecimals(self.decimals)

    def textFromValue(self, value):
        return f"{value:.{self.decimals}f}"

    def valueFromText(self, text):
        # Remove prefix and suffix to extract the numeric value
        text = text.replace(self.prefix(), '').replace(self.suffix(), '').strip()
        return float(text)


class LiDARCalibViewer(q3d.Viewer):
    def __init__(self, **kwargs):
        self.t01 = np.array([0, 0, 0])
        self.R01 = np.eye(3)
        self.cloud_num = 10
        self.voxel_size = 0.1  # Voxel size for downsampling
        self.normal_radius = 0.1  # Radius for normal estimation
        self.icp_distance = 0.2  # Max correspondence distance for ICP
        super().__init__(**kwargs)

    def default_gl_setting(self, glwidget):
        # Set camera position and background color
        glwidget.set_bg_color('#ffffff')
        glwidget.set_cam_position(distance=5)
    
    def add_control_panel(self, main_layout):
        # Create a vertical layout for the settings
        setting_layout = QVBoxLayout()
        setting_layout.setAlignment(QtCore.Qt.AlignTop)
        # Add XYZ spin boxes
        label_xyz = QLabel("Set XYZ:")
        setting_layout.addWidget(label_xyz)
        self.box_x = CustomDoubleSpinBox()
        self.box_x.setSingleStep(0.01)
        setting_layout.addWidget(self.box_x)
        self.box_y = CustomDoubleSpinBox()
        self.box_y.setSingleStep(0.01)
        setting_layout.addWidget(self.box_y)
        self.box_z = CustomDoubleSpinBox()
        self.box_z.setSingleStep(0.01)
        setting_layout.addWidget(self.box_z)
        self.box_x.setRange(-100.0, 100.0)
        self.box_y.setRange(-100.0, 100.0)
        self.box_z.setRange(-100.0, 100.0)

        # Add RPY spin boxes
        label_rpy = QLabel("Set Roll-Pitch-Yaw:")
        setting_layout.addWidget(label_rpy)
        self.box_roll = CustomDoubleSpinBox()
        self.box_roll.setSingleStep(0.01)
        setting_layout.addWidget(self.box_roll)
        self.box_pitch = CustomDoubleSpinBox()
        self.box_pitch.setSingleStep(0.01)
        setting_layout.addWidget(self.box_pitch)
        self.box_yaw = CustomDoubleSpinBox()
        self.box_yaw.setSingleStep(0.01)
        setting_layout.addWidget(self.box_yaw)
        self.box_roll.setRange(-np.pi, np.pi)
        self.box_pitch.setRange(-np.pi, np.pi)
        self.box_yaw.setRange(-np.pi, np.pi)

        label_cloud_num = QLabel("Set cloud frame number:")
        setting_layout.addWidget(label_cloud_num)
        self.box_cloud_num = QSpinBox()
        self.box_cloud_num.setValue(self.cloud_num)
        self.box_cloud_num.setRange(1, 100)
        self.box_cloud_num.valueChanged.connect(self.update_cloud_num)
        setting_layout.addWidget(self.box_cloud_num)

        label_trans_quat = QLabel("The lidar0-lidar1 trans and quat:")
        setting_layout.addWidget(label_trans_quat)
        self.line_trans = QLineEdit()
        self.line_trans.setReadOnly(True)
        setting_layout.addWidget(self.line_trans)
        self.line_quat = QLineEdit()
        self.line_quat.setReadOnly(True)
        setting_layout.addWidget(self.line_quat)

        # ICP configuration group
        icp_group = QGroupBox("ICP Configuration")
        icp_layout = QVBoxLayout()

        self.box_voxel_size = CustomDoubleSpinBox(decimals=2)
        self.box_voxel_size.setPrefix("Voxel Size: ")
        self.box_voxel_size.setSingleStep(0.01)
        self.box_voxel_size.setRange(0.01, 1.0)
        self.box_voxel_size.setValue(self.voxel_size)
        self.box_voxel_size.valueChanged.connect(self.update_voxel_size)
        icp_layout.addWidget(self.box_voxel_size)

        self.box_normal_radius = CustomDoubleSpinBox(decimals=2)
        self.box_normal_radius.setPrefix("Normal Radius: ")
        self.box_normal_radius.setSingleStep(0.01)
        self.box_normal_radius.setRange(0.01, 1.0)
        self.box_normal_radius.setValue(self.normal_radius)
        icp_layout.addWidget(self.box_normal_radius)

        self.box_icp_distance = CustomDoubleSpinBox(decimals=2)
        self.box_icp_distance.setPrefix("ICP Distance: ")
        self.box_icp_distance.setSingleStep(0.1)
        self.box_icp_distance.setRange(0.1, 100.0)
        self.box_icp_distance.setValue(self.icp_distance)
        icp_layout.addWidget(self.box_icp_distance)

        icp_group.setLayout(icp_layout)
        setting_layout.addWidget(icp_group)

        self.icp_button = QPushButton("Auto Scan Matching")
        self.icp_button.clicked.connect(self.perform_matching)
        setting_layout.addWidget(self.icp_button)

        self.line_trans.setText(
            f"[{self.t01[0]:.6f}, {self.t01[1]:.6f}, {self.t01[2]:.6f}]")
        quat = matrix_to_quaternion(self.R01)
        self.line_quat.setText(
            f"[{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")

        # Connect spin boxes to methods
        self.box_x.valueChanged.connect(self.apply_transform)
        self.box_y.valueChanged.connect(self.apply_transform)
        self.box_z.valueChanged.connect(self.apply_transform)
        self.box_roll.valueChanged.connect(self.apply_transform)
        self.box_pitch.valueChanged.connect(self.apply_transform)
        self.box_yaw.valueChanged.connect(self.apply_transform)

        main_layout.addLayout(setting_layout)
        

    def update_cloud_num(self):
        self.cloud_num = self.box_cloud_num.value()

    def update_voxel_size(self):
        self.voxel_size = self.box_voxel_size.value()

    def apply_transform(self):
        """Update transformation parameters and apply to scan1 using set_transform."""
        # Update translation
        x = self.box_x.value()
        y = self.box_y.value()
        z = self.box_z.value()
        self.t01 = np.array([x, y, z])
        self.line_trans.setText(f"[{x:.6f}, {y:.6f}, {z:.6f}]")
        
        # Update rotation
        roll = self.box_roll.value()
        pitch = self.box_pitch.value()
        yaw = self.box_yaw.value()
        self.R01 = euler_to_matrix(np.array([roll, pitch, yaw]))
        quat = matrix_to_quaternion(self.R01)
        self.line_quat.setText(
            f"[{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")
        
        # Apply transformation
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = self.R01
        T[:3, 3] = self.t01
        self['scan1'].set_transform(T)

    def perform_matching(self):
        global cloud0_accum, cloud1_accum

        if o3d is None:
            print("Error: open3d is not available. Cannot perform matching.")
            return

        if cloud0_accum is not None and cloud1_accum is not None:
            # Initial transformation
            T_init = np.eye(4)
            T_init[:3, :3] = self.R01
            T_init[:3, 3] = self.t01

            # Perform ICP registration using cloud_registration.matching
            transformation_icp, result = matching(
                cloud0_accum, cloud1_accum,
                down_sampling_size=self.box_voxel_size.value(),
                normal_radius=self.box_normal_radius.value(),
                icp_distance=self.box_icp_distance.value(),
                T_init=T_init,
                icp_model="p2plane"
            )
            
            print(f"Matching fitness: {result.fitness:.6f}")

            # Update R01 and t01 with ICP result
            R01 = transformation_icp[:3, :3]
            t01 = transformation_icp[:3, 3]

            # Update the UI with new values (block signals to prevent multiple intermediate updates)
            quat = matrix_to_quaternion(R01)
            rpy = matrix_to_euler(R01)
            self.box_roll.blockSignals(True)
            self.box_pitch.blockSignals(True)
            self.box_yaw.blockSignals(True)
            self.box_x.blockSignals(True)
            self.box_y.blockSignals(True)
            self.box_z.blockSignals(True)
            
            self.box_roll.setValue(rpy[0])
            self.box_pitch.setValue(rpy[1])
            self.box_yaw.setValue(rpy[2])
            self.box_x.setValue(t01[0])
            self.box_y.setValue(t01[1])
            self.box_z.setValue(t01[2])
            
            self.box_roll.blockSignals(False)
            self.box_pitch.blockSignals(False)
            self.box_yaw.blockSignals(False)
            self.box_x.blockSignals(False)
            self.box_y.blockSignals(False)
            self.box_z.blockSignals(False)
            self.line_trans.setText(
                f"[{t01[0]:.6f}, {t01[1]:.6f}, {t01[2]:.6f}]")
            self.line_quat.setText(
                f"[{quat[0]: .6f}, {quat[1]: .6f}, {quat[2]: .6f}, {quat[3]: .6f}]")
            self.t01 = t01
            self.R01 = R01
            print("Matching results")
            print(f"translation: [{t01[0]:.6f}, {t01[1]:.6f}, {t01[2]:.6f}]")
            print(
                f"Roll-Pitch-Yaw: [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]")
            print(
                f"Quaternion: [{quat[0]: .6f}, {quat[1]: .6f}, {quat[2]: .6f}, {quat[3]: .6f}]")
            
            # Update visualization with final result
            self.apply_transform()


def scan0_cb(data):
    global viewer, clouds0, cloud0_accum
    cloud, _, _  = convert_pointcloud2_msg(data)
    while len(clouds0) > viewer.cloud_num:
        clouds0.pop(0)
    clouds0.append(cloud)
    cloud0_accum = np.concatenate(clouds0)
    viewer['scan0'].set_data(data=cloud0_accum)


def scan1_cb(data):
    global viewer, clouds1, cloud1_accum
    cloud, _, _  = convert_pointcloud2_msg(data)
    while len(clouds1) > viewer.cloud_num:
        clouds1.pop(0)
    clouds1.append(cloud)
    cloud1_accum = np.concatenate(clouds1)
    viewer['scan1'].set_data(data=cloud1_accum)


def main():
    global viewer
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description="Find a T let T * scan1 = LiDAR0")
    parser.add_argument("lidar0", type=str,
                        help="Topic name for LiDAR0 data")
    parser.add_argument("lidar1", type=str,
                        help="Topic name for LiDAR1 data")
    args = parser.parse_args()

    app = q3d.QApplication(["LiDAR Calib"])
    viewer = LiDARCalibViewer(name='LiDAR Calib')
    grid_item = q3d.GridItem(size=10, spacing=1, color='#00000040')
    scan0_item = q3d.CloudItem(
        size=2, alpha=1, color_mode='FLAT', color='#ff4466')
    scan1_item = q3d.CloudItem(
        size=2, alpha=1, color_mode='FLAT', color='#00dd88')
    viewer.add_items(
        {'scan0': scan0_item, 'scan1': scan1_item, 'grid': grid_item})

    rospy.init_node('lidar_calib', anonymous=True)

    # Use topic names from arguments
    rospy.Subscriber(args.lidar0, PointCloud2, scan0_cb, queue_size=1)
    rospy.Subscriber(args.lidar1, PointCloud2, scan1_cb, queue_size=1)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
