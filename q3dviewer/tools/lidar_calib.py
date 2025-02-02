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
from PySide6.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, \
    QSpinBox, QWidget, QVBoxLayout, QHBoxLayout, QPushButton
from PySide6 import QtCore
from q3dviewer.utils.convert_ros_msg import convert_pointcloud2_msg

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
        return float(text)


class LiDARCalibViewer(q3d.Viewer):
    def __init__(self, **kwargs):
        self.t01 = np.array([0, 0, 0])
        self.R01 = np.eye(3)
        self.cloud_num = 10
        self.radius = 0.2
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

        label_matching_setting = QLabel("Set matching radius:")
        setting_layout.addWidget(label_matching_setting)
        self.box_radius = CustomDoubleSpinBox(decimals=2)
        self.box_radius.setSingleStep(0.1)
        self.box_radius.setRange(0.1, 3.0)
        self.box_radius.setValue(self.radius)
        setting_layout.addWidget(self.box_radius)

        self.icp_button = QPushButton("Auto Scan Matching")
        self.icp_button.clicked.connect(self.perform_matching)
        setting_layout.addWidget(self.icp_button)

        self.line_trans.setText(
            f"[{self.t01[0]:.6f}, {self.t01[1]:.6f}, {self.t01[2]:.6f}]")
        quat = q3d.matrix_to_quaternion(self.R01)
        self.line_quat.setText(
            f"[{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")

        # Connect spin boxes to methods
        self.box_x.valueChanged.connect(self.update_xyz)
        self.box_y.valueChanged.connect(self.update_xyz)
        self.box_z.valueChanged.connect(self.update_xyz)
        self.box_roll.valueChanged.connect(self.update_rpy)
        self.box_pitch.valueChanged.connect(self.update_rpy)
        self.box_yaw.valueChanged.connect(self.update_rpy)

        main_layout.addLayout(setting_layout)
        

    def update_radius(self):
        self.radius = self.box_radius.value()

    def update_cloud_num(self):
        self.cloud_num = self.box_cloud_num.value()

    def update_xyz(self):
        x = self.box_x.value()
        y = self.box_y.value()
        z = self.box_z.value()
        self.t01 = np.array([x, y, z])
        self.line_trans.setText(f"[{x:.6f}, {y:.6f}, {z:.6f}]")

    def update_rpy(self):
        roll = self.box_roll.value()
        pitch = self.box_pitch.value()
        yaw = self.box_yaw.value()
        self.R01 = q3d.euler_to_matrix(np.array([roll, pitch, yaw]))
        quat = q3d.matrix_to_quaternion(self.R01)
        self.line_quat.setText(
            f"[{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")

    def perform_matching(self):
        global cloud0_accum, cloud1_accum

        if o3d is None:
            print("Error: open3d is not available. Cannot perform matching.")
            return

        if cloud0_accum is not None and cloud1_accum is not None:
            # Convert to Open3D point clouds
            cloud0_o3d = o3d.geometry.PointCloud()
            cloud0_o3d.points = o3d.utility.Vector3dVector(cloud0_accum['xyz'])
            cloud1_o3d = o3d.geometry.PointCloud()
            cloud1_o3d.points = o3d.utility.Vector3dVector(cloud1_accum['xyz'])
            voxel_size = 0.1
            cloud0_o3d = cloud0_o3d.voxel_down_sample(voxel_size)
            cloud1_o3d = cloud1_o3d.voxel_down_sample(voxel_size)
            cloud0_o3d.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid
                (radius=self.radius, max_nn=30))
            trans_init = np.eye(4)
            trans_init[:3, :3] = self.R01
            trans_init[:3, 3] = self.t01

            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-4,
                max_iteration=50)

            # Auto Scan Matching
            reg_p2p = o3d.pipelines.registration.registration_icp(
                cloud1_o3d, cloud0_o3d, self.radius, trans_init,
                o3d.pipelines.registration.
                TransformationEstimationPointToPlane(),
                criteria)

            transformation_icp = reg_p2p.transformation
            # print("ICP Transformation:", transformation_icp)

            # Update R01 and t01 with ICP result
            R01 = transformation_icp[:3, :3]
            t01 = transformation_icp[:3, 3]

            # Update the UI with new values
            quat = q3d.matrix_to_quaternion(R01)
            rpy = q3d.matrix_to_euler(R01)
            self.box_roll.setValue(rpy[0])
            self.box_pitch.setValue(rpy[1])
            self.box_yaw.setValue(rpy[2])
            self.box_x.setValue(t01[0])
            self.box_y.setValue(t01[1])
            self.box_z.setValue(t01[2])
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
    cloud0_accum_new = cloud1_accum.copy()
    cloud0_accum_new['xyz'] = (
        viewer.R01 @ cloud1_accum['xyz'].T + viewer.t01[:, np.newaxis]).T

    viewer['scan1'].set_data(data=cloud0_accum_new)


def main():
    global viewer
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description="Configure topic names for LiDAR, image, and camera info.")
    parser.add_argument('--lidar0', type=str, default='/lidar0',
                        help="Topic name for LiDAR data")
    parser.add_argument('--lidar1', type=str, default='/lidar1',
                        help="Topic name for camera image data")
    args = parser.parse_args()

    app = q3d.QApplication(["LiDAR Calib"])
    viewer = LiDARCalibViewer(name='LiDAR Calib')
    grid_item = q3d.GridItem(size=10, spacing=1, color='#00000040')
    scan0_item = q3d.CloudItem(
        size=2, alpha=1, color_mode='FLAT', color='#ff0000')
    scan1_item = q3d.CloudItem(
        size=2, alpha=1, color_mode='FLAT', color='#00ff00')
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
