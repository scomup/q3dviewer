#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import rospy
import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, \
    QSpinBox, QWidget, QVBoxLayout, QHBoxLayout, QCheckBox
from PySide6 import QtCore
import rospy
import cv2
import argparse
from q3dviewer.utils.convert_ros_msg import convert_pointcloud2_msg, convert_image_msg

clouds = []
remap_info = None
K = None
cloud_accum_color = None
cloud_accum = None

class CustomDoubleSpinBox(QDoubleSpinBox):
    def __init__(self, decimals=4, *args, **kwargs):
        self.decimals = decimals
        super().__init__(*args, **kwargs)
        self.setDecimals(self.decimals)  # Set the number of decimal places

    def textFromValue(self, value):
        return f"{value:.{self.decimals}f}"

    def valueFromText(self, text):
        return float(text)


class LidarCamViewer(q3d.Viewer):
    def __init__(self, **kwargs):
        # b: camera body frame
        # c: camera image frame
        # l: lidar frame
        self.Rbl = np.eye(3)
        self.Rbl = q3d.euler_to_matrix(np.array([0.0, 0.0, 0.0]))
        self.Rcb = np.array([[0, -1, 0],
                             [0, 0, -1],
                             [1, 0, 0]])
        self.tbl = np.array([0.0, 0.0, 0.0])
        self.tcl = self.Rcb @ self.tbl
        self.Rcl = self.Rcb @ self.Rbl
        self.psize = 2
        self.cloud_num = 1
        self.en_rgb = False
        super().__init__(**kwargs)

    def default_gl_setting(self, glwidget):
        # Set camera position and background color
        glwidget.set_bg_color('#ffffff')
        glwidget.set_cam_position(distance=5)

    def add_control_panel(self, main_layout):
        # Create a vertical layout for the settings
        setting_layout = QVBoxLayout()
        setting_layout.setAlignment(QtCore.Qt.AlignTop)
        # Add a checkbox for RGB
        self.checkbox_rgb = QCheckBox("Enable RGB Cloud")
        self.checkbox_rgb.setChecked(False)
        setting_layout.addWidget(self.checkbox_rgb)
        self.checkbox_rgb.stateChanged.connect(
            self.checkbox_changed)

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
        self.box_x.setValue(self.tbl[0])
        self.box_y.setValue(self.tbl[1])
        self.box_z.setValue(self.tbl[2])

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
        rpy = q3d.matrix_to_euler(self.Rbl)
        self.box_roll.setValue(rpy[0])
        self.box_pitch.setValue(rpy[1])
        self.box_yaw.setValue(rpy[2])

        label_psize = QLabel("Set point size:")
        setting_layout.addWidget(label_psize)
        self.box_psize = QSpinBox()
        self.box_psize.setValue(self.psize)
        self.box_psize.setRange(0, 5)
        self.box_psize.valueChanged.connect(self.update_point_size)
        setting_layout.addWidget(self.box_psize)

        label_cloud_num = QLabel("Set cloud frame number:")
        setting_layout.addWidget(label_cloud_num)
        self.box_cloud_num = QSpinBox()
        self.box_cloud_num.setValue(self.cloud_num)
        self.box_cloud_num.setRange(1, 100)
        self.box_cloud_num.valueChanged.connect(self.update_cloud_num)
        setting_layout.addWidget(self.box_cloud_num)

        label_res = QLabel("The cam-lidar quat and translation:")
        setting_layout.addWidget(label_res)
        self.line_quat = QLineEdit()
        self.line_quat.setReadOnly(True)
        setting_layout.addWidget(self.line_quat)
        self.line_trans = QLineEdit()
        self.line_trans.setReadOnly(True)
        setting_layout.addWidget(self.line_trans)

        self.line_trans.setText(
            f"[{self.tcl[0]:.6f}, {self.tcl[1]:.6f}, {self.tcl[2]:.6f}]")
        quat = q3d.matrix_to_quaternion(self.Rcl)
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


    def update_point_size(self):
        self.psize = self.box_psize.value()

    def update_cloud_num(self):
        self.cloud_num = self.box_cloud_num.value()

    def update_xyz(self):
        x = self.box_x.value()
        y = self.box_y.value()
        z = self.box_z.value()
        self.tbl = np.array([x, y, z])
        self.tcl =  self.Rcb @ self.tbl
        x, y, z = self.tcl
        self.line_trans.setText(f"[{x:.6f}, {y:.6f}, {x:.6f}]")

    def update_rpy(self):
        roll = self.box_roll.value()
        pitch = self.box_pitch.value()
        yaw = self.box_yaw.value()
        self.Rbl = q3d.euler_to_matrix(np.array([roll, pitch, yaw]))
        self.Rcl = self.Rcb @ self.Rbl
        quat = q3d.matrix_to_quaternion(self.Rcl)
        self.line_quat.setText(
            f"[{quat[0]:.6f}, {quat[1]:.6f}, {quat[2]:.6f}, {quat[3]:.6f}]")

    def checkbox_changed(self, state):
        if state == QtCore.Qt.Checked:
            self.en_rgb = True
        else:
            self.en_rgb = False


def camera_info_cb(data):
    global remap_info, K
    if remap_info is None:  # Initialize only once
        K = np.array(data.K).reshape(3, 3)
        D = np.array(data.D)
        rospy.loginfo("Camera intrinsic parameters set")
        height = data.height
        width = data.width
        mapx, mapy = cv2.initUndistortRectifyMap(
            K, D, None, K, (width, height), cv2.CV_32FC1)
        remap_info = [mapx, mapy]


def scan_cb(data):
    global viewer, clouds, cloud_accum, cloud_accum_color
    cloud, _, _  = convert_pointcloud2_msg(data)
    while len(clouds) > viewer.cloud_num:
        clouds.pop(0)
    clouds.append(cloud)
    cloud_accum = np.concatenate(clouds)

    if cloud_accum_color is not None and viewer.en_rgb:
        viewer['scan'].set_data(data=cloud_accum_color)
        viewer['scan'].set_color_mode('RGB')
    else:
        viewer['scan'].set_data(data=cloud_accum)
        viewer['scan'].set_color_mode('I')


def draw_larger_points(image, points, colors, radius):
    for dx in range(-radius + 1, radius):
        for dy in range(-radius + 1, radius):
            distance = np.sqrt(dx**2 + dy**2)
            if distance <= radius:
                x_indices = points[:, 0] + dx
                y_indices = points[:, 1] + dy
                image[y_indices, x_indices] = colors
    return image


def image_cb(data):
    global cloud_accum, cloud_accum_color, remap_info, K
    if remap_info is None:
        rospy.logwarn("Camera parameters not yet received.")
        return
    image, _ = convert_image_msg(data)

    # Undistort the image
    image_un = cv2.remap(image, remap_info[0], remap_info[1], cv2.INTER_LINEAR)
    if cloud_accum is not None:
        cloud_local = cloud_accum.copy()
        tcl = viewer.tcl
        Rcl = viewer.Rcl
        pl = cloud_local['xyz']
        pc = (Rcl @ pl.T + tcl[:, np.newaxis]).T
        u = (K @ pc.T).T
        u_mask = (u[:, 2] != 0) & (pc[:, 2] > 0.2)
        u = u[:, :2][u_mask] / u[:, 2][u_mask][:, np.newaxis]

        radius = viewer.psize  # Radius of the points to be drawn
        u = np.round(u).astype(np.int32)
        valid_x = (u[:, 0] >= radius) & (u[:, 0] < image_un.shape[1]-radius)
        valid_y = (u[:, 1] >= radius) & (u[:, 1] < image_un.shape[0]-radius)
        valid_points = valid_x & valid_y
        u = u[valid_points]

        intensity = cloud_local['irgb'][u_mask][valid_points] >> 24
        vmin = viewer['scan'].vmin
        vmax = viewer['scan'].vmax
        intensity_color = q3d.rainbow(intensity, scalar_min=vmin, scalar_max=vmax).astype(np.uint8)
        draw_image = image_un.copy()
        draw_image = draw_larger_points(draw_image, u, intensity_color, radius)
        rgb = image_un[u[:, 1], u[:, 0]]
        xyz = cloud_local['xyz'][u_mask][valid_points]
        color = (intensity.astype(np.uint32) << 24) | \
                (rgb[:, 0].astype(np.uint32) << 16) | \
                (rgb[:, 1].astype(np.uint32) << 8) | \
                (rgb[:, 2].astype(np.uint32) << 0)
        cloud_accum_color = np.rec.fromarrays(
            [xyz, color],
            dtype=viewer['scan'].data_type)
        viewer['img'].set_data(data=draw_image)


def main():
    global viewer
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description="Configure topic names for LiDAR, image, and camera info.")
    parser.add_argument(
        '--lidar', type=str, default='lidar', help="Topic of LiDAR data")
    parser.add_argument('--camera', type=str, default='image_raw',
                        help="Topic of camera image data")
    parser.add_argument('--camera_info', type=str,
                        default='camera_info', help="Topic of camera info")
    args = parser.parse_args()

    app = q3d.QApplication(['LiDAR Cam Calib'])
    viewer = LidarCamViewer(name='LiDAR Cam Calib')
    grid_item = q3d.GridItem(size=10, spacing=1, color='#00000040')
    scan_item = q3d.CloudItem(size=2, alpha=1, color_mode='I')
    img_item = q3d.ImageItem(pos=np.array([0, 0]), size=np.array([800, 600]))
    viewer.add_items({'scan': scan_item, 'grid': grid_item, 'img': img_item})

    rospy.init_node('lidar_cam_calib', anonymous=True)

    # Use topic names from arguments
    rospy.Subscriber(args.lidar, PointCloud2, scan_cb, queue_size=1)
    rospy.Subscriber(args.camera, Image, image_cb, queue_size=1)
    rospy.Subscriber(args.camera_info, CameraInfo,
                     camera_info_cb, queue_size=1)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
