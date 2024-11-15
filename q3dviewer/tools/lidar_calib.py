#!/usr/bin/env python3

from sensor_msgs.msg import PointCloud2
from q3dviewer import *
import rospy
import numpy as np
import argparse
import open3d as o3d

viewer = None
cloud0_accum = None
clouds0 = []
cloud1_accum = None
clouds1 = []

class CustomDoubleSpinBox(QDoubleSpinBox):
    def __init__(self, decimals=3, *args, **kwargs):
        self.decimals = decimals
        super().__init__(*args, **kwargs)
        self.setDecimals(self.decimals)  # Set the number of decimal places

    def textFromValue(self, value):
        return f"{value:.{self.decimals}f}"

    def valueFromText(self, text):
        return float(text)


class ViewerWithPanel(Viewer):
    def __init__(self, **kwargs):
        self.t01 = np.array([0, 0, 0])
        self.rpy01 = np.array([0, 0, 0])
        self.R01 = euler_to_matrix(self.rpy01)
        self.cloud_num = 10
        super().__init__(**kwargs)

    def initUI(self):
        centerWidget = QWidget()
        self.setCentralWidget(centerWidget)
        main_layout = QHBoxLayout()
        centerWidget.setLayout(main_layout)

        # Create a vertical layout for the settings
        setting_layout = QVBoxLayout()

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
        self.box_y.setRange(-100.0, 100.0)

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

        label_res = QLabel("The lidar-lidar quat and translation:")
        setting_layout.addWidget(label_res)
        self.line_quat = QLineEdit()
        self.line_quat.setReadOnly(True)
        setting_layout.addWidget(self.line_quat)
        self.line_trans = QLineEdit()
        self.line_trans.setReadOnly(True)
        setting_layout.addWidget(self.line_trans)

        self.icp_button = QPushButton("Auto Scan Matching")
        self.icp_button.clicked.connect(self.perform_icp)
        setting_layout.addWidget(self.icp_button)

        self.line_trans.setText(np.array2string(self.t01, formatter={
                                'float_kind': lambda x: "%.3f" % x}, separator=', '))
        quat = matrix_to_quaternion(self.R01)
        self.line_quat.setText(np.array2string(
            quat, formatter={'float_kind': lambda x: "%.3f" % x}, separator=', '))

        # Connect spin boxes to methods
        self.box_x.valueChanged.connect(self.update_xyz)
        self.box_y.valueChanged.connect(self.update_xyz)
        self.box_z.valueChanged.connect(self.update_xyz)
        self.box_roll.valueChanged.connect(self.update_rpy)
        self.box_pitch.valueChanged.connect(self.update_rpy)
        self.box_yaw.valueChanged.connect(self.update_rpy)

        # Add a stretch to push the widgets to the top
        setting_layout.addStretch(1)

        self.viewerWidget = self.vw()
        main_layout.addLayout(setting_layout)
        main_layout.addWidget(self.viewerWidget, 1)

        timer = QtCore.QTimer(self)
        timer.setInterval(20)  # period, in milliseconds
        timer.timeout.connect(self.update)
        self.viewerWidget.setCameraPosition(distance=5)
        self.viewerWidget.setBKColor('#ffffff')
        timer.start()

    def update_cloud_num(self):
        self.cloud_num = self.box_cloud_num.value()

    def update_xyz(self):
        x = self.box_x.value()
        y = self.box_y.value()
        z = self.box_z.value()
        self.t01 = np.array([x, y, z])
        self.line_trans.setText(np.array2string(self.t01, formatter={
                                'float_kind': lambda x: "%.3f" % x}, separator=', '))

    def update_rpy(self):
        roll = self.box_roll.value()
        pitch = self.box_pitch.value()
        yaw = self.box_yaw.value()
        self.rpy01 = np.array([roll, pitch, yaw])
        self.R01 = euler_to_matrix(self.rpy01)
        quat = matrix_to_quaternion(self.R01)
        self.line_quat.setText(np.array2string(
            quat, formatter={'float_kind': lambda x: "%.3f" % x}, separator=', '))

    def perform_icp(self):
        global cloud0_accum, cloud1_accum
        if cloud0_accum is not None and cloud1_accum is not None:
            # Convert to Open3D point clouds
            cloud0_o3d = o3d.geometry.PointCloud()
            cloud0_o3d.points = o3d.utility.Vector3dVector(cloud0_accum['xyz'])
            cloud1_o3d = o3d.geometry.PointCloud()
            cloud1_o3d.points = o3d.utility.Vector3dVector(cloud1_accum['xyz'])

            # Initialize transformation matrix with R01 and t01
            trans_init = np.eye(4)
            trans_init[:3, :3] = self.R01
            trans_init[:3, 3] = self.t01
            
            # Auto Scan Matching
            threshold = 0.2
            reg_p2p = o3d.pipelines.registration.registration_icp(
                cloud1_o3d, cloud0_o3d, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint())
            
            transformation_icp = reg_p2p.transformation
            print("ICP Transformation:", transformation_icp)
            
            # Update R01 and t01 with ICP result
            self.R01 = transformation_icp[:3, :3]
            self.t01 = transformation_icp[:3, 3]
            
            # Update the UI with new values
            self.line_trans.setText(np.array2string(self.t01, formatter={
                                    'float_kind': lambda x: "%.3f" % x}, separator=', '))
            quat = matrix_to_quaternion(self.R01)
            self.line_quat.setText(np.array2string(
                quat, formatter={'float_kind': lambda x: "%.3f" % x}, separator=', '))

            self.rpy01 = matrix_to_euler(self.R01)
            self.box_roll.setValue(self.rpy01[0])
            self.box_pitch.setValue(self.rpy01[1])
            self.box_yaw.setValue(self.rpy01[2])


def msg2Cloud(data):
    pc = PointCloud.from_msg(data).pc_data
    data_type = [('xyz', '<f4', (3,)), ('color', '<u4')]
    color = pc['intensity'].astype(np.uint32)
    cloud = np.rec.fromarrays(
        [np.stack([pc['x'], pc['y'], pc['z']], axis=1), color],
        dtype=data_type)
    return cloud

def scan0CB(data):
    global viewer, clouds0, cloud0_accum
    cloud = msg2Cloud(data)
    while len(clouds0) > viewer.cloud_num:
        clouds0.pop(0)
    clouds0.append(cloud)
    cloud0_accum = np.concatenate(clouds0)
    viewer['scan0'].setData(data=cloud0_accum)


def scan1CB(data):
    global viewer, clouds1, cloud1_accum
    cloud = msg2Cloud(data)
    while len(clouds1) > viewer.cloud_num:
        clouds1.pop(0)
    clouds1.append(cloud)
    cloud1_accum = np.concatenate(clouds1)
    cloud0_accum_new = cloud1_accum.copy()
    cloud0_accum_new['xyz'] = (viewer.R01 @ cloud1_accum['xyz'].T + viewer.t01[:, np.newaxis]).T

    viewer['scan1'].setData(data=cloud0_accum_new)


def main():
    global viewer
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Configure topic names for LiDAR, image, and camera info.")
    parser.add_argument('--lidar0', type=str, default='/lidar0', help="Topic name for LiDAR data")
    parser.add_argument('--lidar1', type=str, default='/lidar1', help="Topic name for camera image data")
    args = parser.parse_args()

    app = QApplication([])
    viewer = ViewerWithPanel(name='LiDAR Calib')
    grid_item = GridItem(size=10, spacing=1, color=(0, 0, 0, 70))
    scan0_item = CloudItem(size=2, alpha=1, color_mode='#FF0000')
    scan1_item = CloudItem(size=2, alpha=1, color_mode='#00FF00')
    viewer.addItems({'scan0': scan0_item, 'scan1': scan1_item, 'grid': grid_item})

    rospy.init_node('lidar_calib', anonymous=True)
    
    # Use topic names from arguments
    rospy.Subscriber(args.lidar0, PointCloud2, scan0CB, queue_size=1)
    rospy.Subscriber(args.lidar1, PointCloud2, scan1CB, queue_size=1)

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
