#!/usr/bin/env python3

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from q3dviewer import *
import rospy

viewer = None
cloud = None

class CustomDoubleSpinBox(QDoubleSpinBox):
    def textFromValue(self, value):
        return f"{value:.3f}"

class ViewerWithPanel(Viewer):
    def __init__(self, **kwargs):
        self.xyz = np.array([0, 0, 0])
        self.rpy = np.array([0, 0, 0])
        self.K = np.array([[903.00238266,   0.     , 370.99059794],
                          [0.     , 903.8561347, 289.84862588,],
                          [ 0.     ,   0.     ,   1. ]] )
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
        self.box_x.setSingleStep(0.002)
        setting_layout.addWidget(self.box_x)
        self.box_y = CustomDoubleSpinBox()
        self.box_y.setSingleStep(0.002)
        setting_layout.addWidget(self.box_y)
        self.box_z = CustomDoubleSpinBox()
        self.box_z.setSingleStep(0.002)
        setting_layout.addWidget(self.box_z)
        self.box_x.setRange(-100.0, 100.0)
        self.box_y.setRange(-100.0, 100.0)
        self.box_y.setRange(-100.0, 100.0)

        # Add RPY spin boxes
        label_rpy = QLabel("Set Roll-Pitch-Yaw:")
        setting_layout.addWidget(label_rpy)
        self.box_roll = CustomDoubleSpinBox()
        self.box_roll.setSingleStep(0.002)
        setting_layout.addWidget(self.box_roll)
        self.box_pitch = CustomDoubleSpinBox()
        self.box_pitch.setSingleStep(0.002)
        setting_layout.addWidget(self.box_pitch)
        self.box_yaw = CustomDoubleSpinBox()
        self.box_yaw.setSingleStep(0.002)
        setting_layout.addWidget(self.box_yaw)
        self.box_roll.setRange(-100.0, 100.0)
        self.box_pitch.setRange(-100.0, 100.0)
        self.box_yaw.setRange(-100.0, 100.0)

        setting_layout.addStretch(1)  # Add a stretch to push the widgets to the top

        # Connect spin boxes to methods
        self.box_x.valueChanged.connect(self.update_xyz)
        self.box_y.valueChanged.connect(self.update_xyz)
        self.box_z.valueChanged.connect(self.update_xyz)
        self.box_roll.valueChanged.connect(self.update_rpy)
        self.box_pitch.valueChanged.connect(self.update_rpy)
        self.box_yaw.valueChanged.connect(self.update_rpy)

        self.viewerWidget = self.vw()
        main_layout.addLayout(setting_layout)
        main_layout.addWidget(self.viewerWidget, 1)

        timer = QtCore.QTimer(self)
        timer.setInterval(20)  # period, in milliseconds
        timer.timeout.connect(self.update)
        self.viewerWidget.setCameraPosition(distance=5)
        timer.start()

    def update_xyz(self):
        x = self.box_x.value()
        y = self.box_y.value()
        z = self.box_z.value()
        self.xyz = np.array([x, y, z])

    def update_rpy(self):
        roll = self.box_roll.value()
        pitch = self.box_pitch.value()
        yaw = self.box_yaw.value()
        self.rpy = np.array([roll, pitch, yaw])

def scanCB(data):
    global viewer
    global cloud
    pc = PointCloud.from_msg(data).pc_data
    data_type = viewer['scan'].data_type
    color = pc['intensity'].astype(np.uint32)
    cloud = np.rec.fromarrays(
        [np.stack([pc['x'], pc['y'], pc['z']], axis=1), color],
        dtype=data_type)
    viewer['scan'].setData(data=cloud)


def create_point_mask(image_shape, points, radius, color):
    mask = np.zeros(image_shape, dtype=np.uint8)
    for point, col in zip(points, color):
        rr, cc = np.ogrid[:image_shape[0], :image_shape[1]]
        circle = (rr - point[1])**2 + (cc - point[0])**2 <= radius**2
        mask[circle] = col
    return mask


def draw_larger_points(image, points, colors, radius):
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            distance = np.sqrt(dx**2 + dy**2)
            if distance <= radius:
                x_indices = points[:, 0] + dx
                y_indices = points[:, 1] + dy
                valid_indices = (x_indices >= 0) & (x_indices < image.shape[1]) & (y_indices >= 0) & (y_indices < image.shape[0])
                image[y_indices[valid_indices], x_indices[valid_indices]] = colors[valid_indices]
    return image

def imageCB(data):
    image = np.frombuffer(data.data, dtype=np.uint8).reshape(
        data.height, data.width, -1)
    if (data.encoding == 'bgr8'):
        image = image[:, :, ::-1]  # convert bgr to rgb

    if cloud is not None:
        cur_cloud = cloud.copy()
        # camera frame to optical frame
        Roc = np.array([[0, -1, 0],
                        [0, 0, -1],
                        [1, 0, 0]])
        Rcl = euler_to_matrix(viewer.rpy)
        tol = viewer.xyz
        Rol = Roc @ Rcl
        pl = cur_cloud.xyz
        po = (Rol @ pl.T + tol[:, np.newaxis]).T
        u = (viewer.K @ po.T).T
        u = u[:, :2] / u[:, 2][:, np.newaxis]
        radius = 2  # Radius of the points to be drawn
        valid_x = (u[:, 0] >= radius) & (u[:, 0] < image.shape[1]-radius)
        valid_y = (u[:, 1] >= radius) & (u[:, 1] < image.shape[0]-radius)
        valid_points = valid_x & valid_y
        u_valid = u[valid_points].astype(np.int32)
        color = rainbow(cur_cloud.color[valid_points]).astype(np.uint8)
        draw_image = image.copy()
        draw_image = draw_larger_points(draw_image, u_valid, color, radius)

        viewer['img'].setData(data=draw_image)


def main():
    global viewer
    app = QApplication([])
    viewer = ViewerWithPanel(name='example')
    gird_item = GridItem(size=10, spacing=1)
    scan_item = CloudItem(size=2, alpha=1, color_mode='I')
    img_item = ImageItem(pos=np.array([0, 0]), size=np.array([800, 600]))
    viewer.addItems({'scan':scan_item, 'grid': gird_item, 'img': img_item})

    rospy.init_node('lidar_cam_manual_calib', anonymous=True)
    rospy.Subscriber("/livox/lidar", PointCloud2, scanCB, queue_size=1)
    rospy.Subscriber('/usb_cam/image_raw', Image, imageCB, queue_size=1)

    viewer.show()
    app.exec_()

if __name__ == '__main__':
    main()



