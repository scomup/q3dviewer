#!/usr/bin/env python3

"""
Copyright 2024  Liu Yang
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.utils import make_transform
import q3dviewer as q3d
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import random
from pypcd4 import PointCloud
from sensor_msgs.msg import Image

viewer = None
point_num_per_scan = None
color_mode = None


def odom_cb(data):
    global viewer
    pose = np.array(
        [data.pose.pose.position.x,
         data.pose.pose.position.y,
         data.pose.pose.position.z])
    rotation = np.array([
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w])
    transform = make_transform(pose, rotation)
    viewer['odom'].setTransform(transform)


def scan_cb(data):
    global viewer
    global point_num_per_scan
    global color_mode
    pc = PointCloud.from_msg(data).pc_data
    data_type = viewer['scan'].data_type
    if 'rgb' in pc.dtype.names:
        color = pc['rgb'].view(np.uint32)
    else:
        color = pc['intensity']

    if color_mode is None:
        if 'rgb' in pc.dtype.names:
            color_mode = 'RGB'
        else:
            color_mode = 'I'
        viewer['map'].set_color_mode(color_mode)

    cloud = np.rec.fromarrays(
        [np.stack([pc['x'], pc['y'], pc['z']], axis=1), color],
        dtype=data_type)
    if (cloud.shape[0] > point_num_per_scan):
        idx = random.sample(range(cloud.shape[0]), point_num_per_scan)
        cloud = cloud[idx]
    viewer['map'].set_data(data=cloud, append=True)
    viewer['scan'].set_data(data=cloud)


def image_cb(data):
    image = np.frombuffer(data.data, dtype=np.uint8).reshape(
        data.height, data.width, -1)
    if (data.encoding == 'bgr8'):
        image = image[:, :, ::-1]  # convert bgr to rgb
    viewer['img'].set_data(data=image)


def main():
    rospy.init_node('ros_viewer', anonymous=True)

    global viewer
    global point_num_per_scan
    point_num_per_scan = 10000
    map_item = q3d.CloudIOItem(size=1, alpha=0.1, color_mode='RGB')
    scan_item = q3d.CloudItem(size=2, alpha=1, color_mode='#FFFFFF')
    odom_item = q3d.GLAxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=1000, spacing=20)
    img_item = q3d.ImageItem(pos=np.array([0, 0]), size=np.array([800, 600]))

    app = q3d.QApplication(['ROS Viewer'])
    viewer = q3d.Viewer(name='ROS Viewer')

    viewer.add_items({'map': map_item, 'scan': scan_item,
                    'odom': odom_item, 'grid': grid_item, 'img': img_item})

    point_num_per_scan = rospy.get_param("scan_num", 100000)
    print("point_num_per_scan: %d" % point_num_per_scan)
    rospy.Subscriber(
        "/cloud_registered", PointCloud2, scan_cb,
        queue_size=1, buff_size=2**24)
    rospy.Subscriber(
        "/odometry", Odometry, odom_cb, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/image', Image, image_cb)

    viewer.show()
    app.exec_()


if __name__ == "__main__":
    main()
