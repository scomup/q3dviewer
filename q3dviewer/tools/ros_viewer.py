#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.utils.maths import make_transform
import q3dviewer as q3d
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import random
from sensor_msgs.msg import Image
from q3dviewer.utils.convert_ros_msg import convert_pointcloud2_msg, convert_odometry_msg, convert_image_msg

viewer = None
point_num_per_scan = None
color_mode = None
auto_set_color_mode = True

def odom_cb(data):
    global viewer
    transform, _ = convert_odometry_msg(data)
    viewer['odom'].set_transform(transform)


def scan_cb(data):
    global viewer
    global point_num_per_scan
    global auto_set_color_mode
    cloud, fields, _ = convert_pointcloud2_msg(data)
    if (cloud.shape[0] > point_num_per_scan):
        idx = random.sample(range(cloud.shape[0]), point_num_per_scan)
        cloud = cloud[idx]
    if 'rgb' in fields and auto_set_color_mode:
        print("Set color mode to RGB")
        viewer['map'].set_color_mode('RGB')
        auto_set_color_mode = False
    viewer['map'].set_data(data=cloud, append=True)
    viewer['scan'].set_data(data=cloud)


def image_cb(data):
    image, _ = convert_image_msg(data)
    viewer['img'].set_data(data=image)


def main():
    rospy.init_node('ros_viewer', anonymous=True)

    global viewer
    global point_num_per_scan
    point_num_per_scan = 10000
    map_item = q3d.CloudIOItem(size=1, alpha=0.1, color_mode='I')
    scan_item = q3d.CloudItem(
        size=2, alpha=1, color_mode='FLAT', color='#ffffff')
    odom_item = q3d.AxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=1000, spacing=20)
    img_item = q3d.ImageItem(pos=np.array([0, 0]), size=np.array([800, 600]))

    app = q3d.QApplication(['ROS Viewer'])
    viewer = q3d.Viewer(name='ROS Viewer')

    viewer.add_items({'map': map_item, 'scan': scan_item,
                     'odom': odom_item, 'grid': grid_item,
                     'img': img_item})

    point_num_per_scan = rospy.get_param("scan_num", 100000)
    print("point_num_per_scan: %d" % point_num_per_scan)
    rospy.Subscriber(
        "/cloud_registered", PointCloud2, scan_cb,
        queue_size=1, buff_size=2**24)
    rospy.Subscriber(
        "/odometry", Odometry, odom_cb, queue_size=1, buff_size=2**24)
    rospy.Subscriber('/image', Image, image_cb)

    viewer.show()
    app.exec()


if __name__ == "__main__":
    main()
