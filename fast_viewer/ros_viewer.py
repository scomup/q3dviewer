#!/usr/bin/env python3


from fast_viewer.basic_window import *
from fast_viewer.utils import make_transform
from fast_viewer.custom_items import *
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
from PyQt5.QtWidgets import QApplication
import random
from pypcd4 import PointCloud


viewer = None
point_num_per_scan = None

def odomCB(data):
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
    viewer["odom"].setTransform(transform)


def scanCB(data):
    global viewer
    global point_num_per_scan
    pc = PointCloud.from_msg(data).pc_data

    data_type = viewer["scan"].data_type
    cloud = np.rec.fromarrays(
            [np.stack([pc["x"], pc["y"], pc["z"]], axis=1), pc["intensity"]], dtype=data_type)

    if(cloud.shape[0] > point_num_per_scan):
        idx = random.sample(range(cloud.shape[0]), point_num_per_scan)
        cloud = cloud[idx]
    viewer["map"].appendData(data=cloud)
    viewer["scan"].setData(data=cloud)


def main():
    rospy.init_node('ros_viewer', anonymous=True)

    global viewer
    global point_num_per_scan

    point_num_per_scan = 10000
    mapItem = CloudItem(size=1, alpha=0.1, color_mode=-1)
    scanItem = CloudItem(size=3, alpha=1, color_mode="#FFFFFF")
    odomItem = GLAxisItem(size=0.5, width=5)
    girdItem = GridItem(size=1000, spacing=20)

    app = QApplication([])
    viewer = Viewer(name='ROS Viewer')

    viewer.addItems(map=mapItem, scan=scanItem, odom=odomItem, grid=girdItem)

    point_num_per_scan = rospy.get_param("fast_viewer/scan_num", 500)
    print("point_num_per_scan: %d" % point_num_per_scan)
    scan_sub = rospy.Subscriber("/cloud_registered", PointCloud2, scanCB, queue_size=1, buff_size=2**24)
    odom_sub = rospy.Subscriber("/odometry", Odometry, odomCB, queue_size=1, buff_size=2**24)
    viewer.show()
    app.exec_()

if __name__ == "__main__":
    main()
