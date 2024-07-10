#!/usr/bin/env python3


from fast_viewer.basic_window import *
from fast_viewer.utils import make_transform, colormap, FPSMonitor
from fast_viewer.pointcloud2_tools import pointcloud2_to_array
from fast_viewer.custom_items import *
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import signal
import math
from PyQt5.QtWidgets import QApplication
import random
from PyQt5 import QtCore


point_num_per_scan = 10000
mapItem = CloudItem(size=1, alpha=0.1, color_mode=-1)
scanItem = CloudItem(size=3, alpha=1, color_mode="#FFFFFF")
odomItem = GLAxisItem(size=0.5, width=5)
girdItem = GridItem()
girdItem.setSize(1000, 1000)
girdItem.setSpacing(20, 20)

items = {"map": mapItem,  "scan": scanItem, "odom": odomItem, "grid": girdItem}
app = QApplication([])
viewer = Viewer(items)
scan_num = 0


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
    viewer.items["odom"].setTransform(transform)


def scanCB(data):
    global scan_num
    global viewer
    pc = pointcloud2_to_array(data)
    data_type = [('xyz', '<f4', (3, )), ('color', '<u4')]
    cloud = np.rec.fromarrays(
            [np.stack([pc["x"], pc["y"], pc["z"]], axis=1), pc["intensity"]], dtype=data_type)

    if(cloud.shape[0] > point_num_per_scan):
        idx = random.sample(range(cloud.shape[0]), point_num_per_scan)
        cloud = cloud[idx]
    viewer.items["map"].appendData(data=cloud)
    viewer.items["scan"].setData(data=cloud)


def main():
    rospy.init_node('fast_viewer', anonymous=True)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    point_num_per_scan = rospy.get_param("fast_viewer/scan_num", 500)
    print("point_num_per_scan: %d" % point_num_per_scan)
    scan_sub = rospy.Subscriber("/cloud_registered", PointCloud2, scanCB, queue_size=1, buff_size=2**24)
    odom_sub = rospy.Subscriber("/odometry", Odometry, odomCB, queue_size=1, buff_size=2**24)
    viewer.show()
    app.exec_()

if __name__ == "__main__":
    main()
