"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from pypcd4 import PointCloud
from q3dviewer.utils.maths import make_transform


def convert_pointcloud2_msg(msg):
    pc = PointCloud.from_msg(msg).pc_data
    data_type = [('xyz', '<f4', (3,)), ('irgb', '<u4')]
    rgb = np.zeros([pc.shape[0]], dtype=np.uint32)
    intensity = np.zeros([pc.shape[0]], dtype=np.uint32)
    fields = ['xyz']
    if 'intensity' in pc.dtype.names:
        intensity = pc['intensity'].astype(np.uint32)
        fields.append('intensity')
    if 'rgb' in pc.dtype.names:
        rgb = pc['rgb'].view(np.uint32)
        fields.append('rgb')
    irgb = (intensity << 24) | rgb
    xyz = np.stack([pc['x'], pc['y'], pc['z']], axis=1)
    cloud = np.rec.fromarrays([xyz, irgb], dtype=data_type)
    stamp = msg.header.stamp.to_sec()
    return cloud, fields, stamp


def convert_odometry_msg(msg):
    pose = np.array(
        [msg.pose.pose.position.x,
         msg.pose.pose.position.y,
         msg.pose.pose.position.z])
    rotation = np.array([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w])
    transform = make_transform(pose, rotation)
    stamp = msg.header.stamp.to_sec()
    return transform, stamp


def convert_image_msg(msg):
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, -1)
    if (msg.encoding == 'bgr8'):
        image = image[:, :, ::-1]  # convert bgr to rgb
    stamp = msg.header.stamp.to_sec()
    return image, stamp
