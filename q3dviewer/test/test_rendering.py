#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

"""
this script tests the rendering of a cloud in a camera frame based on the 
camera pose and intrinsic matrix
"""

import numpy as np
import q3dviewer as q3d
import cv2

cloud, _ = q3d.load_pcd('//home/liu/tmp/gs_slam_dataset/data.pcd')

pose = np.array([0.00705215, 0.00596959, 0.00128543, 0.00296279, 0.0518244, -0.0033289, 0.998646])

def get_T(pose):
    t = pose[:3]
    quat = pose[3:]
    R = q3d.quaternion_to_matrix(quat)
    T = q3d.makeT(R, t)
    return T

Twl = get_T(pose)

pose_cl = np.array([-0.07903228, -0.00545123, -0.04363822, 0.48705093, -0.46590715,  0.51408928,  0.53049424])

Tcl = get_T(pose_cl)

Tcw = Tcl @ np.linalg.inv(Twl)

# # convert the opengl camera coordinate to the opencv camera coordinate
# Tconv = np.array([[1, 0, 0, 0],
#                   [0, -1, 0, 0],
#                   [0, 0, -1, 0],
#                   [0, 0, 0, 1]])
# 
# Tcw = Tconv @ Tcw

cam_params = np.array([409.66105605989907, 409.32179199376003, 436.5, 193.0])

width=873
height=386

K = np.array([[cam_params[0], 0, cam_params[2]],
                [0, cam_params[1], cam_params[3]],
                [0, 0, 1]])


def render_frame(cloud, Tcw, K, width, height):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    Rcw, tcw = Tcw[:3, :3], Tcw[:3, 3]
    pc = (Rcw @ cloud['xyz'].T).T + tcw
    uv = (K @ pc.T).T
    uv = uv[:, :2] / uv[:, 2][:, np.newaxis]
    mask = (pc[:, 2] > 0) & (pc[:, 2] < 60) & \
           (uv[:, 0] > 0) & (uv[:, 0] < width) & \
           (uv[:, 1] > 0) & (uv[:, 1] < height)
    uv = uv[mask]
    u = uv[:, 0].astype(int)
    v = uv[:, 1].astype(int)
    rgb = cloud['irgb'][mask]
    r = rgb >> 16 & 0xff
    g = rgb >> 8 & 0xff
    b = rgb & 0xff
    i = rgb >> 24 & 0xff

    rgb = q3d.rainbow(i, scalar_min=0, scalar_max=50)[:, [2, 1, 0]]

    # Sort by depth to ensure front points are drawn first
    depth = pc[mask, 2]
    sorted_indices = np.argsort(depth)
    u = u[sorted_indices]
    v = v[sorted_indices]
    image[v, u] = rgb[sorted_indices]
    # r = r[sorted_indices]
    # g = g[sorted_indices]
    # b = b[sorted_indices]
    # image[v, u] = np.stack([b, g, r], axis=1)

    return image


image = render_frame(cloud, Tcw, K, width, height)
cv2.imshow('image', image)
cv2.waitKey(0)
