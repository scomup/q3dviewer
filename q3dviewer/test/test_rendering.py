#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
import cv2

cloud, _ = q3d.load_pcd('/home/liu/lab.pcd')


Tcw = np.array([[7.07106781e-01,  7.07106781e-01,  0.00000000e+00,
                 0.00000000e+00],
                [-3.53553391e-01,  3.53553391e-01,  8.66025404e-01,
                 3.55271368e-15],
                [6.12372436e-01, -6.12372436e-01,  5.00000000e-01,
                 -4.00000000e+01],
                [0.00000000e+00,  0.00000000e+00,  0.00000000e+00,
                 1.00000000e+00]])
# convert the opengl camera coordinate to the opencv camera coordinate
Tconv = np.array([[1, 0, 0, 0],
                  [0, -1, 0, 0],
                  [0, 0, -1, 0],
                  [0, 0, 0, 1]])

Tcw = Tconv @ Tcw

K = np.array([[1.64718029e+03, 0.00000000e+00, 9.51000000e+02],
              [0.00000000e+00, 1.64718036e+03, 5.31000000e+02],
              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])


def render_frame(cloud, Tcw, K, width, height):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    Rcw, tcw = Tcw[:3, :3], Tcw[:3, 3]
    pc = (Rcw @ cloud['xyz'].T).T + tcw
    uv = (K @ pc.T).T
    uv = uv[:, :2] / uv[:, 2][:, np.newaxis]
    mask = (pc[:, 2] > 0) & (uv[:, 0] > 0) & (
        uv[:, 0] < width) & (uv[:, 1] > 0) & (uv[:, 1] < height)
    uv = uv[mask]
    u = uv[:, 0].astype(int)
    v = uv[:, 1].astype(int)
    rgb = cloud['irgb'][mask]
    r = rgb >> 16 & 0xff
    g = rgb >> 8 & 0xff
    b = rgb & 0xff

    # Sort by depth to ensure front points are drawn first
    depth = pc[mask, 2]
    sorted_indices = np.argsort(depth)
    u = u[sorted_indices]
    v = v[sorted_indices]
    r = r[sorted_indices]
    g = g[sorted_indices]
    b = b[sorted_indices]

    image[v, u] = np.stack([b, g, r], axis=1)
    return image


image = render_frame(cloud, Tcw, K, 1902, 1062)
cv2.imshow('image', image)
cv2.waitKey(0)
