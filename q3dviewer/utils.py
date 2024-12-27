"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np


def frustum(left, right, bottom, top, near, far):
    # see https://www.khronos.org/registry/OpenGL-Refpages/gl2.1/xhtml/glFrustum.xml
    if near <= 0 or far <= 0 or near >= far or left == right or bottom == top:
        print("Invalid frustum parameters.")
        return None
    matrix = np.zeros((4, 4), dtype=np.float32)
    matrix[0, 0] = 2.0 * near / (right - left)
    matrix[0, 2] = (right + left) / (right - left)
    matrix[1, 1] = 2.0 * near / (top - bottom)
    matrix[1, 2] = (top + bottom) / (top - bottom)
    matrix[2, 2] = -(far + near) / (far - near)
    matrix[2, 3] = -2.0 * far * near / (far - near)
    matrix[3, 2] = -1.0
    return matrix


def rainbow(scalars, scalar_min=0, scalar_max=255):
    range = scalar_max - scalar_min
    values = 1.0 - (scalars - scalar_min) / range
    # values = (scalars - scalar_min) / range  # using inverted color
    colors = np.zeros([scalars.shape[0], 3], dtype=np.float32)
    values = np.clip(values, 0, 1)

    h = values * 5.0 + 1.0
    i = np.floor(h).astype(int)
    f = h - i
    f[np.logical_not(i % 2)] = 1 - f[np.logical_not(i % 2)]
    n = 1 - f

    # idx = i <= 1
    colors[i <= 1, 0] = n[i <= 1] * 255
    colors[i <= 1, 1] = 0
    colors[i <= 1, 2] = 255

    colors[i == 2, 0] = 0
    colors[i == 2, 1] = n[i == 2] * 255
    colors[i == 2, 2] = 255

    colors[i == 3, 0] = 0
    colors[i == 3, 1] = 255
    colors[i == 3, 2] = n[i == 3] * 255

    colors[i == 4, 0] = n[i == 4] * 255
    colors[i == 4, 1] = 255
    colors[i == 4, 2] = 0

    colors[i >= 5, 0] = 255
    colors[i >= 5, 1] = n[i >= 5] * 255
    colors[i >= 5, 2] = 0
    return colors


def euler_to_matrix(rpy):
    roll, pitch, yaw = rpy
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return R


def matrix_to_euler(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    singular = sy < 1e-6  # Check for gimbal lock
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])   # X-axis rotation
        pitch = np.arctan2(-R[2, 0], sy)      # Y-axis rotation
        yaw = np.arctan2(R[1, 0], R[0, 0])    # Z-axis rotation
    else:
        # Gimbal lock case
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0  # Arbitrarily set yaw to 0

    return np.array([roll, pitch, yaw])


def matrix_to_quaternion(matrix):
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (matrix[2, 1] - matrix[1, 2]) * s
        y = (matrix[0, 2] - matrix[2, 0]) * s
        z = (matrix[1, 0] - matrix[0, 1]) * s
    else:
        if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            w = (matrix[2, 1] - matrix[1, 2]) / s
            x = 0.25 * s
            y = (matrix[0, 1] + matrix[1, 0]) / s
            z = (matrix[0, 2] + matrix[2, 0]) / s
        elif matrix[1, 1] > matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            w = (matrix[0, 2] - matrix[2, 0]) / s
            x = (matrix[0, 1] + matrix[1, 0]) / s
            y = 0.25 * s
            z = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            w = (matrix[1, 0] - matrix[0, 1]) / s
            x = (matrix[0, 2] + matrix[2, 0]) / s
            y = (matrix[1, 2] + matrix[2, 1]) / s
            z = 0.25 * s
    return np.array([x, y, z, w])


def quaternion_to_matrix(quaternion):
    x, y, z, w = quaternion
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    n = np.linalg.norm(q)
    if np.any(n == 0.0):
        raise ZeroDivisionError("bad quaternion input")
    else:
        m = np.empty((3, 3))
        m[0, 0] = 1.0 - 2*(y**2 + z**2)/n
        m[0, 1] = 2*(x*y - z*w)/n
        m[0, 2] = 2*(x*z + y*w)/n
        m[1, 0] = 2*(x*y + z*w)/n
        m[1, 1] = 1.0 - 2*(x**2 + z**2)/n
        m[1, 2] = 2*(y*z - x*w)/n
        m[2, 0] = 2*(x*z - y*w)/n
        m[2, 1] = 2*(y*z + x*w)/n
        m[2, 2] = 1.0 - 2*(x**2 + y**2)/n
        return m


def make_transform(pose, rotation):
    transform = np.eye(4)
    transform[0:3, 0:3] = quaternion_to_matrix(rotation)
    transform[0:3, 3] = pose
    return transform

def makeT(R, t):
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t
    return T

def makeRt(T):
    R = T[0:3, 0:3]
    t = T[0:3, 3]
    return R, t

def hex_to_rgba(hex_color):
    color_flat = int(hex_color[1:], 16)
    red = (color_flat >> 16) & 0xFF
    green = (color_flat >> 8) & 0xFF
    blue = color_flat & 0xFF
    return (red / 255.0, green / 255.0, blue / 255.0, 1.0)

# euler = np.array([1, 0.1, 0.1])
# euler_angles = matrix_to_euler(euler_to_matrix(euler))
# print("Euler Angles:", euler_angles)
