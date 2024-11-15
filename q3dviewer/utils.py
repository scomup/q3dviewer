import numpy as np
import time


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
    # yaw pitch roll order
    R = Rx @ Ry @ Rz
    return R

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


class FPSMonitor():
    def __init__(self):
        self.stamp_record = []

    def count(self):
        cur_stamp = time.time()
        self.stamp_record.append(cur_stamp)
        while len(self.stamp_record) > 0:
            if(cur_stamp - self.stamp_record[0] > 1.):
                self.stamp_record.pop(0)
            else:
                break
        return len(self.stamp_record)

