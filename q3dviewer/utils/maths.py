"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

"""
Math proofs and implementations: 
https://github.com/scomup/MathematicalRobotics.git
"""


import numpy as np


_epsilon_ = 1e-5


def skew(vector):
    return np.array([[0, -vector[2], vector[1]],
                     [vector[2], 0, -vector[0]],
                     [-vector[1], vector[0], 0]])


def expSO3(omega):
    """
    Exponential map of SO3
    The proof is shown in 3d_rotation_group.md (10)
    """
    theta2 = omega.dot(omega)
    theta = np.sqrt(theta2)
    nearZero = theta2 <= _epsilon_
    W = skew(omega)
    if (nearZero):
        return np.eye(3) + W
    else:
        K = W/theta
        KK = K.dot(K)
        sin_theta = np.sin(theta)
        one_minus_cos = 1 - np.cos(theta)
        R = np.eye(3) + sin_theta * K + one_minus_cos * KK  # rotation.md (10)
        return R


def logSO3(R):
    """
    Logarithm map of SO3
    The proof is shown in rotation.md (14)
    """
    R11, R12, R13 = R[0, :]
    R21, R22, R23 = R[1, :]
    R31, R32, R33 = R[2, :]
    tr = np.trace(R)
    omega = np.zeros(3)
    v = np.array([R32 - R23, R13 - R31, R21 - R12])
    # when trace == -1, i.e., the theta is approx to +-pi, +-3pi, +-5pi, etc.
    # we do something special
    if (tr + 1.0 < 1e-3):
        if (R33 > R22 and R33 > R11):
            # R33 is largest
            # sin(theta) approx to sgn_w*pi-theta, cos(theta) approx to -1
            W = R21 - R12  # 2*r3*sin(theta) = 2*r3*(sgn_w*pi-theta)
            Q1 = R31 + R13          # 4 * r1*r3
            Q2 = R23 + R32          # 4 * r2*r3
            Q3 = 2.0 + 2.0 * R33    # 4 * r3*r3
            r = np.sqrt(Q3)         # 2 * r3
            one_over_r = 1 / r      # 1 / (2*r3)
            norm = np.sqrt(Q1*Q1 + Q2*Q2 + Q3*Q3 + W*W)  # 4*r3
            sgn_w = np.sign(W)      # get the sgn of theta
            mag = np.pi - (2 * sgn_w * W) / norm   # theta*sgn_w
            scale = 0.5 * mag * one_over_r  # theta * sgn_w / (4*r3)
            # omega = theta * [4*r1*r3, 4*r2*r3, 4*r3*r3]/ (4*r3)
            omega = sgn_w * scale * np.array([Q1, Q2, Q3])
        elif (R22 > R11):
            # R22 is the largest
            W = R13 - R31  # 2*r2*sin(theta) = 2*r2*(sgn_w*pi-theta)
            Q1 = R12 + R21        # 4 * r2*r1
            Q2 = 2.0 + 2.0 * R22  # 4 * r2*r2
            Q3 = R23 + R32        # 4 * r2*r3
            r = np.sqrt(Q2)
            one_over_r = 1 / r
            norm = np.sqrt(Q1*Q1 + Q2*Q2 + Q3*Q3 + W*W)
            sgn_w = np.sign(W)
            mag = np.pi - (2 * sgn_w * W) / norm
            scale = 0.5 * one_over_r * mag
            omega = sgn_w * scale * np.array([Q1, Q2, Q3])
        else:
            # R11 is the largest
            W = R32 - R23  # 2*r1*sin(theta) = 2*r1*(sgn_w*pi-theta)
            Q1 = 2.0 + 2.0 * R11 # 4 * r1*r1
            Q2 = R12 + R21       # 4 * r1*r2
            Q3 = R31 + R13       # 4 * r1*r3
            r = np.sqrt(Q1)
            one_over_r = 1 / r
            norm = np.sqrt(Q1*Q1 + Q2*Q2 + Q3*Q3 + W*W)
            sgn_w = np.sign(W)
            mag = np.pi - (2 * sgn_w * W) / norm
            scale = 0.5 * one_over_r * mag
            omega = sgn_w * scale * np.array([Q1, Q2, Q3])
    else:
        magnitude = 0
        tr_3 = tr - 3.0
        if (tr_3 < -1e-6):
            # this is the normal case -1 < trace < 3
            theta = np.arccos((tr - 1.0) / 2.0)
            magnitude = theta / (2.0 * np.sin(theta))
        else:
            # when theta near 0, +-2pi, +-4pi, etc. (trace near 3.0)
            # use Taylor expansion: theta \approx 1/2-(t-3)/12 + O((t-3)^2)
            # see https://github.com/borglab/gtsam/issues/746 for details
            magnitude = 0.5 - tr_3 / 12.0 + tr_3*tr_3/60.0
        omega = magnitude * np.array([R32 - R23, R13 - R31, R21 - R12])
    return omega


def interpolate_pose(T1, T2, v_max, omega_max, dt=0.02):
    R1, t1 = makeRt(T1)
    R2, t2 = makeRt(T2)
    
    # Get transfrom time based on linear velocity
    d = np.linalg.norm(t2 - t1)
    t_lin = d / v_max
    
    # Get transform time based on angular velocity
    omega = logSO3(R2 @ R1.T)
    theta = np.linalg.norm(omega)
    t_ang = theta / omega_max
    
    # Get total time based on the linear and angular time.
    t_total = max(t_lin, t_ang)
    num_steps = int(np.ceil(t_total / dt))
    
    # Generate interpolated transforms
    interpolated_Ts = []
    for i in range(num_steps):
        s = i / num_steps
        t_interp = (1 - s) * t1 + s * t2
        # Interpolate rotation using SO3.
        R_interp = expSO3(s * omega) @ R1
        T_interp = makeT(R_interp, t_interp)
        interpolated_Ts.append(T_interp)
    
    return interpolated_Ts


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
    if not hex_color.startswith("#"):
        print("Invalid hex color string.")
        return (1.0, 1.0, 1.0, 1.0)
    if len(hex_color) == 7:
        color_flat = int(hex_color[1:], 16)
        red = (color_flat >> 16) & 0xFF
        green = (color_flat >> 8) & 0xFF
        blue = color_flat & 0xFF
        return (red / 255.0, green / 255.0, blue / 255.0, 1.0)
    elif len(hex_color) == 9:
        color_flat = int(hex_color[1:], 16)
        red = (color_flat >> 24) & 0xFF
        green = (color_flat >> 16) & 0xFF
        blue = (color_flat >> 8) & 0xFF
        alpha = color_flat & 0xFF
        return (red / 255.0, green / 255.0, blue / 255.0, alpha / 255.0)
    else:
        print("Invalid hex color string.")
        return (1.0, 1.0, 1.0, 1.0)

# euler = np.array([1, 0.1, 0.1])
# euler_angles = matrix_to_euler(euler_to_matrix(euler))
# print("Euler Angles:", euler_angles)
