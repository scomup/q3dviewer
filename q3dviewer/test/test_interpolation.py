#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

"""
this script tests interpolation of 3D poses.
"""


import numpy as np
from q3dviewer.utils.maths import expSO3, logSO3, makeT, makeRt


def interpolate_pose(T1, T2, v_max, omega_max, dt=0.1):
    R1, t1 = makeRt(T1)
    R2, t2 = makeRt(T2)
    
    # Get transform time based on linear velocity
    d = np.linalg.norm(t2 - t1)
    t_lin = d / v_max
    
    # Get transform time based on angular velocity
    omega = logSO3(R2 @ R1.T)
    theta = np.linalg.norm(omega)
    t_ang = theta / omega_max
    
    # Get total time based on the linear and angular time
    t_total = max(t_lin, t_ang)
    num_steps = int(np.ceil(t_total / dt))
    
    # Generate interpolated transforms
    interpolated_Ts = []
    for i in range(num_steps + 1):
        s = i / num_steps
        t_interp = (1 - s) * t1 + s * t2
        # Interpolate rotation using SO3
        R_interp = expSO3(s * omega) @ R1
        T_interp = makeT(R_interp, t_interp)
        interpolated_Ts.append(T_interp)
    
    return interpolated_Ts

if __name__ == "__main__":
    T1 = np.eye(4)  # Identity transformation
    T2 = np.array([[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]])  # Target transformation

    v_max = 1.0  # Maximum linear velocity (m/s)
    omega_max = np.pi / 4  # Maximum angular velocity (rad/s)

    # Perform interpolation
    interpolated_poses = interpolate_pose(T1, T2, v_max, omega_max)
    for i, T in enumerate(interpolated_poses):
        print(f"Step {i}:\n{T}\n")


