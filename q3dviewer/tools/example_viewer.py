#!/usr/bin/env python3

import numpy as np
import threading
import time
import q3dviewer as q3d


def update(viewer):
    i = 0.0
    while True:
        i += 0.05
        time.sleep(0.1)
        viewer['traj'].set_data(np.array([np.sin(i) * i, np.cos(i) * i, i]))


def main():
    app = q3d.QApplication([])

    axis_item = q3d.AxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=10, spacing=1)
    traj_item = q3d.TrajectoryItem(width=2)

    viewer = q3d.Viewer(name='example')
    th = threading.Thread(target=update, args=(viewer, ))
    th.start()

    viewer.add_items({
        'grid': grid_item,
        'axis': axis_item,
        'traj': traj_item})

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
