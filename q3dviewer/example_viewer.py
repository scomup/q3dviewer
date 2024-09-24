#!/usr/bin/env python3

import numpy as np
from q3dviewer.custom_items import *
from q3dviewer.basic_window import *
import threading
import time


def update(viewer):
    i = 0.0
    while True:
        i += 0.05
        time.sleep(0.1)
        viewer['traj'].setData(np.array([np.sin(i) * i, np.cos(i) * i, i]))


def main():
    app = QApplication([])

    axis_item = GLAxisItem(size=0.5, width=5)
    gird_item = GridItem(size=10, spacing=1)
    traj_item = TrajectoryItem(width=2)

    viewer = Viewer(name='example')
    th = threading.Thread(target=update, args=(viewer, ))
    th.start()

    viewer.addItems({
        'grid': gird_item,
        'axis': axis_item,
        'traj': traj_item})

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
