#!/usr/bin/env python3

import numpy as np
import q3dviewer as q3d
from PySide6.QtCore import QTimer

i = 0.0

def update(viewer):
    global i
    i += 0.05
    new = np.array([np.sin(i) * i, np.cos(i) * i, i])
    viewer['traj'].set_data(new, append=True)
    viewer.update()
    return i


def main():
    # add a qt application
    app = q3d.QApplication([])

    # create opengl items
    axis_item = q3d.AxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=10, spacing=1)
    traj_item = q3d.LineItem(width=2)

    # create viewer
    viewer = q3d.Viewer(name='example')

    # add all items to viewer
    viewer.add_items({
        'grid': grid_item,
        'axis': axis_item,
        'traj': traj_item})

    # dynamic update by timer
    timer = QTimer()
    timer.timeout.connect(lambda: update(viewer))
    timer.start(100)

    # show viewer and start application
    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
