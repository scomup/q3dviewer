#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, \
    QSpinBox, QWidget, QVBoxLayout, QHBoxLayout, QCheckBox
from PySide6 import QtCore

class CloudViewer(q3d.Viewer):
    def __init__(self, **kwargs):
        super(CloudViewer, self).__init__(**kwargs)
        self.setAcceptDrops(True)

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        """
        Overwrite the drop event to open the cloud file.
        """
        for i, url in enumerate(event.mimeData().urls()):
            file_path = url.toLocalFile()
            self.open_cloud_file(file_path, append=(i > 0))

    def open_cloud_file(self, file, append=False):
        cloud_item = self['cloud']
        if cloud_item is None:
            print("Can't find clouditem.")
            return
        cloud = cloud_item.load(file, append=append)
        center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
        self.glwidget.set_cam_position(pos=center)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="the cloud file path")
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Viewer'])
    viewer = CloudViewer(name='Cloud Viewer')
    cloud_item = q3d.CloudIOItem(size=1, alpha=0.1)
    axis_item = q3d.AxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=1000, spacing=20)

    viewer.add_items(
        {'cloud': cloud_item, 'grid': grid_item, 'axis': axis_item})

    if args.path:
        pcd_fn = args.path
        viewer.open_cloud_file(pcd_fn)

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
