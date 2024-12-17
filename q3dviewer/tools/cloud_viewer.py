#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from pypcd4 import PointCloud
import q3dviewer as q3d
from PyQt5.QtGui import  QVector3D

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
        for i, url in enumerate(event.mimeData().urls()):
            append = False
            if i > 0:
                append = True
            print(url.toLocalFile(), append)
            file_path = url.toLocalFile()
            self.openCloudFile(file_path, append)

    def openCloudFile(self, file, append=False):
        cloud_item = self['cloud']
        if cloud_item is None:
            print("Can't find clouditem.")
            return
        cloud = cloud_item.load(file, append=append)
        center = np.mean(cloud['xyz'], axis=0)
        self.glv_widget.setCameraPosition(pos=QVector3D(center[0], center[1], center[2]))
        # self.glv_widget.setCameraPosition(distance=200)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="the cloud file path")
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Viewer'])
    viewer = CloudViewer(name='Cloud Viewer')
    cloud_item = q3d.CloudIOItem(size=1, alpha=0.1)
    axis_item = q3d.GLAxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=1000, spacing=20)

    viewer.addItems(
        {'cloud': cloud_item, 'grid': grid_item, 'axis': axis_item})

    if args.path:
        pcd_fn = args.path
        viewer.openCloudFile(pcd_fn)

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
