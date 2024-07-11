#!/usr/bin/env python3

import numpy as np
from fast_viewer.custom_items import *
from fast_viewer.basic_window import *
from pypcd4 import PointCloud


class CloudViewer(Viewer):
    def __init__(self):
        super(CloudViewer, self).__init__(name="Cloud Viewer")
        self.setAcceptDrops(True)

    def follow(self, p):
        self.viewer.setCameraPosition(pos=QVector3D(p[0], p[1], p[2]))

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        for url in event.mimeData().urls():
            file_path = url.toLocalFile()
            self.openCloudFile(file_path)

    def openCloudFile(self, file):
            cloud_item = self['cloud']
            if cloud_item is None:
                print("Can't find clouditem.")
                return
            if cloud_item.__class__.__name__ != 'CloudIOItem':
                print("Is not a CloudIOItem.")
                return
            cloud_item.load(file)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", help="the pcd path")
    args = parser.parse_args()
    app = QApplication([])
    viewer = CloudViewer()

    cloudItem = CloudIOItem(size=1, alpha=0.1)
    axisItem = GLAxisItem(size=0.5, width=5)
    gridItem = GridItem(size=1000, spacing=20)

    viewer.addItems({'grid': gridItem, 'axis': axisItem, 'cloud': cloudItem})

    if args.pcd:
        pcd_fn = args.pcd
        viewer.openCloudFile(pcd_fn)

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
