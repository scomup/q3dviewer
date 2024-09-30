#!/usr/bin/env python3

import numpy as np
from q3dviewer.custom_items import *
from q3dviewer.basic_window import *
from pypcd4 import PointCloud

class CloudViewer(Viewer):
    def __init__(self):
        super(CloudViewer, self).__init__(name="Cloud Viewer")
        self.setAcceptDrops(True)

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
        if file.endswith('.pcd'):
            pc = PointCloud.from_path(file).pc_data
            if 'rgb' in pc.dtype.names:
                color = pc["rgb"].astype(np.uint32)
                cloud_item.setColorMode('RGB')
            elif 'intensity' in pc.dtype.names:
                color = pc["intensity"].astype(np.uint32)
                cloud_item.setColorMode('I')
            else:
                color = pc['z'].astype(np.uint32)
                cloud_item.setColorMode('#FFFFFF')
            cloud = np.rec.fromarrays(
                [np.stack([pc["x"], pc["y"], pc["z"]], axis=1), color],
                dtype=cloud_item.data_type)
        elif file.endswith('.npy'):
            pc = np.load(file)
            cloud = np.rec.fromarrays(
                [np.stack([pc[:, 0], pc[:, 1], pc[:, 2]], axis=1), pc[:, 3].astype(np.uint32)],
                dtype=cloud_item.data_type)
        cloud_item.setData(data=cloud)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", help="the pcd path")
    args = parser.parse_args()
    app = QApplication([])
    viewer = CloudViewer()
    cloud_item = CloudIOItem(size=1, alpha=0.1)
    axis_item = GLAxisItem(size=0.5, width=5)
    gird_item = GridItem(size=1000, spacing=20)
    # viewer.viewerWidget.setBackgroundColor(255, 255, 255, 255)

    viewer.addItems({'cloud': cloud_item, 'grid': gird_item, 'axis': axis_item})

    if args.pcd:
        pcd_fn = args.pcd
        viewer.openCloudFile(pcd_fn)

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
