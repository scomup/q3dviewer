#!/usr/bin/env python3

"""
Copyright 2024  Liu Yang
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from pypcd4 import PointCloud
import q3dviewer as q3d


class CloudViewer(q3d.Viewer):
    def __init__(self, **kwargs):
        super(CloudViewer, self).__init__(**kwargs)
        self.setAcceptDrops(True)

    def dragEnterEvent(self, event):
        # override
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        # override
        for url in event.mimeData().urls():
            file_path = url.toLocalFile()
            self.open_cloud_file(file_path)

    def open_cloud_file(self, file):
        cloud_item = self['cloud']
        if cloud_item is None:
            print("Can't find clouditem.")
            return
        if file.endswith('.pcd'):
            pc = PointCloud.from_path(file).pc_data
            if 'rgb' in pc.dtype.names:
                color = pc["rgb"].astype(np.uint32)
                cloud_item.set_color_mode('RGB')
            elif 'intensity' in pc.dtype.names:
                color = pc["intensity"].astype(np.uint32)
                cloud_item.set_color_mode('I')
            else:
                color = pc['z'].astype(np.uint32)
                cloud_item.set_color_mode('#FFFFFF')
            cloud = np.rec.fromarrays(
                [np.stack([pc["x"], pc["y"], pc["z"]], axis=1), color],
                dtype=cloud_item.data_type)
        elif file.endswith('.npy'):
            pc = np.load(file)
            cloud = np.rec.fromarrays(
                [np.stack([pc[:, 0], pc[:, 1], pc[:, 2]], axis=1),
                 pc[:, 3].astype(np.uint32)],
                dtype=cloud_item.data_type)
        cloud_item.set_data(data=cloud)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd", help="the pcd path")
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Viewer'])
    viewer = CloudViewer(name='Cloud Viewer')
    cloud_item = q3d.CloudIOItem(size=1, alpha=0.1)
    axis_item = q3d.GLAxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=1000, spacing=20)

    viewer.add_items(
        {'cloud': cloud_item, 'grid': grid_item, 'axis': axis_item})

    if args.pcd:
        pcd_fn = args.pcd
        viewer.open_cloud_file(pcd_fn)

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
