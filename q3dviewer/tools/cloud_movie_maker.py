#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QVBoxLayout, QProgressBar, QDialog, QLabel, QHBoxLayout
from PySide6.QtCore import QThread, Signal
from cloud_viewer import ProgressDialog,  FileLoaderThread
from PySide6.QtWidgets import QWidget
from PySide6 import QtCore


class Viewer2(q3d.Viewer):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def add_control_panel(self, main_layout):
        # Create a vertical layout for the settings
        setting_layout = QVBoxLayout()
        # Add XYZ spin boxes
        label_xyz = QLabel("Set XYZ:")
        setting_layout.addWidget(label_xyz)
        main_layout.addLayout(setting_layout)



def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="the cloud file path")
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Viewer'])
    viewer = Viewer2(name='Cloud Viewer')
    cloud_item = q3d.CloudIOItem(size=1, alpha=0.1)
    axis_item = q3d.AxisItem(size=0.5, width=5)
    grid_item = q3d.GridItem(size=1000, spacing=20)

    viewer.add_items(
        {'cloud': cloud_item, 'grid': grid_item, 'axis': axis_item})

    if args.path:
        pcd_fn = args.path
        viewer.open_cloud_file(pcd_fn)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
