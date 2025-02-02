#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from PySide6.QtWidgets import QVBoxLayout, QProgressBar, QDialog, QLabel
from PySide6.QtCore import QThread, Signal


class ProgressDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Loading Cloud File")
        self.setModal(True)
        self.progress_bar = QProgressBar(self)
        self.file_label = QLabel(self)
        layout = QVBoxLayout()
        layout.addWidget(self.file_label)
        layout.addWidget(self.progress_bar)
        self.setLayout(layout)

    def set_value(self, value):
        self.progress_bar.setValue(value)

    def set_file_name(self, file_name):
        self.file_label.setText(f"Loading: {file_name}")

    def closeEvent(self, event):
        if self.parent().progress_thread and self.parent().progress_thread.isRunning():
            event.ignore()
        else:
            event.accept()


class FileLoaderThread(QThread):
    progress = Signal(int)
    finished = Signal()

    def __init__(self, viewer, files):
        super().__init__()
        self.viewer = viewer
        self.files = files

    def run(self):
        cloud_item = self.viewer['cloud']
        for i, url in enumerate(self.files):
            file_path = url.toLocalFile()
            self.viewer.progress_dialog.set_file_name(file_path)
            cloud = cloud_item.load(file_path, append=(i > 0))
            center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
            self.viewer.glwidget.set_cam_position(center=center)
            self.progress.emit(int((i + 1) / len(self.files) * 100))
        self.finished.emit()


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
        self.progress_dialog = ProgressDialog(self)
        self.progress_dialog.show()
        files = event.mimeData().urls()
        self.progress_thread = FileLoaderThread(self, files)
        self['cloud'].load(files[0].toLocalFile(), append=False)
        self.progress_thread.progress.connect(self.file_loading_progress)
        self.progress_thread.finished.connect(self.file_loading_finished)
        self.progress_thread.start()

    def file_loading_progress(self, value):
        self.progress_dialog.set_value(value)

    def file_loading_finished(self):
        self.progress_dialog.close()

    def open_cloud_file(self, file, append=False):
        cloud_item = self['cloud']
        if cloud_item is None:
            print("Can't find clouditem.")
            return
        cloud = cloud_item.load(file, append=append)
        center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
        self.glwidget.set_cam_position(center=center)


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
    app.exec()


if __name__ == '__main__':
    main()
