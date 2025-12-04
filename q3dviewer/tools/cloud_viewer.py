#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from q3dviewer.Qt.QtWidgets import QVBoxLayout, QProgressBar, QDialog, QLabel
from q3dviewer.Qt.QtCore import QThread, Signal, Qt
from q3dviewer.Qt.QtGui import QKeyEvent
from q3dviewer import GLWidget


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
        mesh_item = self.viewer['mesh']
        for i, url in enumerate(self.files):
            # if the file is a mesh file, use mesh_item to load
            file_path = url.toLocalFile()
            file_path = url.toLocalFile()
            self.viewer.progress_dialog.set_file_name(file_path)
            if url.toLocalFile().lower().endswith(('.stl')):
                from q3dviewer.utils.cloud_io import load_stl
                verts, faces = load_stl(file_path)
                mesh_item.set_data(verts=verts, faces=faces)
                break
            else:
                cloud = cloud_item.load(file_path, append=(i > 0))
                center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
                self.viewer.glwidget.set_cam_position(center=center)
                self.progress.emit(int((i + 1) / len(self.files) * 100))
        self.finished.emit()


class CustomGLWidget(GLWidget):
    def __init__(self, viewer):
        super().__init__()
        self.viewer = viewer
        self.selected_points = []

    def mousePressEvent(self, event):
        # ctrl + left click to add select point
        # ctrl + right click to remove a point from selected points
        if event.button() == Qt.LeftButton and event.modifiers() & Qt.ControlModifier:
            x, y = event.x(), event.y()
            p = self.get_point(x, y)
            if p is not None:
                self.selected_points.append(p)
                self.update_marker()
        elif event.button() == Qt.RightButton and event.modifiers() & Qt.ControlModifier:
            if len(self.selected_points) > 0:
                self.selected_points.pop()
                self.update_marker()
        super().mouseReleaseEvent(event)

    def update_marker(self):
        marks = []
        for p in self.selected_points:
            m = {'text': '',
                 'position': p,
                 'color': (0.0, 1.0, 0.0, 1.0),
                 'font_size': 16,
                 'point_size': 5.0,
                 'line_width': 1.0}
            marks.append(m)
        
        # calculate distances between consecutive points
        self.viewer['marker'].set_data(data=marks, append=False)
        if len(self.selected_points) >= 2:
            total_dists = 0.0
            for i in range(1, len(self.selected_points)):
                p1 = self.selected_points[i - 1]
                p2 = self.selected_points[i]
                dist = np.linalg.norm(np.array(p2) - np.array(p1))
                total_dists += dist
            self.viewer['text'].set_data(text=f'Total Distance: {total_dists:.2f} meter')
        else:
            self.viewer['text'].set_data(text='')


class CloudViewer(q3d.Viewer):
    def __init__(self, **kwargs):
        gl_widget_class=lambda: CustomGLWidget(self)
        super(CloudViewer, self).__init__(
            **kwargs,  gl_widget_class=gl_widget_class)
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

# print a quick help message
def print_help():
    # ANSI color codes
    GREEN = '\033[92m'
    BLUE = '\033[94m'
    BOLD = '\033[1m'
    END = '\033[0m'
    
    help_msg = f"""
{BOLD}Cloud Viewer Help:{END}
{GREEN}• Drag and drop cloud files into the viewer to load them.{END}
    {BLUE}- support .pcd, .ply, .las, .e57, for point clouds.{END}
    {BLUE}- support .stl for mesh files.{END}
{GREEN}• Measure distance between points:{END}
    {BLUE}- Hold Ctrl and left-click to select points on the cloud.{END}
    {BLUE}- Hold Ctrl and right-click to remove the last selected point.{END}
    {BLUE}- The total distance between selected points will be displayed.{END}
{GREEN}• Press 'M' to open the settings window.{END}
    {BLUE}- Use the settings window to adjust item properties.{END}
    """
    print(help_msg)

def main():
    print_help()
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="the cloud file path")
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Viewer'])
    viewer = CloudViewer(name='Cloud Viewer')
    cloud_item = q3d.CloudIOItem(size=1, alpha=0.1)
    axis_item = q3d.AxisItem(size=0.5, width=5)
    axis_item.disable_setting()
    grid_item = q3d.GridItem(size=1000, spacing=20)
    marker_item = q3d.Text3DItem()  # Changed from CloudItem to Text3DItem
    text_item = q3d.Text2DItem(pos=(20, 40), text="", color='lime', size=16)
    text_item.disable_setting()
    mesh_item = q3d.MeshItem()  # Added MeshIOItem for mesh support

    viewer.add_items(
        {'marker': marker_item, 
         'cloud': cloud_item,
         'mesh': mesh_item,
         'grid': grid_item, 
         'axis': axis_item, 
         'text': text_item,})

    if args.path:
        pcd_fn = args.path
        viewer.open_cloud_file(pcd_fn)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
