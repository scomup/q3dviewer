#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from q3dviewer.Qt.QtWidgets import QVBoxLayout, QDialog, QLabel
from q3dviewer.Qt.QtCore import QThread, Signal, Qt
from q3dviewer import GLWidget
from q3dviewer.utils.helpers import get_version

class ProgressWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Loading")
        self.setModal(True)
        self.setMinimumWidth(400)
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

    def update_progress(self, current, total, file_name):
        text = f"[{current}/{total}] loading file: {file_name}"
        self.label.setText(text)


class FileLoaderThread(QThread):
    progress = Signal(int, int, str)  # current, total, filename
    finished = Signal()

    def __init__(self, viewer, files):
        super().__init__()
        self.viewer = viewer
        self.files = files

    def run(self):
        cloud_item = self.viewer['cloud']
        mesh_item = self.viewer['mesh']
        total = len(self.files)
        for i, url in enumerate(self.files):
            # if the file is a mesh file, use mesh_item to load
            file_path = url.toLocalFile()
            import os
            file_name = os.path.basename(file_path)
            self.progress.emit(i + 1, total, file_name)
            
            if url.toLocalFile().lower().endswith(('.stl')):
                from q3dviewer.utils.cloud_io import load_stl
                mesh = load_stl(file_path)
                mesh_item.set_data(mesh)
            else:
                cloud = cloud_item.load(file_path, append=(i > 0))
                center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
                self.viewer.glwidget.set_cam_position(center=center)
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
        self.progress_window = ProgressWindow(self)
        self.progress_window.show()
        files = event.mimeData().urls()
        self.progress_thread = FileLoaderThread(self, files)
        self.progress_thread.progress.connect(self.file_loading_progress)
        self.progress_thread.finished.connect(self.file_loading_finished)
        self.progress_thread.start()

    def file_loading_progress(self, current, total, file_name):
        self.progress_window.update_progress(current, total, file_name)

    def file_loading_finished(self):
        self.progress_window.close()

    def open_cloud_file(self, file, append=False):
        cloud_item = self['cloud']
        if cloud_item is None:
            print("Can't find clouditem.")
            return
        cloud = cloud_item.load(file, append=append)
        center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
        self.glwidget.set_cam_position(center=center)

# print a quick help message using rich
def print_help():
    from rich.console import Console
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    
    console = Console()
    
    # Create a table for better organization
    table = Table(show_header=False, box=None, padding=(0, 2))
    table.add_column(style="bold cyan", width=30)
    table.add_column(style="white")
    
    # File loading section
    table.add_row("📁 Load Files","Drag and drop files into the viewer")
    table.add_row("","[dim]• Point clouds: .pcd, .ply, .las, .e57[/dim]")
    table.add_row("","[dim]• Mesh files: .stl[/dim]")
    table.add_row("", "")
    
    # Measurement section
    table.add_row("📏 Measure Distance", "Interactive point measurement")
    table.add_row("","[dim]• Ctrl + Left Click: Add measurement point[/dim]")
    table.add_row("","[dim]• Ctrl + Right Click: Remove last point[/dim]")
    table.add_row("","[dim]• Total distance displayed automatically[/dim]")
    table.add_row("", "")
    
    # Camera controls
    table.add_row("🎥 Camera Controls","Navigate the 3D scene")
    table.add_row("","[dim]• Double Click: Set camera center to point[/dim]")
    table.add_row("","[dim]• Right Drag: Rotate view[/dim]")
    table.add_row("","[dim]• Left Drag: Pan view[/dim]")
    table.add_row("","[dim]• Mouse Wheel: Zoom in/out[/dim]")
    table.add_row("", "")
    
    # Settings section
    table.add_row("⚙️  Settings","Press [bold green]'M'[/bold green] to open settings window")
    table.add_row("","[dim]Adjust visualization properties[/dim]")
    
    # Print title and table without border
    console.print()
    console.print(f"[bold magenta]☁️  Cloud Viewer ({get_version()}) Help[/bold magenta]\n")
    console.print(table)
    console.print()

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
    mesh_item = q3d.StaticMeshItem()

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
