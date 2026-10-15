#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from q3dviewer.Qt.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QPushButton, QDoubleSpinBox, QGroupBox,
    QFileDialog, QMessageBox)
from q3dviewer.Qt.QtCore import QThread, Signal, Qt
from q3dviewer import GLWidget
from q3dviewer.custom_items.cloud_item import CloudItem
from q3dviewer.utils.helpers import get_version
from q3dviewer.utils.cloud_io import read_crs_from_file
from q3dviewer.tools.cloud_viewer import ProgressWindow, get_cloud_center
from pathlib import Path

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


class _BruteTree:
    """Fallback radius search when scipy is not available."""

    def __init__(self, xyz):
        self.xyz = xyz

    def query_ball_point(self, p, r):
        d2 = np.sum((self.xyz - np.asarray(p, dtype=np.float32)) ** 2, axis=1)
        return np.flatnonzero(d2 <= r * r)


def build_tree(xyz):
    if cKDTree is not None:
        return cKDTree(xyz)
    return _BruteTree(xyz)


# ---------------------------------------------------------------- operations
# An operation is a dict with a mandatory 'type' key; all other keys are
# type-specific parameters, e.g. {'type': 'del', 'center': [x, y, z], 'radius': 0.5}.
# A handler receives (editor, keep_mask, op) and returns the new keep_mask,
# so ops are applied strictly in list order. Register new types in OP_HANDLERS.

def op_del(editor, keep, op):
    idx = np.asarray(editor.tree.query_ball_point(
        op['center'], op['radius']), dtype=np.int64)
    keep = keep.copy()
    keep[idx] = False
    return keep


OP_HANDLERS = {
    'del': op_del,
}


def make_op(op_type, **params):
    if op_type not in OP_HANDLERS:
        raise ValueError(f"Unknown operation type: {op_type}")
    return {'type': op_type, **params}


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
        satellite_map_item = self.viewer['satellite_map']
        total = len(self.files)
        for i, url in enumerate(self.files):
            file_path = Path(url.toLocalFile())
            file_name = file_path.name
            self.progress.emit(i + 1, total, file_name)

            if file_path.suffix.lower() == '.stl':
                from q3dviewer.utils.cloud_io import load_stl
                mesh_item.set_data(load_stl(file_path))
                continue

            if file_path.suffix.lower() == '.ply':
                from q3dviewer.utils.cloud_io import is_ply_mesh, load_ply_mesh
                if is_ply_mesh(file_path):
                    mesh_item.set_data(load_ply_mesh(file_path))
                    continue

            cloud = cloud_item.load(str(file_path), append=(i > 0))
            if cloud is None:
                continue
            self.viewer.merge_cloud(cloud, append=(i > 0))
            center = get_cloud_center(cloud)
            self.viewer.glwidget.set_cam_position(center=center)

            try:
                epsg_code, bbox_min, bbox_max = read_crs_from_file(file_path)
                if epsg_code is not None:
                    satellite_map_item.set_crs(
                        epsg_code, (bbox_min + bbox_max) / 2)
                    print(
                        f"[CloudEditor] Satellite map configured from {file_name} (EPSG:{epsg_code})")
            except Exception as e:
                print(f"[CloudEditor] Could not configure satellite map: {e}")
        self.finished.emit()


class EditorGLWidget(GLWidget):
    def __init__(self, viewer):
        super().__init__()
        self.viewer = viewer
        self._press_pos = None

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Z and event.modifiers() & Qt.ControlModifier:
            self.viewer.undo()
            event.accept()
            return
        if event.text().lower() == 'r':
            self.viewer['cloud'].force_sort()
        else:
            super().keyPressEvent(event)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.viewer.edit_mode:
            self._press_pos = event.pos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        # A press/release without drag is a click; drags keep panning the view.
        if self._press_pos is not None and event.button() == Qt.LeftButton:
            moved = (event.pos() - self._press_pos).manhattanLength()
            self._press_pos = None
            if moved < 4:
                p = self.get_point(event.x(), event.y())
                if p is not None:
                    self.viewer.add_op(make_op(
                        'del', center=[float(v) for v in p],
                        radius=self.viewer.radius))
        super().mouseReleaseEvent(event)


class CloudEditor(q3d.Viewer):
    def __init__(self, **kwargs):
        self.cloud = None      # CPU copy of the cloud (ops applied)
        self.tree = None
        self.ops = []          # pending operations, see OP_HANDLERS
        self.keep = None       # preview: bool mask after pending ops
        self.edit_mode = False
        self.radius = 0.5

        def gl_widget_class(): return EditorGLWidget(self)
        super(CloudEditor, self).__init__(
            **kwargs, gl_widget_class=gl_widget_class)
        self.setAcceptDrops(True)

    # ------------------------------------------------------------------ UI
    def add_control_panel(self, main_layout):
        panel_layout = QVBoxLayout()
        panel_layout.setAlignment(Qt.AlignTop)

        group = QGroupBox("Edit")
        group_layout = QVBoxLayout()

        self.btn_edit = QPushButton("Delete Mode")
        self.btn_edit.setCheckable(True)
        self.btn_edit.setToolTip(
            "When enabled, left click adds a 'del' operation for points "
            "within radius of the clicked point. Ctrl+Z undoes the last one.")
        self.btn_edit.toggled.connect(self.set_edit_mode)
        group_layout.addWidget(self.btn_edit)

        self.box_radius = QDoubleSpinBox()
        self.box_radius.setPrefix("Radius (m): ")
        self.box_radius.setDecimals(2)
        self.box_radius.setSingleStep(0.1)
        self.box_radius.setRange(0.01, 1000.0)
        self.box_radius.setValue(self.radius)
        self.box_radius.valueChanged.connect(self.set_radius)
        group_layout.addWidget(self.box_radius)

        self.btn_apply = QPushButton("Apply")
        self.btn_apply.setToolTip("Run all pending operations in order.")
        self.btn_apply.clicked.connect(self.apply_ops)
        group_layout.addWidget(self.btn_apply)

        self.btn_save = QPushButton("Save As...")
        self.btn_save.clicked.connect(self.save_as)
        group_layout.addWidget(self.btn_save)

        group.setLayout(group_layout)
        panel_layout.addWidget(group)

        self.label_info = QLabel()
        self.label_info.setWordWrap(True)
        panel_layout.addWidget(self.label_info)

        panel = QWidget()
        panel.setLayout(panel_layout)
        panel.setFixedWidth(220)
        main_layout.addWidget(panel)
        self.update_info()

    def set_edit_mode(self, enabled):
        self.edit_mode = bool(enabled)
        if self.edit_mode:
            self.glwidget.setCursor(Qt.CrossCursor)
            self['text'].set_data(
                text='DELETE MODE: left click to mark, Ctrl+Z undo, Apply to commit')
        else:
            self.glwidget.setCursor(Qt.ArrowCursor)
            self['text'].set_data(text='')

    def set_radius(self, value):
        self.radius = float(value)

    def update_info(self):
        if self.cloud is None:
            self.label_info.setText("No cloud loaded.\nDrag and drop a file.")
            return
        total = self.cloud.shape[0]
        marked = int(np.count_nonzero(~self.keep))
        self.label_info.setText(
            f"Points: {total}\n"
            f"Marked: {marked}\n"
            f"Pending ops: {len(self.ops)}")

    # ---------------------------------------------------------------- data
    def merge_cloud(self, cloud, append=False):
        cloud = cloud[np.isfinite(cloud['xyz']).all(axis=1)]
        if append and self.cloud is not None:
            self.cloud = np.concatenate([self.cloud, cloud])
        else:
            self.cloud = cloud
        self.ops = []
        self.keep = np.ones(self.cloud.shape[0], dtype=bool)
        self.tree = build_tree(self.cloud['xyz'])

    def run_ops(self, ops):
        keep = np.ones(self.cloud.shape[0], dtype=bool)
        for op in ops:
            keep = OP_HANDLERS[op['type']](self, keep, op)
        return keep

    def add_op(self, op):
        if self.tree is None:
            return
        self.ops.append(op)
        self.keep = OP_HANDLERS[op['type']](self, self.keep, op)
        self.refresh_display()

    def undo(self):
        if not self.ops:
            return
        self.ops.pop()
        self.keep = self.run_ops(self.ops)
        self.refresh_display()

    def apply_ops(self):
        if self.cloud is None or not self.ops:
            return
        keep = self.run_ops(self.ops)
        self.cloud = self.cloud[keep]
        self.ops = []
        self.keep = np.ones(self.cloud.shape[0], dtype=bool)
        self.tree = build_tree(self.cloud['xyz'])
        self.refresh_display()

    def refresh_display(self):
        # Points removed by pending ops are drawn only by the red 'marked' item.
        if self.cloud is None:
            self['cloud'].clear()
            self['marked'].clear()
        else:
            self['cloud'].set_data(self.cloud[self.keep])
            if self.keep.all():
                self['marked'].clear()
            else:
                self['marked'].set_data(self.cloud[~self.keep])
        self.update_info()

    def save_as(self):
        if self.cloud is None:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Cloud", str(Path.home() / "edited.pcd"),
            "Point Cloud (*.pcd *.ply *.las *.e57)")
        if not path:
            return
        suffix = Path(path).suffix.lower()
        if suffix == '.pcd':
            from q3dviewer.utils.cloud_io import save_pcd as func
        elif suffix == '.ply':
            from q3dviewer.utils.cloud_io import save_ply as func
        elif suffix == '.las':
            from q3dviewer.utils.cloud_io import save_las as func
        elif suffix == '.e57':
            from q3dviewer.utils.cloud_io import save_e57 as func
        else:
            QMessageBox.warning(self, "Save", f"Unsupported file type: {suffix}")
            return
        try:
            func(self.cloud[self.keep], path)
            QMessageBox.information(self, "Save", f"Saved cloud to {path}")
        except Exception as e:
            QMessageBox.warning(self, "Save", f"Cannot save to {path}\n{e}")

    # --------------------------------------------------------- file loading
    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
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
        self.refresh_display()

    def open_cloud_file(self, file, append=False):
        cloud_item = self['cloud']
        if cloud_item is None:
            print("Can't find clouditem.")
            return
        cloud = cloud_item.load(str(file), append=append)
        if cloud is None:
            return
        self.merge_cloud(cloud, append=append)
        self.glwidget.set_cam_position(center=get_cloud_center(cloud))
        self.refresh_display()


def print_help():
    from rich.console import Console
    from rich.table import Table

    console = Console()
    table = Table(show_header=False, box=None, padding=(0, 2))
    table.add_column(style="bold cyan", width=30)
    table.add_column(style="white")

    table.add_row("📁 Load Files", "Drag and drop files into the viewer")
    table.add_row("", "[dim]• Point clouds: .pcd, .ply, .las, .e57[/dim]")
    table.add_row("", "")
    table.add_row("✂️  Delete Points", "Left panel buttons")
    table.add_row("", "[dim]• [bold green]Delete Mode[/bold green]: left click marks points within radius (KD-tree) in red[/dim]")
    table.add_row("", "[dim]• [bold green]Ctrl+Z[/bold green]: undo last operation[/dim]")
    table.add_row("", "[dim]• [bold green]Apply[/bold green]: run all pending operations in order[/dim]")
    table.add_row("", "[dim]• [bold green]Save As...[/bold green]: save the edited cloud[/dim]")
    table.add_row("", "")
    table.add_row("🎥 Camera Controls", "Navigate the 3D scene")
    table.add_row("", "[dim]• Double Click: Set camera center to point[/dim]")
    table.add_row("", "[dim]• Right Drag: Rotate view[/dim]")
    table.add_row("", "[dim]• Left Drag: Pan view[/dim]")
    table.add_row("", "[dim]• Mouse Wheel: Zoom in/out[/dim]")
    table.add_row("", "")
    table.add_row("🔄 Depth Sorting", "• [dim]Press [bold green]'R'[/bold green] to force sort once[/dim]")
    table.add_row("⚙️  Settings", "• [dim]Press [bold green]'M'[/bold green] to open settings window[/dim]")

    console.print()
    console.print(
        f"[bold magenta]✂️  Cloud Editor ({get_version()}) Help[/bold magenta]\n")
    console.print(table)
    console.print()


def main():
    print_help()
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", help="the cloud file path")
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Editor'])
    viewer = CloudEditor(name='Cloud Editor')
    cloud_item = q3d.CloudSortItem(size=1, alpha=0.1)
    marked_item = CloudItem(size=3, alpha=1.0, color_mode='FLAT', color='red')
    marked_item.disable_setting()
    axis_item = q3d.AxisItem(size=0.5, width=5)
    axis_item.disable_setting()
    grid_item = q3d.GridItem(size=1000, spacing=20)
    center_item = q3d.CenterItem()
    text_item = q3d.Text2DItem(pos=(20, 40), text="", color='red', size=16)
    text_item.disable_setting()
    mesh_item = q3d.StaticMeshItem()
    satellite_map_item = q3d.SatelliteMapItem()

    viewer.add_items(
        {'cloud': cloud_item,
         'marked': marked_item,
         'mesh': mesh_item,
         'grid': grid_item,
         'axis': axis_item,
         'text': text_item,
         'center': center_item,
         'satellite_map': satellite_map_item})
    if args.path:
        viewer.open_cloud_file(args.path)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
