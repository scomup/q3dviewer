#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from pathlib import Path

import q3dviewer as q3d
from q3dviewer import GLWidget
from q3dviewer.Qt.QtCore import QRect, QSize, Qt, Signal
from q3dviewer.Qt.QtGui import QIcon
from q3dviewer.Qt.QtWidgets import (
    QFileDialog, QDialog, QDoubleSpinBox, QHBoxLayout, QLabel, QMessageBox,
    QMdiArea, QRubberBand, QToolButton, QVBoxLayout, QWidget)
from q3dviewer.custom_items.cloud_item import CloudItem
from q3dviewer.tools.cloud_viewer import ProgressWindow
from q3dviewer.tools.cloud_editor.io import CloudEditorIO
from q3dviewer.tools.cloud_editor.operations import CloudEditorOperations
from q3dviewer.utils.helpers import get_version


ICON_DIR = Path(__file__).resolve().parent / 'icons'


def make_icon_button(parent, icon_name, tooltip):
    button = QToolButton(parent)
    button.setIcon(QIcon(str(ICON_DIR / f'{icon_name}.svg')))
    button.setIconSize(QSize(20, 20))
    button.setFixedSize(34, 34)
    button.setAutoRaise(True)
    button.setToolTip(tooltip)
    return button


class ToolDialog(QDialog):
    apply_requested = Signal()
    undo_requested = Signal()
    cancel_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowFlags(self.windowFlags() | Qt.Tool)
        self.setModal(False)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        self.content_layout = QHBoxLayout()
        layout.addLayout(self.content_layout)

        self.btn_undo = make_icon_button(
            self, 'undo', 'Undo last pending operation')
        self.btn_undo.clicked.connect(self.undo_requested)
        layout.addWidget(self.btn_undo)

        self.btn_apply = make_icon_button(
            self, 'check', 'Apply pending operations')
        self.btn_apply.clicked.connect(self.apply_requested)
        layout.addWidget(self.btn_apply)

        self.btn_cancel = make_icon_button(
            self, 'close', 'Cancel pending operations')
        self.btn_cancel.clicked.connect(self.cancel_requested)
        self.cancel_requested.connect(self.reject)
        layout.addWidget(self.btn_cancel)


class RadiusDeleteDialog(ToolDialog):
    radius_changed = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Delete by Radius')
        self.content_layout.addWidget(QLabel('Radius (m)'))

        self.radius_box = QDoubleSpinBox()
        self.radius_box.setDecimals(2)
        self.radius_box.setSingleStep(0.1)
        self.radius_box.setRange(0.01, 1000.0)
        self.radius_box.setValue(0.5)
        self.radius_box.valueChanged.connect(
            lambda value: self.radius_changed.emit(float(value)))
        self.content_layout.addWidget(self.radius_box)

    def set_radius(self, radius):
        self.radius_box.blockSignals(True)
        self.radius_box.setValue(radius)
        self.radius_box.blockSignals(False)


class RectangleDeleteDialog(ToolDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Rectangle delete')


class EditorGLWidget(GLWidget):
    """Translate mouse and key events into signals for the operations class."""

    point_clicked = Signal(int, int)
    rectangle_selected = Signal(object)
    undo_requested = Signal()
    sort_requested = Signal()

    def __init__(self):
        super().__init__()
        self._press_pos = None
        self._ctrl_select = False
        self._mode = None
        self._rubber_band = QRubberBand(QRubberBand.Rectangle, self)

    def set_mode(self, mode):
        self._mode = mode
        self._press_pos = None
        self._ctrl_select = False
        if mode != 'rectangle':
            self._rubber_band.hide()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Z and event.modifiers() & Qt.ControlModifier:
            self.undo_requested.emit()
            event.accept()
            return
        if event.text().lower() == 'r':
            self.sort_requested.emit()
            event.accept()
            return
        super().keyPressEvent(event)

    def mousePressEvent(self, event):
        self._press_pos = None
        self._ctrl_select = bool(event.modifiers() & Qt.ControlModifier)
        if (event.button() == Qt.LeftButton
                and self._ctrl_select
                and self._mode in ('radius', 'rectangle')):
            self._press_pos = event.pos()
            if self._mode == 'rectangle':
                self._rubber_band.setGeometry(
                    QRect(self._press_pos, self._press_pos))
                self._rubber_band.show()
                self._rubber_band.raise_()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if (self._mode == 'rectangle'
                and self._ctrl_select
                and self._press_pos is not None
                and event.buttons() & Qt.LeftButton):
            self._rubber_band.setGeometry(
                QRect(self._press_pos, event.pos()).normalized())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if (self._press_pos is not None
                and self._ctrl_select
                and event.button() == Qt.LeftButton):
            moved = (event.pos() - self._press_pos).manhattanLength()
            selection_rect = QRect(
                self._press_pos, event.pos()).normalized()
            self._press_pos = None
            self._rubber_band.hide()
            if self._mode == 'rectangle' and moved >= 4:
                self.rectangle_selected.emit(selection_rect)
            elif self._mode == 'radius' and moved < 4:
                self.point_clicked.emit(event.x(), event.y())
        self._ctrl_select = False
        super().mouseReleaseEvent(event)


class CloudEditor(q3d.Viewer):
    """Viewer shell: GUI state, display updates, and signal wiring only."""

    load_requested = Signal(object)
    load_file_requested = Signal(str, bool)
    save_requested = Signal(str)
    mode_requested = Signal(object)
    radius_requested = Signal(float)
    apply_requested = Signal()
    undo_requested = Signal()
    cancel_requested = Signal()

    def __init__(self, **kwargs):
        self.io = CloudEditorIO()
        self.active_mode = None
        super().__init__(
            **kwargs, gl_widget_class=EditorGLWidget)
        self.setAcceptDrops(True)

    def init_ui(self):
        center_widget = QWidget()
        self.setCentralWidget(center_widget)
        main_layout = QVBoxLayout(center_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        self.add_control_panel(main_layout)

        self.glwidget = self.gl_widget_class()
        setting_name = self.viewer_name.lower().replace(' ', '_')
        self.glwidget.set_setting_path(
            Path.home() / '.config' / 'q3dviewer' / setting_name / 'setting.json')
        main_layout.addWidget(self.glwidget, 1)

        self.mdi = QMdiArea(self.glwidget)
        self.mdi.setBackground(Qt.transparent)
        self.mdi.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.mdi.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        mdi_layout = QVBoxLayout(self.glwidget)
        mdi_layout.setContentsMargins(0, 0, 0, 0)
        mdi_layout.addWidget(self.mdi)
        self.mdi.show()

        self.operations = CloudEditorOperations(self.glwidget, self.io)
        self.load_requested.connect(self.io.load_files)
        self.load_file_requested.connect(self.io.load_file)
        self.save_requested.connect(self.operations.request_save)
        self.mode_requested.connect(self.operations.set_mode)
        self.radius_requested.connect(self.operations.set_radius)
        self.apply_requested.connect(self.operations.apply_operations)
        self.undo_requested.connect(self.operations.undo)
        self.cancel_requested.connect(self.operations.cancel_pending)

        self.operations.cloud_display_changed.connect(self.display_cloud)
        self.operations.marked_display_changed.connect(self.display_marked)
        self.operations.cloud_loaded.connect(self.set_cloud_color_mode)
        self.operations.info_changed.connect(self.update_info)
        self.operations.camera_center_changed.connect(
            self.set_camera_center)
        self.io.progress.connect(self.file_loading_progress)
        self.io.loading_finished.connect(self.file_loading_finished)
        self.io.operation_error.connect(self.show_operation_error)
        self.io.save_finished.connect(self.show_save_finished)
        self.glwidget.sort_requested.connect(self.force_sort)

    def add_control_panel(self, main_layout):
        toolbar = QWidget()
        toolbar_layout = QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(6, 4, 6, 4)
        toolbar_layout.setSpacing(2)

        self.btn_load = make_icon_button(
            toolbar, 'folder_open', 'Load point cloud files')
        self.btn_load.clicked.connect(self.load_clouds)
        toolbar_layout.addWidget(self.btn_load)

        self.btn_save = make_icon_button(
            toolbar, 'save', 'Save the edited point cloud')
        self.btn_save.clicked.connect(self.save_as)
        toolbar_layout.addWidget(self.btn_save)
        toolbar_layout.addSpacing(8)

        self.btn_radius_delete = make_icon_button(
            toolbar, 'track_changes',
            'Ctrl+click to delete points within the radius')
        self.btn_radius_delete.setCheckable(True)
        self.btn_radius_delete.toggled.connect(self.set_radius_mode)
        toolbar_layout.addWidget(self.btn_radius_delete)

        self.btn_rectangle_delete = make_icon_button(
            toolbar, 'select_all',
            'Ctrl+drag to delete points inside a rectangle')
        self.btn_rectangle_delete.setCheckable(True)
        self.btn_rectangle_delete.toggled.connect(self.set_rectangle_mode)
        toolbar_layout.addWidget(self.btn_rectangle_delete)

        self.label_info = QLabel('No cloud loaded')
        toolbar_layout.addStretch()
        toolbar_layout.addWidget(self.label_info)
        main_layout.addWidget(toolbar)

        self.radius_dialog = RadiusDeleteDialog(self)
        self.radius_dialog.radius_changed.connect(self.set_radius)
        self.radius_dialog.apply_requested.connect(self.apply_requested)
        self.radius_dialog.undo_requested.connect(self.undo_requested)
        self.radius_dialog.cancel_requested.connect(self.cancel_requested)
        self.radius_dialog.finished.connect(self.close_delete_tool)

        self.rectangle_dialog = RectangleDeleteDialog(self)
        self.rectangle_dialog.apply_requested.connect(self.apply_requested)
        self.rectangle_dialog.undo_requested.connect(self.undo_requested)
        self.rectangle_dialog.cancel_requested.connect(self.cancel_requested)
        self.rectangle_dialog.finished.connect(self.close_delete_tool)

    def _show_delete_tool(self, mode):
        self.active_mode = mode
        self.mode_requested.emit(mode)
        self.glwidget.set_mode(mode)
        self.radius_dialog.hide()
        self.rectangle_dialog.hide()
        dialog = (self.radius_dialog
                  if mode == 'radius' else self.rectangle_dialog)
        dialog.show()
        dialog.raise_()
        dialog.activateWindow()

        text_item = self['text']
        if text_item is not None:
            text_item.set_data(
                text='DELETE BY RADIUS: Ctrl+click to mark points within the radius'
                if mode == 'radius' else
                'RECTANGLE DELETE: Ctrl+drag to mark points')

    def close_delete_tool(self, _result=None):
        self.radius_dialog.hide()
        self.rectangle_dialog.hide()
        self.btn_radius_delete.blockSignals(True)
        self.btn_rectangle_delete.blockSignals(True)
        self.btn_radius_delete.setChecked(False)
        self.btn_rectangle_delete.setChecked(False)
        self.btn_radius_delete.blockSignals(False)
        self.btn_rectangle_delete.blockSignals(False)
        self.active_mode = None
        self.mode_requested.emit(None)
        self.glwidget.set_mode(None)
        text_item = self['text']
        if text_item is not None:
            text_item.set_data(text='')

    def set_radius_mode(self, enabled):
        if enabled:
            self.btn_rectangle_delete.blockSignals(True)
            self.btn_rectangle_delete.setChecked(False)
            self.btn_rectangle_delete.blockSignals(False)
            self._show_delete_tool('radius')
        elif not self.btn_rectangle_delete.isChecked():
            self.close_delete_tool()

    def set_rectangle_mode(self, enabled):
        if enabled:
            self.btn_radius_delete.blockSignals(True)
            self.btn_radius_delete.setChecked(False)
            self.btn_radius_delete.blockSignals(False)
            self._show_delete_tool('rectangle')
        elif not self.btn_radius_delete.isChecked():
            self.close_delete_tool()

    def set_radius(self, value):
        self.radius_requested.emit(float(value))

    def update_info(self, total, marked, pending):
        if total == 0:
            self.label_info.setText('No cloud loaded')
            return
        self.label_info.setText(
            f'Points: {total} | Marked: {marked} | Pending: {pending}')

    def set_camera_center(self, center):
        self.glwidget.set_cam_position(center=center)

    def display_cloud(self, cloud):
        cloud_item = self['cloud']
        if cloud_item is None:
            return
        if cloud is None:
            cloud_item.clear()
            return
        cloud_item.set_data(cloud)

    def display_marked(self, marked):
        marked_item = self['marked']
        if marked_item is None:
            return
        if marked is None or marked.size == 0:
            marked_item.clear()
        else:
            marked_item.set_data(marked)

    def set_cloud_color_mode(self, cloud):
        cloud_item = self['cloud']
        if cloud_item is None or cloud is None or cloud.size == 0:
            return
        has_intensity = cloud['irgb'][0] & 0xff000000 > 0
        has_rgb = cloud['irgb'][0] & 0x00ffffff > 0
        if has_rgb:
            cloud_item.set_color_mode('RGB')
        elif has_intensity:
            cloud_item.set_color_mode('I')
        else:
            cloud_item.set_color_mode('FLAT')

    def load_clouds(self):
        paths, _ = QFileDialog.getOpenFileNames(
            self, 'Load Point Clouds', str(Path.home()),
            'Point Cloud (*.pcd *.ply *.las *.e57)')
        if paths:
            self.start_loading(paths)

    def start_loading(self, files):
        self.progress_window = ProgressWindow(self)
        self.progress_window.show()
        self.load_requested.emit(files)

    def save_as(self):
        path, _ = QFileDialog.getSaveFileName(
            self, 'Save Cloud', str(Path.home() / 'edited.pcd'),
            'Point Cloud (*.pcd *.ply *.las *.e57)')
        if path:
            self.save_requested.emit(path)

    def open_cloud_file(self, file, append=False):
        self.load_file_requested.emit(str(file), append)

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        self.start_loading([
            url.toLocalFile() for url in event.mimeData().urls()])

    def file_loading_progress(self, current, total, file_name):
        if hasattr(self, 'progress_window'):
            self.progress_window.update_progress(current, total, file_name)

    def file_loading_finished(self):
        if hasattr(self, 'progress_window'):
            self.progress_window.close()

    def show_operation_error(self, message):
        QMessageBox.warning(self, 'Cloud Editor', message)

    def show_save_finished(self, path):
        QMessageBox.information(self, 'Save', f'Saved cloud to {path}')

    def force_sort(self):
        cloud_item = self['cloud']
        if cloud_item is not None:
            cloud_item.force_sort()


def print_help():
    from rich.console import Console
    from rich.table import Table

    console = Console()
    table = Table(show_header=False, box=None, padding=(0, 2))
    table.add_column(style='bold cyan', width=30)
    table.add_column(style='white')
    table.add_row('Load Files', 'Top toolbar folder icon or drag and drop')
    table.add_row('', 'Point clouds: .pcd, .ply, .las, .e57')
    table.add_row('', '')
    table.add_row('Delete Points', 'Top toolbar delete icons')
    table.add_row('', 'Delete by Radius opens a radius and Apply window')
    table.add_row('', 'Rectangle delete uses Ctrl+drag selection')
    table.add_row('', 'Ctrl+Z undoes the last pending operation')
    table.add_row('', '')
    table.add_row('Camera Controls', 'Right drag rotate, left drag pan, wheel zoom')
    table.add_row('Depth Sorting', "Press 'R' to force sort once")
    table.add_row('Settings', "Press 'M' to open settings")
    console.print()
    console.print(
        f'Cloud Editor ({get_version()}) Help\n')
    console.print(table)
    console.print()


def main():
    print_help()
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', help='the cloud file path')
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Editor'])
    viewer = CloudEditor(name='Cloud Editor')

    cloud_item = q3d.CloudSortItem(size=1, alpha=0.1, point_type='SPHERE')
    marked_item = CloudItem(
        size=5, alpha=1.0, color_mode='FLAT', color='#ff1493', point_type='SPHERE')
    marked_item.disable_setting()
    axis_item = q3d.AxisItem(size=0.5, width=5)
    axis_item.disable_setting()
    grid_item = q3d.GridItem(size=1000, spacing=20)
    center_item = q3d.CenterItem()
    text_item = q3d.Text2DItem(
        pos=(20, 40), text='', color='lime', size=16)
    text_item.disable_setting()

    viewer.add_items({
        'cloud': cloud_item,
        'marked': marked_item,
        'grid': grid_item,
        'axis': axis_item,
        'text': text_item,
        'center': center_item,
    })
    if args.path:
        viewer.open_cloud_file(args.path)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
