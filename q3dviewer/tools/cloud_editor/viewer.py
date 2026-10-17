#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from pathlib import Path

import q3dviewer as q3d
from q3dviewer import GLWidget
from q3dviewer.Qt.QtCore import QRect, Qt, Signal
from q3dviewer.Qt.QtWidgets import (
    QFileDialog, QFrame, QHBoxLayout, QMessageBox, QMdiArea, QRubberBand,
    QVBoxLayout, QWidget)
from q3dviewer.custom_items.cloud_item import CloudItem
from q3dviewer.tools.cloud_viewer import ProgressWindow, get_cloud_center
from q3dviewer.tools.cloud_editor.brush_tool import (
    BrushDeleteTool, BrushDeleteWindow)
from q3dviewer.tools.cloud_editor.io import CloudEditorIO
from q3dviewer.tools.cloud_editor.measurement import (
    MeasurementTool, MeasurementWindow)
from q3dviewer.tools.cloud_editor.operations import CloudEditorOperations
from q3dviewer.tools.cloud_editor.points_selector import PointsSelector
from q3dviewer.tools.cloud_editor.rectangle_tool import (
    RectangleDeleteTool, RectangleDeleteWindow)
from q3dviewer.tools.cloud_editor.tool_window import make_icon_button
from q3dviewer.utils.helpers import get_version


class EditorGLWidget(GLWidget):
    """Translate mouse and key events into signals for the tool classes."""

    brush_drag_started = Signal()
    brush_point_dragged = Signal(int, int)
    brush_drag_finished = Signal()
    measurement_point_clicked = Signal(int, int)
    measurement_undo_requested = Signal()
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
            if self._mode == 'measurement':
                self.measurement_undo_requested.emit()
            else:
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
        if (self._mode == 'measurement'
            and self._ctrl_select
            and event.button() == Qt.LeftButton):
            self._press_pos = event.pos()
            event.accept()
            return
        if (event.button() == Qt.LeftButton
                and self._ctrl_select
                and self._mode == 'brush'):
            self._press_pos = event.pos()
            self.brush_drag_started.emit()
            event.accept()
            return
        if (event.button() == Qt.LeftButton
                and self._ctrl_select
                and self._mode == 'rectangle'):
            self._press_pos = event.pos()
            self._rubber_band.setGeometry(
                QRect(self._press_pos, self._press_pos))
            self._rubber_band.show()
            self._rubber_band.raise_()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if (self._mode == 'brush'
                and self._ctrl_select
                and self._press_pos is not None
                and event.buttons() & Qt.LeftButton):
            self.brush_point_dragged.emit(event.x(), event.y())
            event.accept()
            return
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
        if (self._mode == 'measurement'
                and self._press_pos is not None
                and self._ctrl_select):
            moved = (event.pos() - self._press_pos).manhattanLength()
            self._press_pos = None
            self._ctrl_select = False
            if moved < 4:
                self.measurement_point_clicked.emit(event.x(), event.y())
            event.accept()
            return
        if (self._mode == 'brush'
                and self._press_pos is not None
                and self._ctrl_select
                and event.button() == Qt.LeftButton):
            moved = (event.pos() - self._press_pos).manhattanLength()
            if moved >= 4:
                self.brush_point_dragged.emit(event.x(), event.y())
            self._press_pos = None
            self._ctrl_select = False
            self.brush_drag_finished.emit()
            event.accept()
            return
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
        self._ctrl_select = False
        super().mouseReleaseEvent(event)


class CloudEditor(q3d.Viewer):
    """Viewer shell: GUI state, display updates, and signal wiring only."""

    load_requested = Signal(object)
    load_file_requested = Signal(str, bool)
    save_requested = Signal(str)
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
        main_layout.setSpacing(0)
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
        self.mdi.addSubWindow(self.brush_window)
        self.mdi.addSubWindow(self.rectangle_window)
        self.mdi.addSubWindow(self.measurement_window)
        for window in (
                self.brush_window, self.rectangle_window, self.measurement_window):
            window.setWindowFlag(Qt.WindowSystemMenuHint, False)
            window.setWindowFlag(Qt.WindowMinimizeButtonHint, False)
            window.setWindowFlag(Qt.WindowMaximizeButtonHint, False)
        self.brush_window.hide()
        self.rectangle_window.hide()
        self.measurement_window.hide()

        self.operations = CloudEditorOperations()
        self.selector = PointsSelector(self.glwidget)
        self.brush_tool = BrushDeleteTool(self.glwidget, self.selector)
        self.rectangle_tool = RectangleDeleteTool(
            self.glwidget, self.selector)
        self.measurement_tool = MeasurementTool(self.glwidget)

        self.brush_tool.delete_requested.connect(
            self.operations.mark_deleted_indices)
        self.rectangle_tool.delete_requested.connect(
            self.operations.mark_deleted_indices)
        self.measurement_tool.changed.connect(self.display_measurement)

        self.io.cloud_loaded.connect(self._merge_cloud)
        self.load_requested.connect(self.io.load_files)
        self.load_file_requested.connect(self.io.load_file)
        self.save_requested.connect(self._save_cloud)
        self.apply_requested.connect(self.operations.apply_operations)
        self.undo_requested.connect(self.operations.undo)
        self.cancel_requested.connect(self.operations.cancel_pending)
        self.measurement_window.undo_requested.connect(
            self.measurement_tool.undo)
        self.measurement_window.closed.connect(self.close_delete_tool)

        self.operations.cloud_display_changed.connect(self.display_cloud)
        self.operations.marked_display_changed.connect(self.display_marked)
        self.io.progress.connect(self.file_loading_progress)
        self.io.loading_finished.connect(self.file_loading_finished)
        self.io.operation_error.connect(self.show_operation_error)
        self.io.save_finished.connect(self.show_save_finished)
        self.glwidget.sort_requested.connect(self.force_sort)

    def add_control_panel(self, main_layout):
        toolbar = QWidget()
        toolbar_layout = QHBoxLayout(toolbar)
        toolbar_layout.setContentsMargins(6, 0, 6, 0)
        toolbar_layout.setSpacing(1)

        self.btn_load = make_icon_button(
            toolbar, 'folder_open', 'Load point cloud files')
        self.btn_load.clicked.connect(self.load_clouds)
        toolbar_layout.addWidget(self.btn_load)

        self.btn_save = make_icon_button(
            toolbar, 'save', 'Save the edited point cloud')
        self.btn_save.clicked.connect(self.save_as)
        toolbar_layout.addWidget(self.btn_save)

        separator = QFrame(toolbar)
        separator.setFrameShape(QFrame.VLine)
        separator.setFrameShadow(QFrame.Sunken)
        toolbar_layout.addWidget(separator)

        self.btn_measurement = make_icon_button(
            toolbar, 'straighten',
            'Ctrl+click to measure')
        self.btn_measurement.setCheckable(True)
        self.btn_measurement.toggled.connect(self.set_measurement_mode)
        toolbar_layout.addWidget(self.btn_measurement)

        self.btn_brush_delete = make_icon_button(
            toolbar, 'brush',
            'Ctrl+drag to delete points')
        self.btn_brush_delete.setCheckable(True)
        self.btn_brush_delete.toggled.connect(self.set_brush_mode)
        toolbar_layout.addWidget(self.btn_brush_delete)

        self.btn_rectangle_delete = make_icon_button(
            toolbar, 'select',
            'Ctrl+drag to delete points inside a rectangle')
        self.btn_rectangle_delete.setCheckable(True)
        self.btn_rectangle_delete.toggled.connect(self.set_rectangle_mode)
        toolbar_layout.addWidget(self.btn_rectangle_delete)

        toolbar_layout.addStretch()
        main_layout.addWidget(toolbar)

        self.brush_window = BrushDeleteWindow(self)
        self.brush_window.radius_changed.connect(self.set_radius)
        self.brush_window.apply_requested.connect(self.apply_requested)
        self.brush_window.undo_requested.connect(self.undo_requested)
        self.brush_window.cancel_requested.connect(self.cancel_requested)
        self.brush_window.closed.connect(self.close_delete_tool)

        self.rectangle_window = RectangleDeleteWindow(self)
        self.rectangle_window.apply_requested.connect(self.apply_requested)
        self.rectangle_window.undo_requested.connect(self.undo_requested)
        self.rectangle_window.cancel_requested.connect(self.cancel_requested)
        self.rectangle_window.closed.connect(self.close_delete_tool)

        self.measurement_window = MeasurementWindow(self)

    def _clear_tool_buttons(self):
        for button in (
                self.btn_brush_delete,
                self.btn_rectangle_delete,
                self.btn_measurement):
            button.blockSignals(True)
            button.setChecked(False)
            button.blockSignals(False)

    def _activate_tool(self, mode):
        self._clear_tool_buttons()
        buttons = {
            'brush': self.btn_brush_delete,
            'rectangle': self.btn_rectangle_delete,
            'measurement': self.btn_measurement,
        }
        button = buttons.get(mode)
        if button is not None:
            button.blockSignals(True)
            button.setChecked(True)
            button.blockSignals(False)
        self.active_mode = mode
        self.brush_tool.set_active(mode == 'brush')
        self.rectangle_tool.set_active(mode == 'rectangle')
        self.measurement_tool.set_active(mode == 'measurement')
        self.glwidget.set_mode(mode)

    def _hide_tool_dialogs(self):
        for window in (
            self.brush_window, self.rectangle_window,
            self.measurement_window):
            window.hide()

    def _show_delete_tool(self, mode):
        self._activate_tool(mode)
        self._hide_tool_dialogs()
        window = (self.brush_window
                  if mode == 'brush' else self.rectangle_window)
        window.show()
        window.raise_()
        window.activateWindow()

    def close_delete_tool(self, _result=None):
        self._hide_tool_dialogs()
        self._clear_tool_buttons()
        self.brush_tool.set_active(False)
        self.rectangle_tool.set_active(False)
        self.measurement_tool.set_active(False)
        self.active_mode = None
        self.glwidget.set_mode(None)

    def set_brush_mode(self, enabled):
        if enabled:
            self._show_delete_tool('brush')
        elif self.active_mode == 'brush':
            self.close_delete_tool()

    def set_rectangle_mode(self, enabled):
        if enabled:
            self._show_delete_tool('rectangle')
        elif self.active_mode == 'rectangle':
            self.close_delete_tool()

    def _show_measurement_tool(self):
        self._activate_tool('measurement')
        self._hide_tool_dialogs()
        self.measurement_window.show()
        self.measurement_window.raise_()
        self.measurement_window.activateWindow()

    def set_measurement_mode(self, enabled):
        if enabled:
            self._show_measurement_tool()
        elif self.active_mode == 'measurement':
            self.close_delete_tool()

    def set_radius(self, value):
        self.brush_tool.set_radius(value)

    def _merge_cloud(self, cloud, append):
        merged_cloud = self.operations.set_cloud(cloud, append=append)
        self.selector.set_points(merged_cloud['xyz'])
        self.set_cloud_color_mode(merged_cloud)
        self.set_camera_center(get_cloud_center(cloud))

    def _save_cloud(self, file):
        self.io.save(self.operations.edited_cloud(), file)

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

    def display_measurement(self, markers, point_count, distance):
        measurement_item = self['measurement']
        if measurement_item is not None:
            measurement_item.set_data(data=markers, append=False)
        self.measurement_window.set_measurement(point_count, distance)

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


def print_version():
    print(f'Cloud Editor ({get_version()})')


def main():
    print_version()
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', help='the cloud file path')
    args = parser.parse_args()
    app = q3d.QApplication(['Cloud Editor'])
    viewer = CloudEditor(name='Cloud Editor')

    cloud_item = q3d.CloudSortItem(size=1, alpha=1, point_type='SPHERE')
    marked_item = CloudItem(
        size=5, alpha=1.0, color_mode='FLAT', color='#ff1493', point_type='SPHERE')
    marked_item.disable_setting()
    axis_item = q3d.AxisItem(size=0.5, width=5)
    axis_item.disable_setting()
    grid_item = q3d.GridItem(size=1000, spacing=20)
    measurement_item = q3d.Text3DItem()
    measurement_item.disable_setting()

    viewer.add_items({
        'cloud': cloud_item,
        'marked': marked_item,
        'grid': grid_item,
        'axis': axis_item,
        'measurement': measurement_item,
    })
    if args.path:
        viewer.open_cloud_file(args.path)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
