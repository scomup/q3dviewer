#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

"""Compare two point clouds in vertically stacked OpenGL views."""

import os
import signal

import numpy as np

import q3dviewer as q3d
from q3dviewer import GLWidget
from q3dviewer.glwidget import SettingWindow
from q3dviewer.Qt import QtCore
from q3dviewer.Qt.QtCore import QThread, Signal, Qt
from q3dviewer.Qt.QtWidgets import (
    QApplication,
    QDialog,
    QLabel,
    QMainWindow,
    QOpenGLWidget,
    QSplitter,
    QVBoxLayout,
)
from q3dviewer.utils.helpers import get_version
from q3dviewer.utils.maths import calc_view_matrix


class ProgressWindow(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Loading")
        self.setModal(True)
        self.setMinimumWidth(400)
        self.label = QLabel(self)
        self.label.setAlignment(Qt.AlignCenter)
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

    def update_progress(self, current, total, file_name):
        self.label.setText(f"[{current}/{total}] loading: {file_name}")


class FileLoaderThread(QThread):
    progress = Signal(int, int, str)
    finished = Signal(object)

    def __init__(self, cloud_item, files):
        super().__init__()
        self.cloud_item = cloud_item
        self.files = files

    def run(self):
        center = None
        total = len(self.files)
        for index, url in enumerate(self.files):
            file_path = url.toLocalFile()
            self.progress.emit(index + 1, total, os.path.basename(file_path))
            cloud = self.cloud_item.load(file_path, append=index > 0)
            if cloud is not None:
                center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
        self.finished.emit(center)


class GLWidgetPair(GLWidget):
    """A GLWidget that copies view and display settings to its pair."""

    CLOUD_SETTINGS = (
        'alpha', 'size', 'point_type', 'color_mode', 'color', 'flat_rgb',
        'vmin', 'vmax', 'T', 'use_depth_sorting',
    )
    GRID_SETTINGS = ('size', 'spacing', 'rgba', 'offset')

    def __init__(self, setting_window):
        self.other = None
        self._loader = None
        self.cloud_item = None
        self.grid_item = None
        self.marker_item = None
        self.text_item = None
        super().__init__()
        self.setting_window = setting_window
        self.setAcceptDrops(True)

    def set_other(self, other):
        self.other = other

    def _update_pair(self):
        self.update()
        if self.other is not None:
            self.other.update()

    def mouseMoveEvent(self, event):
        super().mouseMoveEvent(event)
        self._update_pair()

    def wheelEvent(self, event):
        super().wheelEvent(event)
        self._update_pair()

    def mouseDoubleClickEvent(self, event):
        super().mouseDoubleClickEvent(event)
        self._update_pair()

    def update(self):
        self.follow_odom()
        self.update_cam_pose_by_key()
        view_updated = self.need_recalc_view or self.view_changed

        if self.need_recalc_view:
            self.view_matrix = calc_view_matrix(self.center, self.dist, self.euler)
            self.view_changed = True
            self.need_recalc_view = False

        QOpenGLWidget.update(self)
        self.view_changed = False
        for item in self.items:
            item.clear_changed()

        if self.other is None:
            return view_updated

        if view_updated:
            self.other.center = self.center.copy()
            self.other.dist = self.dist
            self.other.euler = self.euler.copy()
            self.other.view_matrix = calc_view_matrix(
                self.other.center, self.other.dist, self.other.euler)
            self.other.need_recalc_view = False
            self.other.view_changed = True

        if self.cloud_item.is_changed():
            self._copy_settings(
                self.cloud_item, self.other.cloud_item, self.CLOUD_SETTINGS)
        if self.grid_item.is_changed():
            self._copy_settings(
                self.grid_item, self.other.grid_item, self.GRID_SETTINGS)
        return view_updated

    @staticmethod
    def _copy_settings(source, target, names):
        changed = False
        for name in names:
            if not hasattr(source, name):
                continue
            value = getattr(source, name)
            target_value = getattr(target, name, None)
            same = (np.array_equal(value, target_value)
                    if isinstance(value, np.ndarray)
                    else value == target_value)
            if not same:
                setattr(target, name,
                        value.copy() if isinstance(value, np.ndarray) else value)
                changed = True

        if not changed:
            return
        if hasattr(target, 'need_update_setting'):
            target.need_update_setting = True
        if hasattr(target, 'need_update_grid'):
            target.need_update_grid = True
        if hasattr(target, 'last_depth_coeffs'):
            target.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])
        target.notify_changed()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and event.modifiers() & Qt.ControlModifier:
            point = self.get_point(event.x(), event.y())
            if point is not None:
                self._add_measurement_point(point)
        elif event.button() == Qt.RightButton and event.modifiers() & Qt.ControlModifier:
            if self.selected_points:
                self.selected_points.pop()
                self._update_measurement()
        super().mousePressEvent(event)

    def _add_measurement_point(self, point):
        self.selected_points.append(point)
        self._update_measurement()

    def _update_measurement(self):
        marks = [
            {
                'text': '',
                'position': point,
                'color': (0.0, 1.0, 0.0, 1.0),
                'font_size': 16,
                'point_size': 5.0,
                'line_width': 1.0,
            }
            for point in self.selected_points
        ]
        self.marker_item.set_data(data=marks, append=False)
        if len(self.selected_points) < 2:
            self.text_item.set_data(text='')
            return
        distance = sum(
            np.linalg.norm(
                np.asarray(self.selected_points[index])
                - np.asarray(self.selected_points[index - 1])
            )
            for index in range(1, len(self.selected_points))
        )
        self.text_item.set_data(text=f'Total Distance: {distance:.2f} m')

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event):
        progress = ProgressWindow(self.window())
        progress.show()
        self._loader = FileLoaderThread(
            self.cloud_item, event.mimeData().urls())
        self._loader.progress.connect(progress.update_progress)
        self._loader.finished.connect(
            lambda center, dialog=progress: self._load_finished(center, dialog))
        self._loader.start()

    def _load_finished(self, center, progress):
        progress.close()
        if center is not None:
            self.set_cam_position(center=center)


class CompareViewer(QMainWindow):
    def __init__(self, name='Compare Viewer', win_size=(1920, 1080)):
        super().__init__()
        signal.signal(signal.SIGINT, lambda *_: QApplication.quit())
        self.setWindowTitle(name)
        self.setGeometry(0, 0, win_size[0], win_size[1])

        self.setting_window = SettingWindow()
        self.top_gl = GLWidgetPair(self.setting_window)
        self.bottom_gl = GLWidgetPair(self.setting_window)
        self.top_gl.set_other(self.bottom_gl)
        self.bottom_gl.set_other(self.top_gl)

        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(self.top_gl)
        splitter.addWidget(self.bottom_gl)
        splitter.setSizes([win_size[1] // 2, win_size[1] // 2])
        self.setCentralWidget(splitter)

        self._setup_pane(self.top_gl)
        self._setup_pane(self.bottom_gl)
        self._add_shared_settings()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self._tick)
        self.timer.start()

    def _setup_pane(self, glwidget):
        cloud_item = q3d.CloudSortItem(size=1, alpha=0.1)
        grid_item = q3d.GridItem(size=1000, spacing=20)
        axis_item = q3d.AxisItem(size=0.5, width=5)
        marker_item = q3d.Text3DItem()
        text_item = q3d.Text2DItem(pos=(20, 40), text='', color='lime', size=16)

        for item in (cloud_item, grid_item, axis_item, marker_item, text_item):
            item.disable_setting()

        glwidget.add_item_with_name('cloud', cloud_item)
        glwidget.add_item_with_name('grid', grid_item)
        glwidget.add_item_with_name('axis', axis_item)
        glwidget.add_item_with_name('marker', marker_item)
        glwidget.add_item_with_name('text', text_item)

        glwidget.cloud_item = cloud_item
        glwidget.grid_item = grid_item
        glwidget.marker_item = marker_item
        glwidget.text_item = text_item
        glwidget.selected_points = []

    def _add_shared_settings(self):
        self.setting_window.add_setting('Cloud', self.top_gl.cloud_item)
        self.setting_window.add_setting('Grid', self.top_gl.grid_item)
        self.setting_window.add_setting('View', self.top_gl)

    def _tick(self):
        self.top_gl.update()
        self.bottom_gl.update()

    def closeEvent(self, event):
        event.accept()
        QApplication.quit()


def print_help():
    print(f"Compare Viewer ({get_version()})")
    print("Drag point-cloud files into either pane.")
    print("The two panes share view, grid, and cloud settings.")
    print("Use Ctrl+click to measure distance and M to open settings.")


def main():
    print_help()
    import argparse

    parser = argparse.ArgumentParser(
        description='Compare two point clouds in stacked views.')
    parser.add_argument('--top', help='Point cloud file for the top pane')
    parser.add_argument('--bottom', help='Point cloud file for the bottom pane')
    args = parser.parse_args()

    app = q3d.QApplication(['Compare Viewer'])
    viewer = CompareViewer()

    def load_pane(glwidget, path):
        if not path or not os.path.isfile(path):
            return
        cloud = glwidget.cloud_item.load(path, append=False)
        if cloud is not None:
            center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
            glwidget.set_cam_position(center=center)

    load_pane(viewer.top_gl, args.top)
    load_pane(viewer.bottom_gl, args.bottom)
    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
