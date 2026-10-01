#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

"""Compare two point clouds in vertically stacked OpenGL views."""

import signal
from pathlib import Path

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

    def __init__(self, cloud_item, files):
        super().__init__()
        self.cloud_item = cloud_item
        self.files = files

    def run(self):
        total = len(self.files)
        for index, url in enumerate(self.files):
            file_path = Path(url.toLocalFile())
            self.progress.emit(index + 1, total, file_path.name)
            self.cloud_item.load(str(file_path), append=index > 0)

class GLWidgetPair(GLWidget):
    """A GLWidget that copies view and display settings to its pair."""

    CLOUD_SETTINGS = (
        ['set_alpha', 'alpha'],
        ['set_size', 'size'],
        ['set_point_type', 'point_type'],
        ['set_color_mode', 'color_mode'],
        ['set_flat_rgb', 'flat_rgb'],
        ['set_value_range', ('vmin', 'vmax')],
        ['set_transform', 'T'],
        ['set_depth_sorting', 'use_depth_sorting'],
    )
    GRID_SETTINGS = (
        ['set_size', 'size'],
        ['set_spacing', 'spacing'],
        ['set_color', 'rgba'],
        ['set_offset', 'offset'],
    )
    GL_SETTINGS = (
        ['set_bg_color', 'color_str'],
    )

    def __init__(self, setting_window):
        self.mode = None
        self._loader = None
        self.cloud_item = None
        self.grid_item = None
        self.need_force_update = False
        super().__init__()
        self.setting_window = setting_window
        self.setAcceptDrops(True)

    def set_other(self, other, mode_type):
        if mode_type not in {'master', 'slave'}:
            raise ValueError("mode_type must be 'master' or 'slave'")
        self.other = other
        self.mode = mode_type

    def _route_event(self, event_name, event):
        receiver = self.other if self.mode == 'slave' else super()
        return getattr(receiver, event_name)(event)

    def keyPressEvent(self, event):
        return self._route_event('keyPressEvent', event)

    def keyReleaseEvent(self, event):
        return self._route_event('keyReleaseEvent', event)

    def mouseMoveEvent(self, event):
        return self._route_event('mouseMoveEvent', event)

    def mouseReleaseEvent(self, event):
        return self._route_event('mouseReleaseEvent', event)

    def mouseDoubleClickEvent(self, event):
        return self._route_event('mouseDoubleClickEvent', event)

    def wheelEvent(self, event):
        return self._route_event('wheelEvent', event)

    def update(self):
        if self.mode == 'master':
            self.update_cam_pose_by_key()
            if self.need_recalc_view:
                self.view_matrix = calc_view_matrix(self.center, self.dist, self.euler)
                self.need_force_update = True
                self.need_recalc_view = False
                self.other.view_matrix = self.view_matrix.copy()
                self.other.need_force_update = True
            if self.cloud_item.is_changed():
                self._copy_settings(
                    self.cloud_item, self.other.cloud_item, self.CLOUD_SETTINGS)
            if self.grid_item.is_changed():
                self._copy_settings(
                    self.grid_item, self.other.grid_item, self.GRID_SETTINGS)
            if self.need_force_update:
                self._copy_settings(self, self.other, self.GL_SETTINGS)

        # only update if there are changes in the view or any items
        have_dirty_item = any(item.is_changed() for item in self.items)
        if have_dirty_item or self.need_force_update:
            QOpenGLWidget.update(self) # will call paintGL()
            self.need_force_update = False
            for item in self.items:
                item.clear_changed()

    @staticmethod
    def _copy_settings(source, target, settings):
        for func, names in settings:
            update = getattr(target, func)
            if isinstance(names, tuple):
                values = tuple(getattr(source, name) for name in names)
                update(*values)
            else:
                values = getattr(source, names)
                update(values)

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
            lambda dialog=progress: self._load_finished(dialog))
        self._loader.start()

    def _load_finished(self, progress):
        progress.close()


class CompareViewer(QMainWindow):
    def __init__(self, name='Compare Viewer', win_size=(1920, 1080)):
        super().__init__()
        signal.signal(signal.SIGINT, lambda *_: QApplication.quit())
        self.setWindowTitle(name)
        self.setGeometry(0, 0, win_size[0], win_size[1])

        self.setting_window = SettingWindow()
        self.top_gl = GLWidgetPair(self.setting_window)
        self.bottom_gl = GLWidgetPair(self.setting_window)
        setting_path = Path.home() / '.config' / 'q3dviewer' / 'compare_viewer' / 'setting.json'
        self.top_gl.set_setting_path(setting_path)
        self.top_gl.set_other(self.bottom_gl, 'master')
        self.bottom_gl.set_other(self.top_gl, 'slave')

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

        for item in (cloud_item, grid_item, axis_item):
            item.disable_setting()

        glwidget.add_item_with_name('cloud', cloud_item)
        glwidget.add_item_with_name('grid', grid_item)
        glwidget.add_item_with_name('axis', axis_item)

        glwidget.cloud_item = cloud_item
        glwidget.grid_item = grid_item

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
    print("Press M to open settings.")


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
        if not path or not Path(path).is_file():
            return
        glwidget.cloud_item.load(path, append=False)

    load_pane(viewer.top_gl, args.top)
    load_pane(viewer.bottom_gl, args.bottom)
    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
