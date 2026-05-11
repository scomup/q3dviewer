#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.

Compare Viewer - side-by-side (top/bottom) point cloud comparison.
Both panes share camera, keyboard, and settings (M key) state.
Cloud visualisation settings are unified: one panel controls both panes.
"""

import os
import signal
import numpy as np

import q3dviewer as q3d
from q3dviewer.Qt import QtCore
from q3dviewer.Qt.QtWidgets import (
    QMainWindow, QApplication, QSplitter,
    QWidget, QVBoxLayout, QDialog, QLabel,
)
from q3dviewer.Qt.QtCore import QThread, Signal, Qt
from q3dviewer import GLWidget
from q3dviewer.glwidget import SettingWindow
from q3dviewer.utils import text_to_rgba
from q3dviewer.utils.helpers import get_version


# ---------------------------------------------------------------------------
# Per-pane file loader (background thread with progress dialog)
# ---------------------------------------------------------------------------

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
        self.label.setText(f"[{current}/{total}] loading: {file_name}")


class FileLoaderThread(QThread):
    progress = Signal(int, int, str)
    finished = Signal(object)   # emits numpy center on done

    def __init__(self, cloud_item, files):
        super().__init__()
        self.cloud_item = cloud_item
        self.files = files

    def run(self):
        total = len(self.files)
        center = None
        for i, url in enumerate(self.files):
            file_path = url.toLocalFile()
            file_name = os.path.basename(file_path)
            self.progress.emit(i + 1, total, file_name)
            cloud = self.cloud_item.load(file_path, append=(i > 0))
            center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
        self.finished.emit(center)


# ---------------------------------------------------------------------------
# LinkedCloudSetting — one settings panel that drives two CloudSortItems
# ---------------------------------------------------------------------------

def _patch_cloud_sync(primary, secondary):
    """
    Monkey-patch *primary*'s setter / event handler methods so every UI-driven
    property change is mirrored to *secondary* as well.

    Only property-level attributes are written to secondary; its Qt widgets are
    never touched (it has none), so there are no "widget not initialised" errors.
    """
    from q3dviewer.custom_items.cloud_item import CloudItem

    # --- set_alpha --------------------------------------------------------
    _o = primary.set_alpha
    def _set_alpha(alpha, _orig=_o, _s=secondary):
        _orig(alpha)
        _s.set_alpha(alpha)
    primary.set_alpha = _set_alpha

    # --- set_size ---------------------------------------------------------
    _o = primary.set_size
    def _set_size(size, _orig=_o, _s=secondary):
        _orig(size)
        _s.set_size(size)
    primary.set_size = _set_size

    # --- _on_color_mode (has UI side-effects on primary only) -------------
    _o = primary._on_color_mode
    def _on_color_mode(index, _orig=_o, _s=secondary):
        _orig(index)
        _s.color_mode = index
        _s.need_update_setting = True
    primary._on_color_mode = _on_color_mode

    # --- _on_point_type_selection -----------------------------------------
    _o = primary._on_point_type_selection
    def _on_ptype(index, _orig=_o, _s=secondary):
        _orig(index)
        _s.point_type = list(CloudItem.POINT_TYPE_TABLE.keys())[index]
        _s.need_update_setting = True
    primary._on_point_type_selection = _on_ptype

    # --- _on_color --------------------------------------------------------
    _o = primary._on_color
    def _on_color(color, _orig=_o, _s=secondary):
        _orig(color)
        try:
            _s.flat_rgb = text_to_rgba(color, flat=True)
            _s.need_update_setting = True
        except ValueError:
            pass
    primary._on_color = _on_color

    # --- _on_range --------------------------------------------------------
    _o = primary._on_range
    def _on_range(lower, upper, _orig=_o, _s=secondary):
        _orig(lower, upper)
        _s.vmin = lower
        _s.vmax = upper
        _s.need_update_setting = True
    primary._on_range = _on_range

    # --- _on_depth_sorting (CloudSortItem only) ---------------------------
    if hasattr(primary, '_on_depth_sorting'):
        _o = primary._on_depth_sorting
        def _on_depth_sort(state, _orig=_o, _s=secondary):
            _orig(state)
            new_state = (state != 0)
            if new_state and not _s.cuda_sorter:
                return
            _s.use_depth_sorting = new_state
            _s.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])
        primary._on_depth_sorting = _on_depth_sort


class LinkedCloudSetting:
    """
    A lightweight proxy registered once in the shared SettingWindow.
    It shows *primary*'s setting widgets; every change is automatically
    forwarded to *secondary* via the patches installed by _patch_cloud_sync.
    """
    _disable_setting = False          # required so SettingWindow can register it

    def __init__(self, primary, secondary):
        self.primary = primary
        self.secondary = secondary
        # Install sync patches before add_setting is called so that the Qt
        # signals created inside add_setting connect to the already-patched methods.
        _patch_cloud_sync(primary, secondary)

    def add_setting(self, layout):
        self.primary.add_setting(layout)


# ---------------------------------------------------------------------------
# CompareGLWidget — GLWidget with camera-sync and per-pane measurement
# ---------------------------------------------------------------------------

class CompareGLWidget(GLWidget):
    """
    Extends GLWidget with:
    - Shared SettingWindow across panes (M key shows the same window).
    - Camera state sync to peer panes after every mouse/wheel/key movement.
    - Ctrl+click point measurement (same as CustomGLWidget in cloud_viewer).
    - Drag-and-drop file loading into this pane's cloud_item.
    """

    def __init__(self, pane_label: str, shared_setting_window: SettingWindow):
        super().__init__()
        # Replace the per-instance SettingWindow with the shared one.
        # super().__init__() creates self.setting_window = SettingWindow(),
        # but no items have been added yet, so we can safely replace it.
        self.setting_window = shared_setting_window

        self.pane_label = pane_label
        self.selected_points = []
        self.peers: list['CompareGLWidget'] = []
        self._syncing = False

        # Set after _setup_pane() calls add_item_with_name()
        self.cloud_item = None
        self.marker_item = None
        self.text_item = None

        self.setAcceptDrops(True)

    # ------------------------------------------------------------------
    # Camera synchronisation
    # ------------------------------------------------------------------

    def _sync_camera_to_peers(self):
        """Push current camera state to all registered peers."""
        if self._syncing:
            return
        for peer in self.peers:
            peer._syncing = True
            peer.set_center(self.center.copy())
            peer.set_euler(self.euler.copy())
            peer.set_dist(self.dist)
            peer._syncing = False

    def mouseMoveEvent(self, ev):
        super().mouseMoveEvent(ev)
        self._sync_camera_to_peers()

    def wheelEvent(self, ev):
        super().wheelEvent(ev)
        self._sync_camera_to_peers()

    def mouseDoubleClickEvent(self, ev):
        super().mouseDoubleClickEvent(ev)
        self._sync_camera_to_peers()

    def update(self):
        had_keys = bool(self.active_keys)
        super().update()
        # update_movement() runs inside super().update() when keys are active
        if had_keys and not self._syncing:
            self._sync_camera_to_peers()

    # ------------------------------------------------------------------
    # Measurement (Ctrl + click, same as CustomGLWidget)
    # ------------------------------------------------------------------

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and event.modifiers() & Qt.ControlModifier:
            p = self.get_point(event.x(), event.y())
            if p is not None:
                self.selected_points.append(p)
                self._update_marker()
        elif event.button() == Qt.RightButton and event.modifiers() & Qt.ControlModifier:
            if self.selected_points:
                self.selected_points.pop()
                self._update_marker()
        super().mouseReleaseEvent(event)

    def _update_marker(self):
        marks = [
            {
                'text': '',
                'position': p,
                'color': (0.0, 1.0, 0.0, 1.0),
                'font_size': 16,
                'point_size': 5.0,
                'line_width': 1.0,
            }
            for p in self.selected_points
        ]
        if self.marker_item:
            self.marker_item.set_data(data=marks, append=False)

        if len(self.selected_points) >= 2:
            total = sum(
                np.linalg.norm(np.array(self.selected_points[i]) -
                               np.array(self.selected_points[i - 1]))
                for i in range(1, len(self.selected_points))
            )
            if self.text_item:
                self.text_item.set_data(
                    text=f'Total Distance: {total:.2f} m')
        else:
            if self.text_item:
                self.text_item.set_data(text='')

    # ------------------------------------------------------------------
    # Drag-and-drop into this pane
    # ------------------------------------------------------------------

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        if self.cloud_item is None:
            return
        # Show progress dialog parented to the top-level window
        top = self.window()
        progress = ProgressWindow(top)
        progress.show()

        files = event.mimeData().urls()
        self._loader = FileLoaderThread(self.cloud_item, files)
        self._loader.progress.connect(progress.update_progress)
        self._loader.finished.connect(
            lambda center, p=progress: self._on_load_finished(center, p))
        self._loader.start()

    def _on_load_finished(self, center, progress_window):
        progress_window.close()
        if center is not None:
            self.set_cam_position(center=center)
            self._sync_camera_to_peers()


# ---------------------------------------------------------------------------
# CompareViewer — QMainWindow with top/bottom split
# ---------------------------------------------------------------------------

class CompareViewer(QMainWindow):
    """
    Main window split vertically into two panes.
    Each pane is a CompareGLWidget with its own CloudSortItem.
    Camera, keyboard, and settings (M key) are shared/synced between panes.
    """

    def __init__(self, name='Compare Viewer', win_size=(1920, 1080)):
        super().__init__()
        signal.signal(signal.SIGINT, lambda s, f: QApplication.quit())
        self.setWindowTitle(name)
        self.setGeometry(0, 0, win_size[0], win_size[1])

        # One shared SettingWindow for both panes
        self.shared_setting_window = SettingWindow()

        # Build UI
        splitter = QSplitter(Qt.Vertical)
        self.top_gl = CompareGLWidget('Top', self.shared_setting_window)
        self.bottom_gl = CompareGLWidget('Bottom', self.shared_setting_window)

        splitter.addWidget(self.top_gl)
        splitter.addWidget(self.bottom_gl)
        splitter.setSizes([win_size[1] // 2, win_size[1] // 2])
        self.setCentralWidget(splitter)

        # Wire up camera-sync peers
        self.top_gl.peers = [self.bottom_gl]
        self.bottom_gl.peers = [self.top_gl]

        # Populate each pane with items
        self._setup_pane(self.top_gl, 'Top')
        self._setup_pane(self.bottom_gl, 'Bottom')

        # One unified cloud settings panel — changes propagate to both panes.
        linked_cloud = LinkedCloudSetting(
            self.top_gl.cloud_item, self.bottom_gl.cloud_item)
        self.shared_setting_window.add_setting('cloud', linked_cloud)

        # Per-pane viewport settings (background colour, show-centre, follow).
        self.shared_setting_window.add_setting('Top Viewport', self.top_gl)
        self.shared_setting_window.add_setting(
            'Bottom Viewport', self.bottom_gl)

        # Periodic repaint timer
        timer = QtCore.QTimer(self)
        timer.setInterval(20)
        timer.timeout.connect(self._tick)
        timer.start()

    # ------------------------------------------------------------------
    # Pane initialisation
    # ------------------------------------------------------------------

    def _setup_pane(self, glwidget: CompareGLWidget, label: str):
        """Add standard items to a pane and store convenient references."""
        cloud_item = q3d.CloudSortItem(size=1, alpha=0.1)
        # Disable auto-registration so only the shared LinkedCloudSetting
        # appears in the settings window (not separate Top/Bottom entries).
        cloud_item.disable_setting()

        axis_item = q3d.AxisItem(size=0.5, width=5)
        axis_item.disable_setting()
        grid_item = q3d.GridItem(size=1000, spacing=20)
        marker_item = q3d.Text3DItem()
        text_item = q3d.Text2DItem(pos=(20, 40), text="", color='lime', size=16)
        text_item.disable_setting()

        glwidget.add_item_with_name(f'{label}_cloud', cloud_item)
        glwidget.add_item_with_name(f'{label}_grid', grid_item)
        glwidget.add_item_with_name(f'{label}_axis', axis_item)
        glwidget.add_item_with_name(f'{label}_marker', marker_item)
        glwidget.add_item_with_name(f'{label}_text', text_item)

        glwidget.cloud_item = cloud_item
        glwidget.marker_item = marker_item
        glwidget.text_item = text_item

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _tick(self):
        self.top_gl.update()
        self.bottom_gl.update()

    def closeEvent(self, event):
        event.accept()
        QApplication.quit()


# ---------------------------------------------------------------------------
# CLI entry-point
# ---------------------------------------------------------------------------

def print_help():
    try:
        from rich.console import Console
        from rich.table import Table

        console = Console()
        table = Table(show_header=False, box=None, padding=(0, 2))
        table.add_column(style="bold cyan", width=32)
        table.add_column(style="white")

        table.add_row("📁 Load Files",
                      "Drag and drop into the TOP or BOTTOM pane")
        table.add_row("", "[dim]• Point clouds: .pcd, .ply, .las, .e57[/dim]")
        table.add_row("", "")
        table.add_row("📏 Measure Distance", "Interactive point measurement")
        table.add_row("", "[dim]• Ctrl + Left Click : Add point[/dim]")
        table.add_row("", "[dim]• Ctrl + Right Click: Remove last point[/dim]")
        table.add_row("", "")
        table.add_row("🎥 Camera (synced)", "Controls apply to BOTH panes")
        table.add_row("", "[dim]• Double Click  : Set center[/dim]")
        table.add_row("", "[dim]• Right Drag    : Rotate[/dim]")
        table.add_row("", "[dim]• Left Drag     : Pan[/dim]")
        table.add_row("", "[dim]• Wheel         : Zoom[/dim]")
        table.add_row("", "[dim]• WASD / Arrows : Move[/dim]")
        table.add_row("", "")
        table.add_row("⚙️  Settings",
                      "Press [bold green]'M'[/bold green] — shared between panes")

        console.print()
        console.print(
            f"[bold magenta]🔍 Compare Viewer ({get_version()}) Help"
            f"[/bold magenta]\n")
        console.print(table)
        console.print()
    except ImportError:
        print(f"Compare Viewer ({get_version()})")
        print("Drag & drop point clouds into the top or bottom pane.")
        print("Controls are synced between both panes.")
        print("Press M to open the shared settings window.")
        print()


def main():
    print_help()
    import argparse
    parser = argparse.ArgumentParser(
        description="Compare two point clouds side-by-side (top/bottom).")
    parser.add_argument("--top", help="Point cloud file for the top pane")
    parser.add_argument("--bottom",
                        help="Point cloud file for the bottom pane")
    args = parser.parse_args()

    app = q3d.QApplication(['Compare Viewer'])
    viewer = CompareViewer(name='Compare Viewer')

    def load_pane(glwidget, path):
        if not path or not os.path.isfile(path):
            return
        cloud = glwidget.cloud_item.load(path, append=False)
        center = np.nanmean(cloud['xyz'].astype(np.float64), axis=0)
        glwidget.set_cam_position(center=center)
        glwidget._sync_camera_to_peers()

    if args.top:
        load_pane(viewer.top_gl, args.top)
    if args.bottom:
        load_pane(viewer.bottom_gl, args.bottom)

    viewer.show()
    app.exec()


if __name__ == '__main__':
    main()
