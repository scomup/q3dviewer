"""Brush-based point deletion tool."""

import numpy as np
from q3dviewer.Qt.QtCore import QObject, Signal
from q3dviewer.Qt.QtWidgets import QDoubleSpinBox, QLabel
from q3dviewer.tools.cloud_editor.points_selector import PointsSelector
from q3dviewer.tools.cloud_editor.tool_window import ToolWindow


class BrushDeleteTool(QObject):
    """Convert brush drags into point-index deletion requests."""

    delete_requested = Signal(object)

    def __init__(self, view_handler, selector):
        super().__init__()
        self.view_handler = view_handler
        self.selector = selector
        self.active = False
        self.radius = 0.5
        self._last_point = None

        view_handler.brush_drag_started.connect(self._on_drag_started)
        view_handler.brush_point_dragged.connect(self._on_point_dragged)
        view_handler.brush_drag_finished.connect(self._on_drag_finished)

    def set_active(self, active):
        self.active = bool(active)
        if not self.active:
            self._last_point = None

    def set_radius(self, radius):
        self.radius = float(radius)

    def _on_drag_started(self):
        if self.active:
            self._last_point = None

    def _on_point_dragged(self, x, y):
        if not self.active:
            return

        point = self.view_handler.get_point(x, y)
        if point is None:
            return
        point = np.asarray(point, dtype=np.float32)
        if (self._last_point is not None
                and np.linalg.norm(point - self._last_point)
                <= self.radius * 0.5):
            return

        data = (point, self.radius)
        indices = self.selector.select(PointsSelector.RADIUS_MODE, data)
        self.delete_requested.emit(indices)
        self._last_point = point

    def _on_drag_finished(self):
        if self.active:
            self._last_point = None


class BrushDeleteWindow(ToolWindow):
    """Controls for the brush deletion tool."""

    radius_changed = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Brush Delete')
        self.content_layout.addWidget(QLabel('Brush size (m)'))

        self.radius_box = QDoubleSpinBox()
        self.radius_box.setDecimals(2)
        self.radius_box.setSingleStep(0.1)
        self.radius_box.setRange(0.01, 1000.0)
        self.radius_box.setValue(0.5)
        self.radius_box.valueChanged.connect(
            lambda value: self.radius_changed.emit(float(value)))
        self.content_layout.addWidget(self.radius_box)
        self.set_hint('Ctrl+drag to mark points')

    def set_radius(self, radius):
        self.radius_box.blockSignals(True)
        self.radius_box.setValue(radius)
        self.radius_box.blockSignals(False)
