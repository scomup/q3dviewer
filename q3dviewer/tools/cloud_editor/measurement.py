"""Interactive point-to-point measurement for the cloud editor."""

import numpy as np
from q3dviewer.Qt.QtCore import QObject, Qt, Signal
from q3dviewer.Qt.QtWidgets import QLabel
from q3dviewer.tools.cloud_editor.tool_window import ToolWindow


class MeasurementTool(QObject):
    """Own measurement points and publish marker data plus total distance."""

    changed = Signal(object, int, float)

    def __init__(self, view_handler):
        super().__init__()
        self.view_handler = view_handler
        self.active = False
        self.points = []
        view_handler.measurement_point_clicked.connect(self._add_screen_point)
        view_handler.measurement_undo_requested.connect(self.undo)

    def set_active(self, active):
        self.active = bool(active)
        if not self.active:
            self.clear()

    def _add_screen_point(self, x, y):
        if not self.active:
            return
        point = self.view_handler.get_point(x, y)
        if point is not None:
            self.add_point(point)

    def add_point(self, point):
        if not self.active:
            return
        self.points.append(np.asarray(point, dtype=np.float32))
        self._publish()

    def undo(self):
        if self.active and self.points:
            self.points.pop()
            self._publish()

    def clear(self):
        if self.points:
            self.points = []
            self._publish()
        else:
            self.changed.emit([], 0, 0.0)

    def _publish(self):
        markers = [
            {
                'text': '',
                'position': point,
                'color': (0.0, 1.0, 0.0, 1.0),
                'font_size': 16,
                'point_size': 5.0,
                'line_width': 2.0,
            }
            for point in self.points
        ]
        distance = sum(
            np.linalg.norm(self.points[index] - self.points[index - 1])
            for index in range(1, len(self.points)))
        self.changed.emit(markers, len(self.points), float(distance))


class MeasurementWindow(ToolWindow):
    """Controls and readout for the measurement tool."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Measurement')
        self.setMinimumWidth(360)
        self.point_count = QLabel('Points: 0')
        self.total_distance = QLabel('Total distance: <b>0.00</b> m')
        self.total_distance.setTextFormat(Qt.TextFormat.RichText)
        self.content_layout.addWidget(self.point_count)
        self.content_layout.addWidget(self.total_distance)
        self.btn_apply.hide()
        self.set_hint('Ctrl+click to add points')
        self.resize(self.width(), self.sizeHint().height())

    def set_measurement(self, point_count, distance):
        self.point_count.setText(f'Points: {point_count}')
        self.total_distance.setText(
            f'Total distance: <b>{distance:.2f}</b> m')
