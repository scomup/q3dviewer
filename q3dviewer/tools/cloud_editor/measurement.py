"""Interactive point-to-point measurement for the cloud editor."""

import numpy as np
from q3dviewer.Qt.QtCore import QObject, Signal


class MeasurementTool(QObject):
    """Own measurement points and publish marker data plus total distance."""

    changed = Signal(object, str)

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
            self.changed.emit([], '')

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
        label = f'Total distance: {distance:.2f} m' if self.points else ''
        self.changed.emit(markers, label)
