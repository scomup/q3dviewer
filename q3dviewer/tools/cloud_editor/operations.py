"""Data and editing operations for the cloud editor."""

import numpy as np
from q3dviewer.Qt.QtCore import QObject, Signal
from q3dviewer.tools.cloud_viewer import get_cloud_center
from q3dviewer.tools.cloud_editor.points_selector import PointsSelector
from q3dviewer.tools.cloud_editor.measurement import MeasurementTool


# An operation is a dict with a mandatory 'type' key; all other keys are
# type-specific parameters. Handlers are kept here with the data they modify.
def op_mark_deleted_indices(operations, keep, operation):
    keep = keep.copy()
    keep[operation['indices']] = False
    return keep


OP_HANDLERS = {
    'mark_deleted_indices': op_mark_deleted_indices,
}


def make_op(op_type, **params):
    if op_type not in OP_HANDLERS:
        raise ValueError(f"Unknown operation type: {op_type}")
    return {'type': op_type, **params}


class CloudEditorOperations(QObject):
    """Own cloud data and editing actions, using the viewer as an input handler."""

    cloud_display_changed = Signal(object)
    marked_display_changed = Signal(object)
    cloud_loaded = Signal(object)
    camera_center_changed = Signal(object)
    save_requested = Signal(object, str)
    measurement_changed = Signal(object, str)

    def __init__(self, view_handler, io_handler):
        super().__init__()
        self.view_handler = view_handler
        self.io_handler = io_handler
        self.cloud = None
        self.tree = None
        self.operations = []
        self.applied_keep = None
        self.keep = None
        self.mode = None
        self.radius = 0.5
        self.selector = PointsSelector(view_handler)
        self.measurement = MeasurementTool(view_handler)
        self.measurement.changed.connect(self.measurement_changed)

        view_handler.point_clicked.connect(self._radius_point_clicked)
        view_handler.rectangle_selected.connect(self.rectangle_selected)
        view_handler.undo_requested.connect(self.undo)
        io_handler.cloud_loaded.connect(self._merge_loaded_cloud)
        self.save_requested.connect(io_handler.save)

    def _publish_cloud_display(self):
        cloud = None if self.cloud is None else self.cloud[self.applied_keep]
        self.cloud_display_changed.emit(cloud)

    def _publish_marked_display(self):
        marked = (None if self.cloud is None
                  else self.cloud[self.applied_keep & ~self.keep])
        self.marked_display_changed.emit(marked)

    def _publish_preview(self):
        self._publish_marked_display()

    def set_mode(self, mode):
        self.mode = mode
        self.measurement.set_active(mode == 'measurement')

    def set_radius(self, radius):
        self.radius = float(radius)

    def _radius_point_clicked(self, x, y):
        if self.mode != 'radius' or self.view_handler is None:
            return
        point = self.view_handler.get_point(x, y)
        if point is not None:
            self.mark_deleted_by_radius(point, self.radius)

    def rectangle_selected(self, rect):
        if self.mode != 'rectangle' or self.view_handler is None:
            return
        selected = self.selector.select_rectangle(rect)
        self.mark_deleted_indices(selected[self.keep[selected]])

    def _merge_loaded_cloud(self, cloud, append):
        self.merge_cloud(cloud, append=append)
        self.camera_center_changed.emit(get_cloud_center(cloud))
        self.cloud_loaded.emit(self.cloud)

    def merge_cloud(self, cloud, append=False):
        cloud = cloud[np.isfinite(cloud['xyz']).all(axis=1)]
        if append and self.cloud is not None:
            self.cloud = np.concatenate([self.cloud, cloud])
        else:
            self.cloud = cloud
        self.operations = []
        self.applied_keep = np.ones(self.cloud.shape[0], dtype=bool)
        self.keep = np.ones(self.cloud.shape[0], dtype=bool)
        self.selector.set_points(self.cloud['xyz'])
        self._publish_cloud_display()
        self._publish_preview()

    def mark_deleted_by_radius(self, center, radius):
        indices = self.selector.select_radius(center, radius)
        self.mark_deleted_indices(indices)

    def mark_deleted_indices(self, indices):
        indices = np.asarray(indices, dtype=np.int64)
        if self.cloud is None or indices.size == 0:
            return
        operation = make_op('mark_deleted_indices', indices=indices)
        self.operations.append(operation)
        self.keep = OP_HANDLERS[operation['type']](
            self, self.keep, operation)
        self._publish_preview()

    def undo(self):
        if not self.operations:
            return
        self.operations.pop()
        self.keep = self._replay_operations(self.operations)
        self._publish_preview()

    def undo_measurement(self):
        self.measurement.undo()

    def _replay_operations(self, operations):
        keep = self.applied_keep.copy()
        for operation in operations:
            keep = OP_HANDLERS[operation['type']](self, keep, operation)
        return keep

    def apply_operations(self):
        if self.cloud is None or not self.operations:
            return
        self.applied_keep = self.keep.copy()
        self.operations = []
        self._publish_cloud_display()
        self._publish_preview()

    def cancel_pending(self):
        if self.cloud is None or not self.operations:
            return
        self.keep = self.applied_keep.copy()
        self.operations = []
        self._publish_preview()

    def request_save(self, file):
        if self.cloud is None:
            return
        self.save_requested.emit(self.cloud[self.keep], file)
