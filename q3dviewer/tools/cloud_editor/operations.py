"""Operation history for point-cloud editing."""

import numpy as np
from q3dviewer.Qt.QtCore import QObject, Signal


def op_mark_deleted_indices(keep, operation):
    """Return a mask with the operation's indices marked as deleted."""
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
    """Manage pending edit operations and their undo/apply lifecycle."""

    cloud_display_changed = Signal(object)
    marked_display_changed = Signal(object)

    def __init__(self):
        super().__init__()
        self.cloud = None
        self.pending_operations = []
        self.applied_keep = None
        self.keep = None

    def set_cloud(self, cloud, append=False):
        """Replace or append the cloud and reset the edit history."""
        cloud = cloud[np.isfinite(cloud['xyz']).all(axis=1)]
        if append and self.cloud is not None:
            self.cloud = np.concatenate([self.cloud, cloud])
        else:
            self.cloud = cloud

        self.pending_operations = []
        self.applied_keep = np.ones(self.cloud.shape[0], dtype=bool)
        self.keep = self.applied_keep.copy()
        self._publish_cloud_display()
        self._publish_marked_display()
        return self.cloud

    def edited_cloud(self):
        """Return the current cloud with pending deletions applied."""
        if self.cloud is None:
            return None
        return self.cloud[self.keep]

    def mark_deleted_indices(self, indices):
        """Add a deletion operation for currently visible point indices."""
        if self.cloud is None:
            return

        indices = np.asarray(indices, dtype=np.int64)
        valid = (indices >= 0) & (indices < self.keep.size)
        indices = indices[valid]
        indices = indices[self.keep[indices]]
        if indices.size == 0:
            return

        operation = make_op(
            'mark_deleted_indices',
            indices=np.unique(indices))
        self.pending_operations.append(operation)
        self.keep = self._apply_operation(self.keep, operation)
        self._publish_marked_display()

    def undo(self):
        if not self.pending_operations:
            return
        self.pending_operations.pop()
        self.keep = self._replay_operations(self.pending_operations)
        self._publish_marked_display()

    def apply_operations(self):
        if self.cloud is None or not self.pending_operations:
            return
        self.applied_keep = self.keep.copy()
        self.pending_operations = []
        self._publish_cloud_display()
        self._publish_marked_display()

    def cancel_pending(self):
        if self.cloud is None or not self.pending_operations:
            return
        self.keep = self.applied_keep.copy()
        self.pending_operations = []
        self._publish_marked_display()

    def _apply_operation(self, keep, operation):
        handler = OP_HANDLERS[operation['type']]
        return handler(keep, operation)

    def _replay_operations(self, operations):
        keep = self.applied_keep.copy()
        for operation in operations:
            keep = self._apply_operation(keep, operation)
        return keep

    def _publish_cloud_display(self):
        cloud = None if self.cloud is None else self.cloud[self.applied_keep]
        self.cloud_display_changed.emit(cloud)

    def _publish_marked_display(self):
        marked = (None if self.cloud is None
                  else self.cloud[self.applied_keep & ~self.keep])
        self.marked_display_changed.emit(marked)
