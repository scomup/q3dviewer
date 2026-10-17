"""Rectangle-based point deletion tool."""

from q3dviewer.Qt.QtCore import QObject, Signal
from q3dviewer.tools.cloud_editor.points_selector import PointsSelector
from q3dviewer.tools.cloud_editor.tool_window import ToolWindow


class RectangleDeleteTool(QObject):
    """Convert rectangle selections into point-index deletion requests."""

    delete_requested = Signal(object)

    def __init__(self, view_handler, selector):
        super().__init__()
        self.active = False
        self.selector = selector
        view_handler.rectangle_selected.connect(self._on_rectangle_selected)

    def set_active(self, active):
        self.active = bool(active)

    def _on_rectangle_selected(self, selection_rect):
        if not self.active:
            return
        indices = self.selector.select(
            PointsSelector.RECTANGLE_MODE, selection_rect)
        self.delete_requested.emit(indices)


class RectangleDeleteWindow(ToolWindow):
    """Controls for the rectangle deletion tool."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Rectangle delete')
        self.set_hint('Ctrl+drag to mark points')
        self.resize(self.width(), self.sizeHint().height())
