"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.Qt.QtCore import Qt, Signal
from q3dviewer.Qt.QtGui import QPainter, QColor
from q3dviewer.Qt.QtWidgets import QSlider, QToolTip


class RangeSlider(QSlider):
    # Signal emitted when the range changes
    rangeChanged = Signal(int, int)

    def __init__(self, orientation=Qt.Horizontal,
                 parent=None, vmin=0, vmax=255):
        super().__init__(orientation, parent)
        self.setMinimum(vmin)
        self.setMaximum(vmax)
        self.lower_value = vmin
        self.upper_value = vmax
        self.active_handle = None
        self.bar_color = QColor(200, 200, 200)  # Gray bar
        self.highlight_color = QColor(100, 100, 255)  # Blue for selected range
        self.handle_color = QColor(50, 50, 255)  # Blue handles

        self.setTickPosition(QSlider.NoTicks)  # Hide original ticks
        # Hide slider handle
        self.setStyleSheet("QSlider::handle { background: transparent; }")

    def mousePressEvent(self, event):
        """Override to handle which handle is selected."""
        pos = self.pixelPosToValue(event.pos())
        if abs(pos - self.lower_value) < abs(pos - self.upper_value):
            self.active_handle = "lower"
        else:
            self.active_handle = "upper"

    def mouseMoveEvent(self, event):
        """Override to update handle positions."""
        if event.buttons() != Qt.LeftButton:
            return

        pos = self.pixelPosToValue(event.pos())
        if self.active_handle == "lower":
            self.lower_value = max(
                self.minimum(), min(pos, self.upper_value - 1))
            QToolTip.showText(event.globalPos(), f"Lower: {self.lower_value:.1f}")
        elif self.active_handle == "upper":
            self.upper_value = min(
                self.maximum(), max(pos, self.lower_value + 1))
            QToolTip.showText(event.globalPos(), f"Upper: {self.upper_value:.1f}")
        self.rangeChanged.emit(self.lower_value, self.upper_value)
        self.update()

    def mouseReleaseEvent(self, event):
        """Override to hide tooltip when mouse stops."""
        QToolTip.hideText()
        super().mouseReleaseEvent(event)

    def enterEvent(self, event):
        """Show tooltips for lower and upper values when mouse enters."""
        QToolTip.showText(self.mapToGlobal(self.rect().topLeft()), 
                          f"Lower: {self.lower_value:.1f}, Upper: {self.upper_value:.1f}")
        super().enterEvent(event)

    def leaveEvent(self, event):
        """Hide tooltips when mouse leaves."""
        QToolTip.hideText()
        super().leaveEvent(event)

    def paintEvent(self, event):
        """Override to paint custom range handles."""
        painter = QPainter(self)

        # Draw the range bar
        painter.setPen(Qt.NoPen)

        bar_height = 6
        bar_y = self.height() // 2 - bar_height // 2
        painter.setBrush(self.bar_color)
        painter.drawRect(0, bar_y, self.width(), bar_height)

        # Draw the selected range
        lower_x = int(self.valueToPixelPos(self.lower_value))
        upper_x = int(self.valueToPixelPos(self.upper_value))
        painter.setBrush(self.highlight_color)
        painter.drawRect(lower_x, bar_y, upper_x - lower_x, bar_height)

        # Draw the range handles
        painter.setBrush(self.handle_color)
        painter.drawEllipse(lower_x - 5, bar_y - 4, 12, 12)
        painter.drawEllipse(upper_x - 5, bar_y - 4, 12, 12)

        painter.end()

    def pixelPosToValue(self, pos):
        """Convert pixel position to slider value."""
        return self.minimum() + (self.maximum() - self.minimum()) \
            * pos.x() / self.width()

    def valueToPixelPos(self, value):
        """Convert slider value to pixel position."""
        return self.width() * (value - self.minimum()) /\
            (self.maximum() - self.minimum())
