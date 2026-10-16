"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from q3dviewer.base_item import BaseItem
from q3dviewer.Qt import QtCore, QtGui
from q3dviewer.Qt.QtCore import Qt
from q3dviewer.Qt.QtWidgets import (
    QCheckBox, QLabel, QMdiArea, QMdiSubWindow)


class ImageLabel(QLabel):
    image_received = QtCore.Signal(object)
    first_image_received = QtCore.Signal(object)
    close_requested = QtCore.Signal()
    _resize_grip_size = 16
    _close_button_size = 16

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pixmap = None
        self._drag_offset = None
        self._resize_start = None
        self._resize_size = None
        self._initialized = False
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(80, 40)
        self.setStyleSheet('background-color: black;')
        self.image_received.connect(self._update_image)

    def set_image(self, image):
        self.image_received.emit(image)

    def _update_image(self, image):
        image = np.ascontiguousarray(image)
        if image.ndim == 2 or image.shape[2] == 1:
            if image.ndim == 3:
                image = image[:, :, 0]
            qimage = QtGui.QImage(
                image.data, image.shape[1], image.shape[0], image.strides[0],
                QtGui.QImage.Format_Grayscale8).copy()
        elif image.shape[2] == 3:
            qimage = QtGui.QImage(
                image.data, image.shape[1], image.shape[0], image.strides[0],
                QtGui.QImage.Format_RGB888).copy()
        elif image.shape[2] == 4:
            qimage = QtGui.QImage(
                image.data, image.shape[1], image.shape[0], image.strides[0],
                QtGui.QImage.Format_RGBA8888).copy()
        else:
            raise ValueError('Unsupported image channel count')
        self._pixmap = QtGui.QPixmap.fromImage(qimage)
        if not self._initialized:
            self._initialized = True
            self.first_image_received.emit(qimage)
        self._refresh_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self):
        if self._pixmap is not None:
            self.setPixmap(self._pixmap.scaled(
                self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QtGui.QColor(190, 190, 190, 220))
        size = self._resize_grip_size
        painter.drawPolygon(QtGui.QPolygon([
            QtCore.QPoint(self.width() - size, self.height()),
            QtCore.QPoint(self.width(), self.height() - size),
            QtCore.QPoint(self.width(), self.height()),
        ]))
        painter.setPen(QtGui.QPen(QtGui.QColor(190, 190, 190, 220), 1))
        size = self._close_button_size
        painter.drawLine(self.width() - size, 0, self.width(), size)
        painter.drawLine(self.width(), 0, self.width() - size, size)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            subwindow = self._subwindow()
            if subwindow is not None:
                if self._in_close_button(event.pos()):
                    self.close_requested.emit()
                    event.accept()
                    return
                if self._in_resize_grip(event.pos()):
                    self._resize_start = event.globalPos()
                    self._resize_size = subwindow.size()
                    event.accept()
                    return
                top_left = subwindow.mapToGlobal(QtCore.QPoint(0, 0))
                self._drag_offset = event.globalPos() - top_left
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._resize_start is not None and event.buttons() & Qt.LeftButton:
            self._resize_subwindow(event.globalPos())
            event.accept()
            return
        if self._drag_offset is not None and event.buttons() & Qt.LeftButton:
            subwindow = self._subwindow()
            if subwindow is not None:
                parent = subwindow.parentWidget()
                subwindow.move(parent.mapFromGlobal(
                    event.globalPos() - self._drag_offset))
                event.accept()
                return
        if not event.buttons():
            self._set_cursor(self._in_resize_grip(event.pos()))

    def mouseReleaseEvent(self, event):
        self._drag_offset = None
        self._resize_start = None
        self._resize_size = None
        self._set_cursor(False)
        event.accept()

    def _in_resize_grip(self, position):
        size = self._resize_grip_size
        return (position.x() >= self.width() - size and
                position.y() >= self.height() - size)

    def _in_close_button(self, position):
        size = self._close_button_size
        return position.x() >= self.width() - size and position.y() <= size

    def _set_cursor(self, in_resize_grip):
        cursor = Qt.SizeFDiagCursor if in_resize_grip else Qt.ArrowCursor
        self.setCursor(cursor)

    def _resize_subwindow(self, global_position):
        subwindow = self._subwindow()
        if subwindow is None:
            return
        delta = global_position - self._resize_start
        width = max(self.minimumWidth(), self._resize_size.width() + delta.x())
        height = max(self.minimumHeight(), self._resize_size.height() + delta.y())
        subwindow.resize(width, height)

    def _subwindow(self):
        widget = self.parentWidget()
        while widget is not None:
            if isinstance(widget, QMdiSubWindow):
                return widget
            widget = widget.parentWidget()
        return None


class ImageItem(BaseItem):
    def __init__(self, pos=np.array([0, 0]), size=np.array([1280/2, 720/2]), corner=None):
        BaseItem.__init__(self)
        self.pos = pos
        self.size = size
        self.image = None
        self.corner = corner

    def initialize_gl(self):
        mdi = self.glwidget().findChild(QMdiArea)
        if mdi is None:
            raise RuntimeError('ImageItem requires a QMdiArea on the GLWidget')
        self.image_label = ImageLabel()
        self.image_label.first_image_received.connect(
            self._initialize_image_window)
        self.image_label.close_requested.connect(self.hide)
        self.subwindow = mdi.addSubWindow(self.image_label)
        self.subwindow.setWindowFlags(Qt.FramelessWindowHint)
        self.subwindow.setGeometry(
            int(self.pos[0]), int(self.pos[1]),
            int(self.size[0]), int(self.size[1]))
        if self.visible():
            self.subwindow.show()
        else:
            self.subwindow.hide()
        if self.image is not None:
            self.image_label.set_image(self.image)

    def _initialize_image_window(self, qimage):
        width = self.subwindow.width()
        height = max(
            self.image_label.minimumHeight(),
            round(width * qimage.height() / qimage.width()))
        self.subwindow.resize(width, height)
        if self.corner is not None:
            self.move_to_corner(self.corner)
        self.subwindow.show()

    def set_data(self, data):
        if not isinstance(data, np.ndarray):
            print("The image type is not supported.")
            raise NotImplementedError
        self.image = data
        if hasattr(self, 'image_label'):
            self.image_label.set_image(data)

    def move_to_corner(self, corner):
        corners = {'TL', 'TR', 'BR', 'BL'}
        corner = corner.upper()
        if corner not in corners:
            raise ValueError(
                "corner must be one of 'TL', 'TR', 'BR', or 'BL'")

        if not hasattr(self, 'subwindow'):
            self.corner = corner
            self.pos = np.array([0, 0])
            return

        parent = self.subwindow.parentWidget()
        width = parent.width() - self.subwindow.width()
        height = parent.height() - self.subwindow.height()
        x = width if corner in {'TR', 'BR'} else 0
        y = height if corner in {'BR', 'BL'} else 0
        self.subwindow.move(max(0, x), max(0, y))
        self.pos = np.array([max(0, x), max(0, y)])
        self.corner = corner

    def set_visible(self, visible):
        self._visible = visible
        if hasattr(self, 'subwindow'):
            self.subwindow.setVisible(visible)
        if hasattr(self, 'checkbox_show'):
            self.checkbox_show.blockSignals(True)
            self.checkbox_show.setChecked(visible)
            self.checkbox_show.blockSignals(False)

    def hide(self):
        self.set_visible(False)

    def show(self):
        self.set_visible(True)

    def add_setting(self, layout):
        self.checkbox_show = QCheckBox("Show Image")
        self.checkbox_show.setChecked(self.visible())
        self.checkbox_show.stateChanged.connect(self.set_visible)
        layout.addWidget(self.checkbox_show)
