"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
import numpy as np
import threading
from PySide6.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox
from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
from q3dviewer.utils.maths import hex_to_rgba


class LineItem(BaseItem):
    def __init__(self, width=1, color='#00ff00', line_type='LINE_STRIP'):
        super(LineItem, self).__init__()
        self.width = width
        self.buff = np.empty((0, 3), np.float32)
        self.wait_add_data = None
        self.mutex = threading.Lock()
        self.capacity = 100000
        self.valid_buff_top = 0
        self.color = color
        self.rgb = hex_to_rgba(color)
        self.line_type = GL_LINE_STRIP if line_type == 'LINE_STRIP' else GL_LINES

    def add_setting(self, layout):
        label_color = QLabel("Color:")
        layout.addWidget(label_color)
        self.color_edit = QLineEdit()
        self.color_edit.setToolTip("Hex number, i.e. #FF4500")
        self.color_edit.setText(self.color)
        self.color_edit.textChanged.connect(self._on_color)
        regex = QRegularExpression(r"^#[0-9A-Fa-f]{6}$")
        validator = QRegularExpressionValidator(regex)
        self.color_edit.setValidator(validator)
        layout.addWidget(self.color_edit)

        spinbox_width = QDoubleSpinBox()
        spinbox_width.setPrefix("Width: ")
        spinbox_width.setSingleStep(0.1)
        spinbox_width.setValue(self.width)
        spinbox_width.setRange(0.1, 10.0)
        spinbox_width.valueChanged.connect(self.set_width)
        layout.addWidget(spinbox_width)

    def _on_color(self, color):
        try:
            self.rgb = hex_to_rgba(color)
            self.color = color
        except ValueError:
            pass

    def set_color(self, color):
        self.color_edit.setText(color)

    def set_width(self, width):
        self.width = width

    def set_data(self, data, append=False):
        self.mutex.acquire()
        data = data.astype(np.float32).reshape(-1, 3)
        if (append is False):
            self.wait_add_data = data
            self.add_buff_loc = 0
        else:
            if (self.wait_add_data is None):
                self.wait_add_data = data
            else:
                self.wait_add_data = np.concatenate([self.wait_add_data, data])
            self.add_buff_loc = self.valid_buff_top
        self.mutex.release()

    def update_render_buffer(self):
        if (self.wait_add_data is None):
            return
        self.mutex.acquire()

        new_buff_top = self.add_buff_loc + self.wait_add_data.shape[0]
        if new_buff_top > self.buff.shape[0]:
            buff_capacity = self.buff.shape[0]
            while (new_buff_top > buff_capacity):
                buff_capacity += self.capacity
            self.buff = np.empty((buff_capacity, 3), np.float32)
            self.buff[self.add_buff_loc:new_buff_top] = self.wait_add_data
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferData(GL_ARRAY_BUFFER, self.buff.nbytes,
                         self.buff, GL_DYNAMIC_DRAW)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
        else:
            self.buff[self.add_buff_loc:new_buff_top] = self.wait_add_data
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferSubData(GL_ARRAY_BUFFER, self.add_buff_loc * 12,
                            self.wait_add_data.shape[0] * 12,
                            self.wait_add_data)
        self.valid_buff_top = new_buff_top
        self.wait_add_data = None
        self.mutex.release()

    def initialize_gl(self):
        self.vbo = glGenBuffers(1)

    def paint(self):
        self.update_render_buffer()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(3, GL_FLOAT, 0, None)
        glLineWidth(self.width)
        glColor4f(*self.rgb)

        glDrawArrays(self.line_type, 0, self.valid_buff_top)
        glDisableClientState(GL_VERTEX_ARRAY)

        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDisable(GL_BLEND)
