"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
from PySide6.QtWidgets import QLabel, QDoubleSpinBox, QLineEdit
from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator
import numpy as np
from q3dviewer.utils.maths import hex_to_rgba


class GridItem(BaseItem):
    def __init__(self, size=100, spacing=20, color='#ffffff40', offset=np.array([0., 0., 0.])):
        super().__init__()
        self.size = size
        self.spacing = spacing
        self.offset = offset
        self.need_update_grid = True
        self.set_color(color)
        self.vertices = self.generate_grid_vertices()

    def set_color(self, color):
        try:
            self.rgba = hex_to_rgba(color)
        except ValueError:
            pass

    def generate_grid_vertices(self):
        vertices = []
        x, y, z = self.offset
        half_size = self.size / 2  # Keep as float
        for i in np.arange(-half_size, half_size + self.spacing, self.spacing):
            vertices.extend([i + x, -half_size + y, z, i + x, half_size + y, z])  # Grid lines parallel to Y axis
            vertices.extend([-half_size + x, i + y, z, half_size + x, i + y, z])  # Grid lines parallel to X axis
        vertices = np.array(vertices, dtype=np.float32)
        return vertices

    def initialize_gl(self):
        self.vao = glGenVertexArrays(1)
        vbo = glGenBuffers(1)

        glBindVertexArray(self.vao)

        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(GL_ARRAY_BUFFER, self.vertices.nbytes, self.vertices, GL_STATIC_DRAW)

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, ctypes.c_void_p(0))
        glEnableVertexAttribArray(0)

        glBindVertexArray(0)

    def add_setting(self, layout):
        spinbox_size = QDoubleSpinBox()
        spinbox_size.setPrefix("Size: ")
        spinbox_size.setSingleStep(1.0)
        spinbox_size.setValue(self.size)
        spinbox_size.setRange(0, 100000)
        spinbox_size.valueChanged.connect(self.set_size)
        layout.addWidget(spinbox_size)

        spinbox_spacing = QDoubleSpinBox()
        spinbox_spacing.setPrefix("Spacing: ")
        spinbox_spacing.setSingleStep(0.1)
        spinbox_spacing.setValue(self.spacing)
        spinbox_spacing.setRange(0, 1000)
        spinbox_spacing.valueChanged.connect(self._on_spacing)
        layout.addWidget(spinbox_spacing)

        spinbox_offset_x = QDoubleSpinBox()
        spinbox_offset_x.setPrefix("Offset X: ")
        spinbox_offset_x.setSingleStep(0.1)
        spinbox_offset_x.setValue(self.offset[0])
        spinbox_offset_x.valueChanged.connect(self._on_offset_x)
        layout.addWidget(spinbox_offset_x)
        
        spinbox_offset_y = QDoubleSpinBox()
        spinbox_offset_y.setPrefix("Offset Y: ")
        spinbox_offset_y.setSingleStep(0.1)
        spinbox_offset_y.setValue(self.offset[1])
        spinbox_offset_y.valueChanged.connect(self._on_offset_y)
        layout.addWidget(spinbox_offset_y)
        
        spinbox_offset_z = QDoubleSpinBox()
        spinbox_offset_z.setPrefix("Offset Z: ")
        spinbox_offset_z.setSingleStep(0.1)
        spinbox_offset_z.setValue(self.offset[2])
        spinbox_offset_z.valueChanged.connect(self._on_offset_z)
        layout.addWidget(spinbox_offset_z)

    def set_size(self, size):
        self.size = size
        self.need_update_grid = True

    def _on_spacing(self, spacing):
        if spacing > 0:
            self.spacing = spacing
            self.need_update_grid = True

    def _on_offset_x(self, value):
        self.offset[0] = value
        print(self.offset)
        self.need_update_grid = True

    def _on_offset_y(self, value):
        self.offset[1] = value
        self.need_update_grid = True

    def _on_offset_z(self, value):
        self.offset[2] = value
        self.need_update_grid = True

    def set_offset(self, offset):
        if isinstance(offset, np.ndarray) and offset.shape == (3,):
            self.offset = offset
            self.need_update_grid = True
        else:
            raise ValueError("Offset must be a numpy array with shape (3,)")

    def paint(self):
        if self.need_update_grid:
            self.vertices = self.generate_grid_vertices()
            self.initialize_gl()
            self.need_update_grid = False
        
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glLineWidth(1)
        glColor4f(*self.rgba)
        glBindVertexArray(self.vao)
        glDrawArrays(GL_LINES, 0, len(self.vertices) // 3)
        glBindVertexArray(0)
        glLineWidth(1)
        glDisable(GL_BLEND)


