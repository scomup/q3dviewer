"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.base_item import BaseItem
from q3dviewer.utils import set_uniform
from OpenGL.GL import *
import numpy as np
from OpenGL.GL import shaders
from PySide6.QtWidgets import QLabel, QDoubleSpinBox


class AxisItem(BaseItem):
    def __init__(self, size=1.0, width=2):
        super().__init__()
        self.size = size
        self.width = width
        self.T = np.eye(4, dtype=np.float32)
        self.need_update_setting = True

    def initialize_gl(self):
        # Axis vertices
        self.vertices = np.array([
            # positions
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],  # X axis
            [0.0, 0.0, 0.0], [0.0, 1.0, 0.0],  # Y axis
            [0.0, 0.0, 0.0], [0.0, 0.0, 1.0],  # Z axis
        ], dtype=np.float32)

        # Axis colors
        self.colors = np.array([
            [1.0, 0.0, 0.0], [1.0, 0.0, 0.0],  # X axis (red)
            [0.0, 1.0, 0.0], [0.0, 1.0, 0.0],  # Y axis (green)
            [0.0, 0.0, 1.0], [0.0, 0.0, 1.0],  # Z axis (blue)
        ], dtype=np.float32)


    def add_setting(self, layout):
        label_size = QLabel("Set size:")
        layout.addWidget(label_size)
        spinbox_size = QDoubleSpinBox()
        spinbox_size.setSingleStep(0.1)
        layout.addWidget(spinbox_size)
        spinbox_size.setValue(self.size)
        spinbox_size.valueChanged.connect(self.set_size)
        spinbox_size.setRange(0.0, 100)

        label_width = QLabel("Set width:")
        layout.addWidget(label_width)
        spinbox_width = QDoubleSpinBox()
        layout.addWidget(spinbox_width)
        spinbox_width.setSingleStep(0.1)
        spinbox_width.setValue(self.width)
        spinbox_width.valueChanged.connect(self.set_width)
        spinbox_width.setRange(0, 1000)

    def set_size(self, size):
        self.size = size

    def set_width(self, width):
        self.width = width
        
    def set_transform(self, transform):
        """
        Set the transformation matrix for the axis item.
        """
        self.T = transform
        self.need_update_setting = True

    def paint(self):
        glLineWidth(self.width)

        glPushMatrix()
        glMultMatrixf(self.T.T)

        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(3, GL_FLOAT, 0, self.vertices * self.size)

        glEnableClientState(GL_COLOR_ARRAY)
        glColorPointer(3, GL_FLOAT, 0, self.colors)

        glDrawArrays(GL_LINES, 0, len(self.vertices))

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
        glPopMatrix()

        glLineWidth(1)
