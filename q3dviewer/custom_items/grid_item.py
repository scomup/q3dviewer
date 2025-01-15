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


class GridItem(BaseItem):
    def __init__(self, size, spacing, color=np.array([255, 255, 255, 76.5]), offset=np.array([0, 0, 0])):
        super().__init__()
        self.size = size
        self.spacing = spacing
        self.color = color
        self.offset = offset

    def add_setting(self, layout):
        label_size = QLabel("Set size:")
        layout.addWidget(label_size)
        spinbox_size = QDoubleSpinBox()
        spinbox_size.setSingleStep(1.0)
        layout.addWidget(spinbox_size)
        spinbox_size.setValue(self.size)
        spinbox_size.valueChanged.connect(self.set_size)
        spinbox_size.setRange(0, 100000)

        label_spacing = QLabel("Set spacing:")
        layout.addWidget(label_spacing)
        spinbox_spacing = QDoubleSpinBox()
        layout.addWidget(spinbox_spacing)
        spinbox_spacing.setSingleStep(0.1)
        spinbox_spacing.setValue(self.spacing)
        spinbox_spacing.valueChanged.connect(self._on_spacing)
        spinbox_spacing.setRange(0, 1000)

        label_offset = QLabel("Set offset (x;y;z):")
        layout.addWidget(label_offset)
        self.edit_offset = QLineEdit()
        regex = QRegularExpression(r"^-?\d+(\.\d+)?;-?\d+(\.\d+)?;-?\d+(\.\d+)?$")
        validator = QRegularExpressionValidator(regex)
        self.edit_offset.setValidator(validator)
        self.edit_offset.setText(f"{self.offset[0]};{self.offset[1]};{self.offset[2]}")
        self.edit_offset.textChanged.connect(self._on_offset)
        layout.addWidget(self.edit_offset)

    def set_size(self, size):
        self.size = size

    def _on_spacing(self, spacing):
        if spacing > 0:
            self.spacing = spacing

    def _on_offset(self, text):
        try:
            values = list(map(float, text.split(';')))
            if len(values) == 3:
                self.offset = np.array(values)
            else:
                raise ValueError("Offset must have 3 values separated by ';'")
        except ValueError:
            pass

    def set_offset(self, offset):
        if isinstance(offset, np.ndarray) and offset.shape == (3,):
            self.offset = offset
            self.edit_offset.setText(f"{self.offset[0]};{self.offset[1]};{self.offset[2]}")
        else:
            raise ValueError("Offset must be a numpy array with shape (3,)")

    def paint(self):
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glColor4f(self.color[0] / 255.0, self.color[1] / 255.0, self.color[2] / 255.0, self.color[3] / 255.0)
        glBegin(GL_LINES)
        for i in np.arange(-self.size, self.size + self.spacing, self.spacing):
            glVertex3f(i + self.offset[0], -self.size + self.offset[1], self.offset[2])
            glVertex3f(i + self.offset[0], self.size + self.offset[1], self.offset[2])
            glVertex3f(-self.size + self.offset[0], i + self.offset[1], self.offset[2])
            glVertex3f(self.size + self.offset[0], i + self.offset[1], self.offset[2])
        glEnd()
        glDisable(GL_BLEND)


