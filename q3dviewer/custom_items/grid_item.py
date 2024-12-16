"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import pyqtgraph.opengl as gl
from OpenGL.GL import *
from PyQt5.QtWidgets import QLabel, QDoubleSpinBox


class GridItem(gl.GLGridItem):
    def __init__(self, size, spacing, color=(255, 255, 255, 76.5)):
        super(GridItem, self).__init__(color=color)
        self.setSize0(size)
        self.setSpacing0(spacing)

    def addSetting(self, layout):
        label1 = QLabel("Set size:")
        layout.addWidget(label1)
        box1 = QDoubleSpinBox()
        box1.setSingleStep(1.0)
        layout.addWidget(box1)
        box1.setValue(self.size0)
        box1.valueChanged.connect(self.setSize0)
        box1.setRange(0, 100000)

        label2 = QLabel("Set spacing:")
        layout.addWidget(label2)
        box2 = QDoubleSpinBox()
        layout.addWidget(box2)
        box2.setSingleStep(0.1)
        box2.setValue(self.spacing0)
        box2.valueChanged.connect(self.setSpacing0)
        box2.setRange(0, 1000)

    def setSize0(self, size):
        self.size0 = size
        self.setSize(self.size0, self.size0)

    def setSpacing0(self, spacing):
        self.spacing0 = spacing
        self.setSpacing(self.spacing0, self.spacing0)
