import pyqtgraph.opengl as gl
from OpenGL.GL import *
import numpy as np
from PyQt5.QtWidgets import QLabel, QDoubleSpinBox


class GridItem(gl.GLGridItem):
    def __init__(self, size, spacing):
        super(GridItem, self).__init__()
        self.setSize0(size)
        self.setSpacing0(spacing)

    def addSetting(self, parent):
        label1 = QLabel("Set size:")
        parent.layout.addWidget(label1)
        box1 = QDoubleSpinBox()
        box1.setSingleStep(1.0)
        parent.layout.addWidget(box1)
        box1.setValue(self.size0)
        box1.valueChanged.connect(self.setSize0)
        box1.setRange(0, 100000)

        label2 = QLabel("Set spacing:")
        parent.layout.addWidget(label2)
        box2 = QDoubleSpinBox()
        parent.layout.addWidget(box2)
        box2.setSingleStep(0.1)
        box2.setValue(self.spacing0)
        box2.valueChanged.connect(self.setSpacing0)
        box2.setRange(0, 1000)

        parent.tmp_widgets.append(label1)
        parent.tmp_widgets.append(box1)
        parent.tmp_widgets.append(label2)
        parent.tmp_widgets.append(box2)

    def setSize0(self, size):
        self.size0 = size
        self.setSize(self.size0, self.size0)

    def setSpacing0(self, spacing):
        self.spacing0 = spacing
        self.setSpacing(self.spacing0, self.spacing0)
