#!/usr/bin/env python3

import numpy as np
import pyqtgraph.opengl as gl
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np
from PyQt5.QtWidgets import QLabel, QDoubleSpinBox


class GLAxisItem(gl.GLGraphicsItem.GLGraphicsItem):
    def __init__(self, size=1., width=5, glOptions='translucent'):
        gl.GLGraphicsItem.GLGraphicsItem.__init__(self)
        self.size = size
        self.width = width
        self.org = np.array([0, 0, 0, 1])
        self.axis_x = np.array([self.size, 0, 0, 1])
        self.axis_y = np.array([0, self.size, 0, 1])
        self.axis_z = np.array([0, 0, self.size, 1])
        self.setGLOptions(glOptions)
        self.T = np.eye(4)
        self.settings = []

    def addSetting(self, layout):
        label1 = QLabel("Set size:")
        layout.addWidget(label1)
        box1 = QDoubleSpinBox()
        box1.setSingleStep(0.1)
        layout.addWidget(box1)
        box1.setValue(self.size)
        box1.valueChanged.connect(self.setSize)
        box1.setRange(0.0, 100)

        label2 = QLabel("Set Width:")
        layout.addWidget(label2)
        box2 = QDoubleSpinBox()
        layout.addWidget(box2)
        box2.setSingleStep(0.1)
        box2.setValue(self.width)
        box2.valueChanged.connect(self.setWidth)
        box2.setRange(0, 1000)

    def setSize(self, size):
        self.size = size
        self.axis_x = np.array([self.size, 0, 0, 1])
        self.axis_y = np.array([0, self.size, 0, 1])
        self.axis_z = np.array([0, 0, self.size, 1])

    def setWidth(self, width):
        self.width = width

    def setTransform(self, T):
        self.T = T

    def paint(self):
        org = self.T.dot(self.org)
        axis_x = self.T.dot(self.axis_x)
        axis_y = self.T.dot(self.axis_y)
        axis_z = self.T.dot(self.axis_z)
        self.setupGLState()
        glLineWidth(self.width)
        glBegin(GL_LINES)
        glColor4f(0, 0, 1, 1)  # z is blue
        glVertex3f(org[0], org[1], org[2])
        glVertex3f(axis_z[0], axis_z[1], axis_z[2])
        glColor4f(0, 1, 0, 1)  # y is green
        glVertex3f(org[0], org[1], org[2])
        glVertex3f(axis_y[0], axis_y[1], axis_y[2])
        glColor4f(1, 0, 0, 1)  # x is red
        glVertex3f(org[0], org[1], org[2])
        glVertex3f(axis_x[0], axis_x[1], axis_x[2])
        glEnd()
