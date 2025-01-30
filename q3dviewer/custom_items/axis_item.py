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


# Vertex and Fragment shader source code
vertex_shader_source = """
#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;

out vec3 ourColor;

uniform mat4 view_matrix;
uniform mat4 project_matrix;
uniform mat4 model_matrix;
uniform float size;

void main()
{
    vec3 scaled_position = position * size;
    gl_Position = project_matrix * view_matrix * model_matrix * vec4(scaled_position, 1.0);
    ourColor = color;
}
"""

fragment_shader_source = """
#version 330 core
in vec3 ourColor;
out vec4 color;

void main()
{
    color = vec4(ourColor, 1.0);
}
"""


class AxisItem(BaseItem):
    def __init__(self, size=1.0, width=2):
        super().__init__()
        self.size = size
        self.width = width
        self.T = np.eye(4, dtype=np.float32)
        self.need_update_setting = True

    def initialize_gl(self):
        # Axis vertices and colors
        self.vertices = np.array([
            # positions         # colors
            [0.0, 0.0, 0.0,    1.0, 0.0, 0.0],  # X axis (red)
            [1.0, 0.0, 0.0,    1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0,    0.0, 1.0, 0.0],  # Y axis (green)
            [0.0, 1.0, 0.0,    0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0,    0.0, 0.0, 1.0],  # Z axis (blue)
            [0.0, 0.0, 1.0,    0.0, 0.0, 1.0],
        ], dtype=np.float32)

        self.vao = glGenVertexArrays(1)
        vbo = glGenBuffers(1)

        glBindVertexArray(self.vao)

        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(GL_ARRAY_BUFFER, self.vertices.nbytes, self.vertices, GL_STATIC_DRAW)

        # Vertex positions
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, ctypes.c_void_p(0))
        glEnableVertexAttribArray(0)

        # Vertex colors
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 24, ctypes.c_void_p(12))
        glEnableVertexAttribArray(1)

        # Compile shaders and create shader program
        self.program = shaders.compileProgram(
            shaders.compileShader(vertex_shader_source, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader_source, GL_FRAGMENT_SHADER),
        )

        glBindVertexArray(0)

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
        self.need_update_setting = True

    def update_setting(self):
        if not self.need_update_setting:
            return
        glUseProgram(self.program)
        set_uniform(self.program, float(self.size), 'size')
        set_uniform(self.program, self.T, 'model_matrix')
        glUseProgram(0)
        self.need_update_setting = False

    def set_width(self, width):
        self.width = width
        
    def set_transform(self, transform):
        """
        Set the transformation matrix for the axis item.
        """
        self.T = transform
        self.need_update_setting = True

    def paint(self):
        self.update_setting()
        glLineWidth(self.width)
        glUseProgram(self.program)
        glBindVertexArray(self.vao)

        view_matrix = self.glwidget().get_view_matrix()
        project_matrix = self.glwidget().get_projection_matrix()
        set_uniform(self.program, view_matrix, 'view_matrix')
        set_uniform(self.program, project_matrix, 'project_matrix')

        glDrawArrays(GL_LINES, 0, 6)

        glBindVertexArray(0)
        glUseProgram(0)
        glLineWidth(1)
