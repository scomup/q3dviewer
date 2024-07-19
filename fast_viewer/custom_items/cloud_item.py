#!/usr/bin/env python3

import numpy as np
import pyqtgraph.opengl as gl
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np
import threading
from PyQt5.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, QSpinBox
import threading
from PyQt5.QtGui import QValidator
from OpenGL.GL import shaders


vertex_shader = """
#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in uint value;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float alpha;
uniform int color_mode;
uniform float vmin = 0;
uniform float vmax = 255;
out vec4 color;

vec3 getRainbowColor(uint value_raw) {
    // Normalize value to [0, 1]
    float range = vmax - vmin;
    float value = 1.0 - (float(value_raw) - vmin) / range;
    value = clamp(value, 0.0, 1.0);
    // Convert value to hue in the range [0, 1]
    float hue = value * 5.0 + 1.0;
    int i = int(floor(hue));
    float f = hue - float(i);
    if (mod(i, 2) == 0) f = 1.0 - f; // if i is even
    float n = 1.0 - f;

    // Determine RGB components based on hue value
    vec3 color;
    if (i <= 1) color = vec3(n, 0.0, 1.0);
    else if (i == 2) color = vec3(0.0, n, 1.0);
    else if (i == 3) color = vec3(0.0, 1.0, n);
    else if (i == 4) color = vec3(n, 1.0, 0.0);
    else if (i >= 5) color = vec3(1.0, n, 0.0);

    return color;
}

void main()
{
    vec4 pw = vec4(position, 1.0);
    vec4 pc = view_matrix * pw;
    gl_Position = projection_matrix * pc;
    vec3 c = vec3(1.0, 1.0, 1.0);
    if (color_mode == -1)
    {
        c = getRainbowColor(value);
    }
    else if(color_mode == -2)
    {
        c.z = float(value & uint(0x000000FF))/255.;
        c.y = float((value & uint(0x0000FF00)) >> 8)/255.;
        c.x = float((value & uint(0x00FF0000)) >> 16)/255.;
    }
    else
    {
        c.z = float( uint(color_mode) & uint(0x000000FF))/255.;
        c.y = float((uint(color_mode) & uint(0x0000FF00)) >> 8)/255.;
        c.x = float((uint(color_mode) & uint(0x00FF0000)) >> 16)/255.;
    }
    color = vec4(c, alpha);
}
"""

fragment_shader = """
#version 330 core

in vec4 color;

out vec4 finalColor;

void main()
{
    finalColor = color;
}
"""


class CustomValidator(QValidator):
    def __init__(self, parent=None):
        super().__init__(parent)

    def validate(self, input, pos):
        if input.isdigit():
            return (QValidator.Acceptable, input, pos)
        elif input == '' or input.startswith('#') or input.startswith('-'):
            return (QValidator.Acceptable, input, pos)
        else:
            return (QValidator.Invalid, input, pos)


class CustomLineEdit(QLineEdit):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setValidator(CustomValidator(self))


def set_uniform_mat4(shader, content, name):
    content = content.T
    glUniformMatrix4fv(
        glGetUniformLocation(shader, name),
        1,
        GL_FALSE,
        content.astype(np.float32)
    )


# draw points with color (x, y, z, color)
class CloudItem(gl.GLGraphicsItem.GLGraphicsItem):
    def __init__(self, size, alpha, color_mode=-1):
        super().__init__()
        self.valid_buff_top = 0
        self.add_buff_loc = 0
        self.alpha = alpha
        self.size = size
        self.mutex = threading.Lock()
        self.data_type = [('xyz', '<f4', (3,)), ('color', '<u4')]
        self.color_mode = str(color_mode)  # -1: use rain color, -2: use rgb color:, positive
        self.CAPACITY = 10000000  # 10MB * 3 (x,y,z, color) * 4
        self.vmax = 255
        self.buff = np.empty((0), self.data_type)
        self.wait_add_data = None

    def addSetting(self, layout):
        label1 = QLabel("Set Size:")
        layout.addWidget(label1)
        box1 = QSpinBox()
        box1.setSingleStep(1)
        layout.addWidget(box1)
        box1.setValue(self.size)
        box1.valueChanged.connect(self.setSize)
        box1.setRange(0, 100)

        label2 = QLabel("Set Alpha:")
        layout.addWidget(label2)
        box2 = QDoubleSpinBox()
        layout.addWidget(box2)
        box2.setSingleStep(0.01)
        box2.setValue(self.alpha)
        box2.valueChanged.connect(self.setAlpha)
        box2.setRange(0, 1)

        label3 = QLabel("Set ColorMode:")
        label3.setToolTip("intensity mode: -1; rgb mode: -2; matplotlib color: i.e. #FF4500")
        layout.addWidget(label3)
        box3 = CustomLineEdit()
        box3.setToolTip("intensity mode: -1; rgb mode: -2; matplotlib color: i.e. #FF4500")

        box3.setText(str(self.color_mode))
        box3.textChanged.connect(self.setColorMode)
        layout.addWidget(box3)

    def setAlpha(self, alpha):
        self.alpha = alpha
        if hasattr(self, 'program'):
            glUseProgram(self.program)
            glUniform1f(glGetUniformLocation(self.program, "alpha"), self.alpha)
            glUseProgram(0)

    def setVmax(self, vmax):
        self.vmax = vmax
        if hasattr(self, 'program'):
            glUseProgram(self.program)
            glUniform1f(glGetUniformLocation(self.program, "vmax"), self.vmax)
            glUseProgram(0)

    def setColorMode(self, mode):
        """
        intensity mode: -1;
        rgb mode: -2;
        matplotlib color: i.e. '#FF4500';
        """
        if (type(mode) == str):
            if mode.startswith("#"):
                try:
                    mode = int(mode[1:], 16)
                except ValueError:
                    return
            else:
                try:
                    mode = int(mode)
                except ValueError:
                    return
        if hasattr(self, 'program'):
            glUseProgram(self.program)
            glUniform1i(glGetUniformLocation(self.program, "color_mode"), mode)
            glUseProgram(0)

    def setSize(self, size):
        self.size = size

    def clear(self):
        data = np.empty((0), self.data_type)
        self.setData(data)

    def setData(self, data, append=False):
        self.mutex.acquire()
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

    def updateRenderBuffer(self):
        # is not new data dont update buff
        if(self.wait_add_data is None):
            return
        self.mutex.acquire()

        new_buff_top = self.add_buff_loc + self.wait_add_data.shape[0]
        if new_buff_top > self.buff.shape[0]:
            # if need to update buff capacity, create new cpu buff and new vbo
            buff_capacity = self.buff.shape[0]
            while (new_buff_top > buff_capacity):
                buff_capacity += self.CAPACITY
                print("Update capacity to %d" % buff_capacity)
            self.buff = np.empty((buff_capacity), self.data_type)
            self.buff[self.add_buff_loc:new_buff_top] = self.wait_add_data
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferData(GL_ARRAY_BUFFER, self.buff.nbytes, self.buff, GL_DYNAMIC_DRAW)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
        else:
            self.buff[self.add_buff_loc:new_buff_top] = self.wait_add_data
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferSubData(GL_ARRAY_BUFFER, self.add_buff_loc * 16,
                            self.wait_add_data.shape[0] * 16, self.wait_add_data)
        self.valid_buff_top = new_buff_top
        self.wait_add_data = None
        self.mutex.release()

    def initializeGL(self):
        self.program = shaders.compileProgram(
            shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER),
        )
        # Bind attribute locations
        # set constant parameter for cloud shader
        self.setAlpha(self.alpha)
        self.setColorMode(self.color_mode)
        self.setVmax(self.vmax)
        self.vbo = glGenBuffers(1)

    def paint(self):
        self.setupGLState()
        self.updateRenderBuffer()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glUseProgram(self.program)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 16, ctypes.c_void_p(0))
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_UNSIGNED_INT, 16, ctypes.c_void_p(12))
        glEnableVertexAttribArray(0)
        glEnableVertexAttribArray(1)

        view_matrix = np.array(self._GLGraphicsItem__view.viewMatrix().data(), np.float32).reshape([4, 4]).T
        set_uniform_mat4(self.program, view_matrix, 'view_matrix')
        project_matrix = np.array(self._GLGraphicsItem__view.projectionMatrix().data(), np.float32).reshape([4, 4]).T
        set_uniform_mat4(self.program, project_matrix, 'projection_matrix')

        glPointSize(self.size)
        glDrawArrays(GL_POINTS, 0, self.valid_buff_top)

        # unbind VBO
        glDisableVertexAttribArray(0)
        glDisableVertexAttribArray(1)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glUseProgram(0)
