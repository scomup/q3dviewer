#!/usr/bin/env python3

import numpy as np
import pyqtgraph.opengl as gl
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np
import threading
from PyQt5.QtWidgets import QPushButton, QLabel, QLineEdit, QDoubleSpinBox, QSpinBox, QMessageBox
import threading
import os
from pathlib import Path
from pypcd4 import PointCloud, MetaData
from PyQt5.QtGui import QValidator


vertex_shader = """
#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in uint value;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float alpha;
uniform int color_mode;
out vec4 color;

uniform float scalar_min = 0;
uniform float scalar_max = 255;

vec3 getRainbowColor(uint value_raw) {
    // Normalize value to [0, 1]
    float range = scalar_max - scalar_min;
    float value = 1.0 - (float(value_raw) - scalar_min) / range;
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
        if input == '' or input.startswith('#'):
            return (QValidator.Acceptable, input, pos)
        elif input.isdigit():
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


CAPACITY = 10000000  # 10MB * 3 (x,y,z, color) * 4


# draw points with color (x, y, z, color)
class CloudItem(gl.GLGraphicsItem.GLGraphicsItem):
    def __init__(self, size, alpha, color_mode=-1):
        super().__init__()
        self.valid_buff_num = 0
        self.alpha = alpha
        self.size = size
        self.mutex = threading.Lock()
        self.data_type = [('xyz', '<f4', (3,)), ('color', '<u4')]
        self.points_capacity = 0
        self.wait_add_buff_num = 0
        self.data = np.empty((0), self.data_type)
        self.update_buff_capacity = True
        self.valid_buff_num = 0
        self.color_mode = str(color_mode)  # -1: use rain color, -2: use rgb color:, positive
        self.save_path = str(Path(os.getcwd(), "data.pcd"))

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

        label4 = QLabel("Save Path:")
        layout.addWidget(label4)
        box4 = QLineEdit()
        box4.setText(self.save_path)
        box4.textChanged.connect(self.setPath)
        layout.addWidget(box4)
        save_button = QPushButton("Save Cloud")
        save_button.clicked.connect(self.saveFile)
        layout.addWidget(save_button)

    def saveFile(self):
        cloud = self.data[:self.valid_buff_num]
        if self.save_path.endswith(".pcd"):
            fields = ("x", "y", "z", "rgb")
            metadata = MetaData.model_validate(
                {
                    "fields": fields,
                    "size": [4, 4, 4, 4],
                    "type": ['F', 'F', 'F', 'U'],
                    "count": [1, 1, 1, 1],
                    "width": cloud.shape[0],
                    "points": cloud.shape[0],
                })
            pc = PointCloud(metadata, cloud)
            try:
                pc.save(self.save_path)
                save_msg = QMessageBox()
                save_msg.setIcon(QMessageBox.Information)
                save_msg.setWindowTitle("save")
                save_msg.setStandardButtons(QMessageBox.Ok)
                save_msg.setText("save OK!")
            except:
                save_msg.setText("Cannot save to %s" % self.save_path)
            save_msg.exec()

        elif self.save_path.endswith(".ply"):
            print("Not implment yet!")
        else:
            print("Not supported cloud file type!")

    def setAlpha(self, alpha):
        self.alpha = alpha
        glUseProgram(self.program)
        glUniform1f(glGetUniformLocation(self.program, "alpha"), self.alpha)
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
        glUseProgram(self.program)
        glUniform1i(glGetUniformLocation(self.program, "color_mode"), mode)
        glUseProgram(0)

    def setSize(self, size):
        self.size = size

    def setPath(self, path):
        self.save_path = path

    def setData(self, data):
        self.mutex.acquire()
        new_point_num = data.shape[0]

        while (new_point_num > self.points_capacity):
            self.update_buff_capacity = True
            self.points_capacity += CAPACITY

        if (self.update_buff_capacity):
            self.data = np.empty((self.points_capacity), self.data_type)

        self.valid_buff_num = 0  # reset the buff to 0
        self.data[:new_point_num] = data
        self.wait_add_buff_num = new_point_num
        self.mutex.release()

    def appendData(self, data):
        self.mutex.acquire()
        new_point_num = data.shape[0]
        old_point_num = self.valid_buff_num + self.wait_add_buff_num

        while (old_point_num + new_point_num > self.points_capacity):
            self.update_buff_capacity = True
            self.points_capacity += CAPACITY
            print("Update capacity to %d" % self.points_capacity)

        if (self.update_buff_capacity):
            # copy old data
            new_data = np.empty((self.points_capacity), self.data_type)
            new_data[:old_point_num] = self.data[:old_point_num]
            self.data = new_data

        # copy new data
        self.data[old_point_num:old_point_num + new_point_num] = data
        self.wait_add_buff_num += new_point_num
        self.mutex.release()

    def updateRenderBuffer(self):
        if(self.wait_add_buff_num == 0):
            return
        self.mutex.acquire()
        if self.update_buff_capacity:
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferData(GL_ARRAY_BUFFER, self.data.nbytes, self.data, GL_STATIC_DRAW)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            self.update_buff_capacity = False
            self.valid_buff_num += self.wait_add_buff_num
        else:
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferSubData(GL_ARRAY_BUFFER, self.valid_buff_num * 16,
                            self.wait_add_buff_num * 16, self.data[self.valid_buff_num:])
            self.valid_buff_num += self.wait_add_buff_num
        self.wait_add_buff_num = 0
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
        self.vbo = glGenBuffers(1)

    def paint(self):
        self.view_matrix = np.array(self._GLGraphicsItem__view.viewMatrix().data(), np.float32).reshape([4, 4]).T
        self.setupGLState()
        if self.valid_buff_num == 0 and self.wait_add_buff_num == 0:
            return

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        self.updateRenderBuffer()
        glUseProgram(self.program)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 16, ctypes.c_void_p(0))
        glVertexAttribPointer(1, 1, GL_FLOAT, GL_UNSIGNED_INT, 16, ctypes.c_void_p(12))
        glEnableVertexAttribArray(0)
        glEnableVertexAttribArray(1)

        set_uniform_mat4(self.program, self.view_matrix, 'view_matrix')
        project_matrix = np.array(self._GLGraphicsItem__view.projectionMatrix().data(), np.float32).reshape([4, 4]).T
        set_uniform_mat4(self.program, project_matrix, 'projection_matrix')
        glPointSize(self.size)
        glDrawArrays(GL_POINTS, 0, self.valid_buff_num)

        # unbind VBO
        glDisableVertexAttribArray(0)
        glDisableVertexAttribArray(1)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glUseProgram(0)
