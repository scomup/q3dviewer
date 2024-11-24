"""
Copyright 2024  Liu Yang
Distributed under MIT license. See LICENSE for more information.
"""


import numpy as np
import pyqtgraph.opengl as gl
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np
import threading
from PyQt5.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, \
    QSpinBox, QComboBox
from OpenGL.GL import shaders
from q3dviewer.gl_utils import *

vertex_shader = """
#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in uint value;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float alpha = 1;
uniform int color_mode = 0;
uniform float vmin = 0;
uniform float vmax = 255;
uniform float focal = 1000;
uniform int point_type = 0; // 0 pixel, 1 flat square
uniform float point_size_world = 0.01;  // World size for each point (meter)
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

    // Calculate point size in pixels based on distance
    if (point_type == 0)
        gl_PointSize = int(point_size_world);
    else
        gl_PointSize = point_size_world / gl_Position.w * focal;
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
    else if(color_mode == -3)
    {
        uint intensity = value >> 24;
        c = getRainbowColor(intensity);
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


# draw points with color (x, y, z, color)
class CloudItem(gl.GLGraphicsItem.GLGraphicsItem):
    def __init__(self, size, alpha, color_mode='I'):
        super().__init__()
        self.valid_buff_top = 0
        self.add_buff_loc = 0
        self.alpha = alpha
        self.size = size
        self.mutex = threading.Lock()
        self.data_type = [('xyz', '<f4', (3,)), ('color', '<u4')]
        self.color_mode = color_mode
        self.set_color_mode(color_mode)
        self.CAPACITY = 10000000  # 10MB * 3 (x,y,z, color) * 4
        self.vmax = 255
        self.point_type = 0
        self.buff = np.empty((0), self.data_type)
        self.wait_add_data = None
        self.need_update_setting = True

    def add_setting(self, layout):
        label0 = QLabel("Set Point Type:")
        layout.addWidget(label0)
        combo0 = QComboBox()
        combo0.addItem("pixel")
        combo0.addItem("flat square")
        combo0.currentIndexChanged.connect(self.on_combo_selection)
        layout.addWidget(combo0)
        self.label_size = QLabel("Set Size: (pixel)")
        layout.addWidget(self.label_size)
        self.box_size = QDoubleSpinBox()
        self.box_size.setSingleStep(0.01)
        layout.addWidget(self.box_size)
        self.box_size.setValue(self.size)
        self.box_size.valueChanged.connect(self.set_size)
        self.box_size.setRange(0, 100)

        label2 = QLabel("Set Alpha:")
        layout.addWidget(label2)
        box2 = QDoubleSpinBox()
        layout.addWidget(box2)
        box2.setSingleStep(0.01)
        box2.setValue(self.alpha)
        box2.valueChanged.connect(self.set_alpha)
        box2.setRange(0, 1)

        label3 = QLabel("Set ColorMode:")
        label3.setToolTip(
            "'I': intensity mode; 'IRGB': IRGB mode; 'RGB': rgb mode; '#xxxxxx'; matplotlib color, i.e. #FF4500;")
        layout.addWidget(label3)
        box3 = QLineEdit()
        box3.setToolTip(
            "'I': intensity mode; 'IRGB': IRGB mode; 'RGB': rgb mode; '#xxxxxx'; matplotlib color, i.e. #FF4500;")

        box3.setText(str(self.color_mode))
        box3.textChanged.connect(self.set_color_mode)
        layout.addWidget(box3)

    def on_combo_selection(self, index):
        if (index == 0):
            self.label_size.setText("Set Size: (pixel)")
            self.box_size.setSingleStep(1)
            self.point_type = 0
            self.box_size.setValue(1)
        else:
            self.label_size.setText("Set Size: (meter)")
            self.box_size.setSingleStep(0.01)
            self.point_type = 1
            self.box_size.setValue(0.01)
        self.need_update_setting = True

    def set_alpha(self, alpha):
        self.alpha = alpha
        self.need_update_setting = True

    def set_vmax(self, vmax):
        self.vmax = vmax
        self.need_update_setting = True

    def set_color_mode(self, color_mode):
        """
        intensity mode: -1;
        rgb mode: -2;
        matplotlib color: i.e. '#FF4500';
        """
        if (type(color_mode) == str):
            if color_mode.startswith("#"):
                try:
                    self.color_mode_int = int(color_mode[1:], 16)
                except ValueError:
                    return
            elif color_mode == 'RGB':
                self.color_mode_int = -2
            elif color_mode == 'IRGB':
                self.color_mode_int = -3
            elif color_mode == 'I':
                self.color_mode_int = -1
        else:
            return
        self.color_mode = color_mode
        self.need_update_setting = True

    def set_size(self, size):
        self.size = size
        self.need_update_setting = True

    def clear(self):
        data = np.empty((0), self.data_type)
        self.set_data(data)

    def set_data(self, data, append=False):
        if data.dtype is np.dtype('float32') or data.dtype is np.dtype('float64'):
            xyz = data[:, :3]
            if (data.shape[1] == 4):
                color = data[:, 3].view(np.uint32)
            else:
                color = np.zeros(data.shape[0], dtype=np.uint32)
            data = np.rec.fromarrays(
                [xyz, color],
                dtype=self.data_type)
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

    def update_setting(self):
        if (self.need_update_setting is False):
            return
        glUseProgram(self.program)
        set_uniform(self.program, int(self.color_mode_int), 'color_mode')
        set_uniform(self.program, float(self.vmax), 'vmax')
        set_uniform(self.program, float(self.alpha), 'alpha')
        set_uniform(self.program, float(self.size), 'point_size_world')
        set_uniform(self.program, int(self.point_type), 'point_type')
        glUseProgram(0)
        self.need_update_setting = False

    def update_render_buffer(self):
        # is not new data dont update buff
        if (self.wait_add_data is None):
            return
        self.mutex.acquire()

        new_buff_top = self.add_buff_loc + self.wait_add_data.shape[0]
        if new_buff_top > self.buff.shape[0]:
            # if need to update buff capacity, create new cpu buff and new vbo
            buff_capacity = self.buff.shape[0]
            while (new_buff_top > buff_capacity):
                buff_capacity += self.CAPACITY
            print("Update capacity to %d" % buff_capacity)
            new_buff = np.empty((buff_capacity), self.data_type)
            new_buff[:self.add_buff_loc] = self.buff[:self.add_buff_loc]
            new_buff[self.add_buff_loc:new_buff_top] = self.wait_add_data
            self.buff = new_buff
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferData(GL_ARRAY_BUFFER, self.buff.nbytes,
                         self.buff, GL_DYNAMIC_DRAW)
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
        self.set_alpha(self.alpha)
        self.set_color_mode(self.color_mode)
        self.set_vmax(self.vmax)
        self.vbo = glGenBuffers(1)

    def paint(self):
        self.setupGLState()
        self.update_render_buffer()
        self.update_setting()
        glEnable(GL_BLEND)
        glEnable(GL_PROGRAM_POINT_SIZE)
        # glDisable(GL_POINT_SMOOTH)

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glUseProgram(self.program)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 16, ctypes.c_void_p(0))
        glVertexAttribPointer(
            1, 1, GL_FLOAT, GL_UNSIGNED_INT, 16, ctypes.c_void_p(12))
        glEnableVertexAttribArray(0)
        glEnableVertexAttribArray(1)

        view_matrix = np.array(
            self._GLGraphicsItem__view.viewMatrix().data(), np.float32).reshape([4, 4]).T
        set_uniform(self.program, view_matrix, 'view_matrix')
        project_matrix = np.array(self._GLGraphicsItem__view.projectionMatrix(
        ).data(), np.float32).reshape([4, 4]).T
        set_uniform(self.program, project_matrix, 'projection_matrix')
        width = self._GLGraphicsItem__view.deviceWidth()
        focal = project_matrix[0, 0] * width / 2
        set_uniform(self.program, float(focal), 'focal')

        glDrawArrays(GL_POINTS, 0, self.valid_buff_top)

        # unbind VBO
        glDisableVertexAttribArray(0)
        glDisableVertexAttribArray(1)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glUseProgram(0)
