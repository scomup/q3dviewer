"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""


import numpy as np
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import threading
from PyQt5.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, \
    QComboBox, QCheckBox
from OpenGL.GL import shaders
from q3dviewer.gl_utils import *
from q3dviewer.range_slider import RangeSlider
from PyQt5.QtCore import QRegularExpression
from PyQt5.QtGui import QRegularExpressionValidator
from pyqtgraph.Qt import QtCore


vertex_shader = """
#version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in uint value;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform float alpha = 1;
uniform int color_mode = 0;
uniform int flat_rgb = 0;
uniform float vmin = 0;
uniform float vmax = 255;
uniform float focal = 1000;
uniform int point_type = 0; // 0 pixel, 1 flat square, 2 sphere
uniform float point_size = 0.01;  // World size for each point (meter)
out vec4 color;


vec3 getRainbowColor(uint value_raw) {
    float range = vmax - vmin;
    float value = 1.0 - (float(value_raw) - vmin) / range;
    value = clamp(value, 0.0, 1.0);
    float hue = value * 5.0 + 1.0;
    int i = int(floor(hue));
    float f = hue - float(i);
    if (mod(i, 2) == 0) f = 1.0 - f;
    float n = 1.0 - f;

    vec3 color;
    if (i <= 1) color = vec3(n, 0.0, 1.0);
    else if (i == 2) color = vec3(0.0, n, 1.0);
    else if (i == 3) color = vec3(0.0, 1.0, n);
    else if (i == 4) color = vec3(n, 1.0, 0.0);
    else color = vec3(1.0, n, 0.0);
    return color;
}

void main()
{
    vec4 pw = vec4(position, 1.0);
    vec4 pc = view_matrix * pw;
    gl_Position = projection_matrix * pc;

    // Calculate point size in pixels based on distance
    if (point_type == 0)
        gl_PointSize = int(point_size);
    else
        gl_PointSize = point_size / gl_Position.w * focal;
    vec3 c = vec3(1.0, 1.0, 1.0);
    if (color_mode == 1)
    {
        c = getRainbowColor(value);
    }
    else if(color_mode == 2)
    {
        c.z = float(value & uint(0x000000FF))/255.;
        c.y = float((value & uint(0x0000FF00)) >> 8)/255.;
        c.x = float((value & uint(0x00FF0000)) >> 16)/255.;
    }
    else if(color_mode == 3)
    {
        uint intensity = value >> 24;
        c = getRainbowColor(intensity);
    }
    else
    {
        c.z = float( uint(flat_rgb) & uint(0x000000FF))/255.;
        c.y = float((uint(flat_rgb) & uint(0x0000FF00)) >> 8)/255.;
        c.x = float((uint(flat_rgb) & uint(0x00FF0000)) >> 16)/255.;
    }
    color = vec4(c, alpha);
}
"""

fragment_shader = """
#version 330 core

uniform int point_type;

in vec4 color;

out vec4 finalColor;

void main()
{
    // only do this when point_type is sphere
    if (point_type == 2)
    {
        vec2 coord = gl_PointCoord * 2.0 - vec2(1.0); // Map [0,1] to [-1,1]
        float distance = dot(coord, coord); // Squared distance

        // Discard fragments outside the circle (radius = 1.0)
        if (distance > 1.0)
            discard;
    }

    finalColor = color;
}
"""


# draw points with color (x, y, z, color)
class CloudItem(BaseItem):
    def __init__(self, size, alpha, color_mode='I', color='#ffffff'):
        super().__init__()
        self.STRIDE = 16  # stride of cloud array
        self.valid_buff_top = 0
        self.add_buff_loc = 0
        self.alpha = alpha
        self.size = size
        self.mutex = threading.Lock()
        self.data_type = [('xyz', '<f4', (3,)), ('color', '<u4')]
        self.flat_rgb = int(color[1:], 16)
        self.mode_table = {'FLAT': 0,  'I': 1,  'RGB': 2,  'IRGB': 3}
        self.color_mode = self.mode_table[color_mode]
        self.CAPACITY = 10000000  # 10MB * 3 (x,y,z, color) * 4
        self.vmin = 0
        self.vmax = 255
        self.point_type = 0
        self.buff = np.empty((0), self.data_type)
        self.wait_add_data = None
        self.need_update_setting = True
        # Enable depth test when full opaque
        self.depth_test_enabled = (alpha == 1)

    def addSetting(self, layout):
        label_ptype = QLabel("Set point display type:")
        layout.addWidget(label_ptype)
        combo_ptype = QComboBox()
        combo_ptype.addItem("pixels")
        combo_ptype.addItem("flat squares")
        combo_ptype.addItem("spheres")
        combo_ptype.currentIndexChanged.connect(self._onPTypeSelection)
        layout.addWidget(combo_ptype)
        self.label_size = QLabel("Set size: (pixel)")
        layout.addWidget(self.label_size)
        self.box_size = QDoubleSpinBox()
        self.box_size.setSingleStep(1)
        self.box_size.setDecimals(0)
        layout.addWidget(self.box_size)
        self.box_size.setValue(self.size)
        self.box_size.valueChanged.connect(self.setSize)
        self.box_size.setRange(0, 100)

        label_alpha = QLabel("Set Alpha:")
        layout.addWidget(label_alpha)
        box_alpha = QDoubleSpinBox()
        layout.addWidget(box_alpha)
        box_alpha.setSingleStep(0.01)
        box_alpha.setValue(self.alpha)
        box_alpha.valueChanged.connect(self.setAlpha)
        box_alpha.setRange(0, 1)

        label_color = QLabel("Set ColorMode:")
        layout.addWidget(label_color)
        self.combo_color = QComboBox()
        self.combo_color.addItem("flat color")
        self.combo_color.addItem("intensity")
        self.combo_color.addItem("RGB")
        self.combo_color.addItem("intensity for RGB data")
        self.combo_color.currentIndexChanged.connect(self._onColorMode)
        layout.addWidget(self.combo_color)

        self.edit_rgb = QLineEdit()
        self.edit_rgb.setToolTip("Hex number, i.e. #FF4500")
        self.edit_rgb.setText(f"#{self.flat_rgb:06x}")
        self.edit_rgb.textChanged.connect(self._onColor)
        regex = QRegularExpression(r"^#[0-9A-Fa-f]{6}$")
        validator = QRegularExpressionValidator(regex)
        self.edit_rgb.setValidator(validator)
        layout.addWidget(self.edit_rgb)

        self.slider_v = RangeSlider()
        self.slider_v.setRange(0, 255)
        self.slider_v.rangeChanged.connect(self._onRangeV)

        layout.addWidget(self.slider_v)
        self.combo_color.setCurrentIndex(self.color_mode)

        # Add a checkbox for enabling/disabling depth test
        self.checkbox_depth_test = QCheckBox(
            "Show Front Points First (Enable Depth Test)")
        self.checkbox_depth_test.setChecked(self.depth_test_enabled)
        self.checkbox_depth_test.stateChanged.connect(self.setDepthTest)
        layout.addWidget(self.checkbox_depth_test)

    def _onRangeV(self, lower, upper):
        self.vmin = lower
        self.vmax = upper
        self.need_update_setting = True

    def _onColorMode(self, index):
        self.color_mode = index
        self.edit_rgb.hide()
        self.slider_v.hide()
        if (index == self.mode_table['FLAT']):  # flat color
            self.edit_rgb.show()
        elif (index == self.mode_table['IRGB']):  # flat color
            self.slider_v.show()
        elif (index == self.mode_table['I']):  # flat color
            self.slider_v.show()
        self.need_update_setting = True

    def setColorMode(self, color_mode):
        if color_mode in {'FLAT', 'RGB', 'IRGB', 'I'}:
            self.combo_color.setCurrentIndex(self.mode_table[color_mode])
        else:
            print(f"Invalid color mode: {color_mode}")

    def _onPTypeSelection(self, index):
        self.point_type = index
        if (index == 0):
            self.label_size.setText("Set size: (pixel)")
            self.box_size.setDecimals(0)
            self.box_size.setSingleStep(1)
            self.box_size.setValue(1)
            self.size = 1
        else:
            self.label_size.setText("Set size: (meter)")
            self.box_size.setDecimals(2)
            self.box_size.setSingleStep(0.01)
            self.box_size.setValue(0.01)
            self.size = 0.01
        self.need_update_setting = True

    def setAlpha(self, alpha):
        self.alpha = alpha
        self.need_update_setting = True

    def setFlatRGB(self, color):
        try:
            self.edit_rgb.setText(color)
        except ValueError:
            pass

    def _onColor(self, color):
        try:
            flat_rgb = int(color[1:], 16)
            self.flat_rgb = flat_rgb
            self.need_update_setting = True
        except ValueError:
            pass

    def setSize(self, size):
        self.size = size
        self.need_update_setting = True

    def setDepthTest(self, state):
        self.depth_test_enabled = (state == QtCore.Qt.Checked)

    def clear(self):
        data = np.empty((0), self.data_type)
        self.setData(data)

    def setData(self, data, append=False):
        if not isinstance(data, np.ndarray):
            raise ValueError("Input data must be a numpy array.")

        if data.dtype in {np.dtype('float32'), np.dtype('float64')}:
            xyz = data[:, :3]
            color = data[:, 3].view(np.uint32) if data.shape[1] == 4 else \
                np.zeros(data.shape[0], dtype=np.uint32)
            data = np.rec.fromarrays(
                [xyz, color[:data.shape[0]]], dtype=self.data_type)

        with self.mutex:
            if append:
                if self.wait_add_data is None:
                    self.wait_add_data = data
                else:
                    self.wait_add_data = np.concatenate(
                        [self.wait_add_data, data])
                self.add_buff_loc = self.valid_buff_top
            else:
                self.wait_add_data = data
                self.add_buff_loc = 0

            while self.add_buff_loc + self.wait_add_data.shape[0] > self.max_cloud_size:
                # Randomly select half of the points
                print(
                    "Warning: Buffer size exceeds the maximum cloud size. Randomly selecting half of the points.")
                indices = np.random.choice(
                    self.add_buff_loc, self.add_buff_loc // 2, replace=False)
                self.buff[:self.add_buff_loc // 2] = self.buff[indices]
                self.add_buff_loc //= 2
                indices = np.random.choice(
                    self.wait_add_data.shape[0], self.wait_add_data.shape[0] // 2, replace=False)
                self.wait_add_data = self.wait_add_data[indices]

    def updateSetting(self):
        if (self.need_update_setting is False):
            return
        glUseProgram(self.program)
        set_uniform(self.program, int(self.flat_rgb), 'flat_rgb')
        set_uniform(self.program, int(self.color_mode), 'color_mode')
        set_uniform(self.program, float(self.vmax), 'vmax')
        set_uniform(self.program, float(self.vmin), 'vmin')
        set_uniform(self.program, float(self.alpha), 'alpha')
        set_uniform(self.program, float(self.size), 'point_size')
        set_uniform(self.program, int(self.point_type), 'point_type')
        glUseProgram(0)
        self.need_update_setting = False

    def updateRenderBuffer(self):
        # Ensure there is data waiting to be added to the buffer
        if (self.wait_add_data is None):
            return
        # Acquire lock to update the buffer safely
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
            glBufferSubData(GL_ARRAY_BUFFER, self.add_buff_loc * self.STRIDE,
                            self.wait_add_data.shape[0] * self.STRIDE,
                            self.wait_add_data)
        self.valid_buff_top = new_buff_top
        self.wait_add_data = None
        self.mutex.release()

    def initializeGL(self):
        self.program = shaders.compileProgram(
            shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER),
        )
        self.max_cloud_size = glGetIntegerv(
            GL_MAX_SHADER_STORAGE_BLOCK_SIZE) // self.STRIDE
        # Bind attribute locations
        self.vbo = glGenBuffers(1)

    def paint(self):
        self.updateRenderBuffer()
        self.updateSetting()
        glEnable(GL_BLEND)
        glEnable(GL_PROGRAM_POINT_SIZE)
        glEnable(GL_POINT_SPRITE)
        if self.depth_test_enabled:
            glEnable(GL_DEPTH_TEST)
        else:
            glDisable(GL_DEPTH_TEST)

        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glUseProgram(self.program)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              self.STRIDE, ctypes.c_void_p(0))
        glVertexAttribPointer(
            1, 1, GL_FLOAT, GL_UNSIGNED_INT, self.STRIDE, ctypes.c_void_p(12))
        glEnableVertexAttribArray(0)
        glEnableVertexAttribArray(1)

        view_matrix = self.view().viewMatrix()
        set_uniform(self.program, view_matrix, 'view_matrix')
        project_matrix = self.view().projectionMatrix()
        set_uniform(self.program, project_matrix, 'projection_matrix')
        width = self.view().deviceWidth()
        focal = project_matrix[0, 0] * width / 2
        set_uniform(self.program, float(focal), 'focal')

        glDrawArrays(GL_POINTS, 0, self.valid_buff_top)

        # unbind VBO
        glDisableVertexAttribArray(0)
        glDisableVertexAttribArray(1)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glUseProgram(0)
        glDisable(GL_POINT_SPRITE)
        glDisable(GL_PROGRAM_POINT_SIZE)
        glDisable(GL_BLEND)
        if self.depth_test_enabled:
            glDisable(GL_DEPTH_TEST)  # Disable depth testing if it was enabled
