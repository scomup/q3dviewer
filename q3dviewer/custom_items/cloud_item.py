"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""


import numpy as np
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
import threading
import os
from PySide6.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, \
    QComboBox, QCheckBox
from OpenGL.GL import shaders
from q3dviewer.utils import *
from q3dviewer.utils.range_slider import RangeSlider
from PySide6.QtCore import QRegularExpression
from PySide6.QtGui import QRegularExpressionValidator


# draw points with color (x, y, z, color)
class CloudItem(BaseItem):
    def __init__(self, size, alpha, 
                 color_mode='I', 
                 color='#ffffff', 
                 point_type='PIXEL', 
                 depth_test=False):
        super().__init__()
        self.STRIDE = 16  # stride of cloud array
        self.valid_buff_top = 0
        self.add_buff_loc = 0
        self.alpha = alpha
        self.size = size
        self.point_type = point_type
        self.mutex = threading.Lock()
        self.data_type = [('xyz', '<f4', (3,)), ('irgb', '<u4')]
        self.flat_rgb = int(color[1:], 16)
        self.mode_table = {'FLAT': 0,  'I': 1,  'RGB': 2}
        self.point_type_table = {'PIXEL': 0, 'SQUARE': 1, 'SPHERE': 2}
        self.color_mode = self.mode_table[color_mode]
        self.CAPACITY = 10000000  # 10MB * 3 (x,y,z, color) * 4
        self.vmin = 0
        self.vmax = 255
        self.buff = np.empty((0), self.data_type)
        self.wait_add_data = None
        self.need_update_setting = True
        self.max_cloud_size = 300000000
        # Enable depth test when full opaque
        self.depth_test = depth_test
        self.path = os.path.dirname(__file__)

    def add_setting(self, layout):
        label_ptype = QLabel("Point Type:")
        layout.addWidget(label_ptype)
        combo_ptype = QComboBox()
        combo_ptype.addItem("pixels")
        combo_ptype.addItem("flat squares")
        combo_ptype.addItem("spheres")
        combo_ptype.setCurrentIndex(self.point_type_table[self.point_type])
        combo_ptype.currentIndexChanged.connect(self._on_point_type_selection)
        layout.addWidget(combo_ptype)

        self.box_size = QDoubleSpinBox()
        self.box_size.setPrefix("Size: ")
        self.box_size.setSingleStep(1)
        self.box_size.setDecimals(0)
        self.box_size.setValue(self.size)
        self.box_size.setRange(0, 100)
        self.box_size.valueChanged.connect(self.set_size)
        self._on_point_type_selection(self.point_type_table[self.point_type])
        layout.addWidget(self.box_size)

        box_alpha = QDoubleSpinBox()
        box_alpha.setPrefix("Alpha: ")
        box_alpha.setSingleStep(0.01)
        box_alpha.setValue(self.alpha)
        box_alpha.setRange(0, 1)
        box_alpha.valueChanged.connect(self.set_alpha)
        layout.addWidget(box_alpha)

        label_color = QLabel("Color Mode:")
        layout.addWidget(label_color)
        self.combo_color = QComboBox()
        self.combo_color.addItem("flat color")
        self.combo_color.addItem("intensity")
        self.combo_color.addItem("RGB")
        self.combo_color.setCurrentIndex(self.color_mode)
        self.combo_color.currentIndexChanged.connect(self._on_color_mode)
        layout.addWidget(self.combo_color)

        label_rgb = QLabel("Color:")
        layout.addWidget(label_rgb)
        self.edit_rgb = QLineEdit()
        self.edit_rgb.setToolTip("Hex number, i.e. #FF4500")
        self.edit_rgb.setText(f"#{self.flat_rgb:06x}")
        self.edit_rgb.textChanged.connect(self._on_color)
        regex = QRegularExpression(r"^#[0-9A-Fa-f]{6}$")
        validator = QRegularExpressionValidator(regex)
        self.edit_rgb.setValidator(validator)
        layout.addWidget(self.edit_rgb)

        self.slider_v = RangeSlider()
        self.slider_v.setRange(0, 255)
        self.slider_v.rangeChanged.connect(self._on_range)
        layout.addWidget(self.slider_v)

        self.checkbox_depth_test = QCheckBox(
            "Show front points first (Depth Test)")
        self.checkbox_depth_test.setChecked(self.depth_test)
        self.checkbox_depth_test.stateChanged.connect(self.set_depthtest)
        layout.addWidget(self.checkbox_depth_test)

    def _on_range(self, lower, upper):
        self.vmin = lower
        self.vmax = upper
        self.need_update_setting = True

    def _on_color_mode(self, index):
        self.color_mode = index
        self.edit_rgb.hide()
        self.slider_v.hide()
        if (index == self.mode_table['FLAT']):  # flat color
            self.edit_rgb.show()
        elif (index == self.mode_table['I']):  # flat color
            self.slider_v.show()
        self.need_update_setting = True

    def set_color_mode(self, color_mode):
        if color_mode in {'FLAT', 'RGB', 'I'}:
            try:
                self.combo_color.setCurrentIndex(self.mode_table[color_mode])
            except RuntimeError:
                pass
            except ValueError:
                pass
        else:
            print(f"Invalid color mode: {color_mode}")

    def _on_point_type_selection(self, index):
        self.point_type = list(self.point_type_table.keys())[index]
        if self.point_type == 'PIXEL':
            self.box_size.setPrefix("Set size (pixel): ")
            self.box_size.setDecimals(0)
            self.box_size.setSingleStep(1)
            self.box_size.setValue(1)
            self.size = 1
        else:
            self.box_size.setPrefix("Set size (meter): ")
            self.box_size.setDecimals(2)
            self.box_size.setSingleStep(0.01)
            self.box_size.setValue(0.01)
            self.size = 0.01
        self.need_update_setting = True

    def set_alpha(self, alpha):
        self.alpha = alpha
        self.need_update_setting = True

    def set_flat_rgb(self, color):
        try:
            self.edit_rgb.setText(color)
        except ValueError:
            pass

    def _on_color(self, color):
        try:
            flat_rgb = int(color[1:], 16)
            self.flat_rgb = flat_rgb
            self.need_update_setting = True
        except ValueError:
            pass

    def set_size(self, size):
        self.size = size
        self.need_update_setting = True

    def set_depthtest(self, state):
        self.depth_test = state

    def clear(self):
        data = np.empty((0), self.data_type)
        self.set_data(data)

    def set_data(self, data, append=False):
        if not isinstance(data, np.ndarray):
            raise ValueError("Input data must be a numpy array.")

        if data.dtype in {np.dtype('float32'), np.dtype('float64')}:
            if data.size == 0:
                data = np.empty((0), self.data_type)
            elif data.ndim == 2 and data.shape[1] >= 3:
                xyz = data[:, :3]
                if data.shape[1] >= 4:
                    color = data[:, 3].view(np.uint32)
                else:
                    color = np.zeros(data.shape[0], dtype=np.uint32)
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


    def update_setting(self):
        if (self.need_update_setting is False):
            return
        glUseProgram(self.program)
        set_uniform(self.program, int(self.flat_rgb), 'flat_rgb')
        set_uniform(self.program, int(self.color_mode), 'color_mode')
        set_uniform(self.program, float(self.vmax), 'vmax')
        set_uniform(self.program, float(self.vmin), 'vmin')
        set_uniform(self.program, float(self.alpha), 'alpha')
        set_uniform(self.program, float(self.size), 'point_size')
        set_uniform(self.program, int(self.point_type_table[self.point_type]), 'point_type')
        glUseProgram(0)
        self.need_update_setting = False

    def update_render_buffer(self):
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
            print("[Cloud Item] Update capacity to %d" % buff_capacity)
            new_buff = np.empty((buff_capacity), self.data_type)
            new_buff[:self.add_buff_loc] = self.buff[:self.add_buff_loc]
            new_buff[self.add_buff_loc:new_buff_top] = self.wait_add_data
            self.buff = new_buff

            # if exceed the maximum cloud size, randomly select half of the points
            exceed_flag = False
            while new_buff.shape[0] > self.max_cloud_size:
                exceed_flag = True
                new_buff_half = new_buff[:new_buff_top:2]
                new_buff_top = new_buff_half.shape[0]
                new_buff = new_buff_half
            if exceed_flag:
                print("[Cloud Item] Exceed maximum cloud size %d, reduce the data size" % self.max_cloud_size)
                self.buff = self.buff[:self.max_cloud_size]
                self.buff[:new_buff_top] = new_buff


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

    def initialize_gl(self):
        vertex_shader = open(self.path + '/../shaders/cloud_vert.glsl', 'r').read()
        fragment_shader = open(self.path + '/../shaders/cloud_frag.glsl', 'r').read()
        self.program = shaders.compileProgram(
            shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER),
        )
        self.max_cloud_size = glGetIntegerv(
            GL_MAX_SHADER_STORAGE_BLOCK_SIZE) // self.STRIDE
        # Bind attribute locations
        self.vbo = glGenBuffers(1)

    def paint(self):
        self.update_render_buffer()
        self.update_setting()
        glEnable(GL_BLEND)
        glEnable(GL_PROGRAM_POINT_SIZE)
        glEnable(GL_POINT_SPRITE)
        if self.depth_test:
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

        view_matrix = self.glwidget().view_matrix
        set_uniform(self.program, view_matrix, 'view_matrix')
        project_matrix = self.glwidget().projection_matrix
        set_uniform(self.program, project_matrix, 'projection_matrix')
        width = self.glwidget().current_width()
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
        if self.depth_test:
            glDisable(GL_DEPTH_TEST)  # Disable depth testing if it was enabled
