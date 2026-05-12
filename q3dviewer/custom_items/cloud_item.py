"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""


import numpy as np
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
from OpenGL.GL import shaders

import threading
import os
from q3dviewer.Qt.QtCore import Qt
from q3dviewer.Qt.QtWidgets import QLabel, QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox, QCheckBox, QSlider, QHBoxLayout
from q3dviewer.utils.range_slider import RangeSlider
from q3dviewer.utils import set_uniform
from q3dviewer.utils import text_to_rgba
from q3dviewer.Qt import Q3D_DEBUG


# draw points with color (x, y, z, color)
class CloudItem(BaseItem):
    """
    A OpenGL point cloud item.

    Attributes:
        size (float): The size of each point. When `point_type` is 'PIXEL', this is the size in screen pixels.
            When `point_type` is 'SQUARE' or 'SPHERE', this is the size in centimeters in 3D space.
        alpha (float): The transparency of the points, in the range [0, 1], where 0 is fully transparent and 1 is fully opaque.
        color_mode (str): The coloring mode for the points.
            - 'FLAT': Single flat color for all points (uses the `color` attribute).
            - 'I': Color by intensity.
            - 'RGB': Per-point RGB color.
            - 'GRAY': Per-point grayscale color.
        color (str or tuple): The flat color to use when `color_mode` is 'FLAT'. Accepts any valid matplotlib color (e.g., 'red', '#FF4500', (1.0, 0.5, 0.0)).
        point_type (str): The type/rendering style of each point:
            - 'PIXEL': Draw each point as a square pixel on the screen.
            - 'SQUARE': Draw each point as a square in 3D space.
            - 'SPHERE': Draw each point as a sphere in 3D space.
        depth_test (bool): Whether to enable depth testing. If True, points closer to the camera will appear in front of farther ones.
    """
    # Class-level constants
    STRIDE = 16  # Stride of cloud array in bytes
    CAPACITY = 10000000  # Growth increment (10M points = 160 MB)
    DATA_TYPE = [('xyz', '<f4', (3,)), ('irgb', '<u4')]
    MODE_TABLE = {'FLAT': 0, 'I': 1, 'RGB': 2, 'GRAY': 3}
    POINT_TYPE_TABLE = {'PIXEL': 0, 'SQUARE': 1, 'SPHERE': 2}

    def __init__(self, size, alpha,
                 color_mode='I',
                 color='white',
                 point_type='PIXEL'):
        super().__init__()
        self.valid_buff_top = 0  # the loc in buffer of the top of valid data
        self.add_buff_loc = 0    # the loc in buffer to add new data
        self.buff_capacity = 0   # the size of GPU buffer in number of points
        self.alpha = alpha
        self.size = size
        self.point_type = point_type
        self.mutex = threading.Lock()
        self.color = color
        try:
            self.flat_rgb = text_to_rgba(color, flat=True)
        except ValueError:
            print(
                f"Invalid color: {color}, please use matplotlib color format")
            exit(1)
        self.color_mode = self.MODE_TABLE[color_mode]
        self.vmin = 0
        self.vmax = 255
        self.wait_add_data = None
        self.need_update_setting = True
        # Default 10M points (160 MB). Set item.max_cloud_size before adding
        # to viewer to override. Pre-allocated once at initialize_gl time.
        self.max_cloud_size = 10000000
        self.downsample_count = 0   # how many GPU downsample passes have run
        self.downsample_program = None
        self.T = np.eye(4, dtype=np.float32)
        # Enable depth test when full opaque
        self.path = os.path.dirname(__file__)

    def add_setting(self, layout):
        label_ptype = QLabel("Point Type:")
        layout.addWidget(label_ptype)
        combo_ptype = QComboBox()
        combo_ptype.addItem("pixels")
        combo_ptype.addItem("flat squares")
        combo_ptype.addItem("spheres")
        combo_ptype.setCurrentIndex(self.POINT_TYPE_TABLE[self.point_type])
        combo_ptype.currentIndexChanged.connect(self._on_point_type_selection)
        layout.addWidget(combo_ptype)

        self.box_size = QSpinBox()
        self.box_size.setPrefix("Size: ")
        self.box_size.setSingleStep(1)
        self.box_size.setValue(int(self.size))
        self.box_size.setRange(0, 100)
        self.box_size.valueChanged.connect(self.set_size)
        self._on_point_type_selection(self.POINT_TYPE_TABLE[self.point_type])
        layout.addWidget(self.box_size)

        alpha_layout = QHBoxLayout()
        alpha_label = QLabel("Alpha:")
        alpha_layout.addWidget(alpha_label)
        self.alpha_slider = QSlider()
        self.alpha_slider.setOrientation(Qt.Horizontal)
        self.alpha_slider.setRange(0, 100)
        self.alpha_slider.setValue(int(self.alpha * 100))
        self.alpha_slider.valueChanged.connect(
            lambda v: self.set_alpha(v / 100.0))
        alpha_layout.addWidget(self.alpha_slider)
        layout.addLayout(alpha_layout)

        label_color = QLabel("Color Mode:")
        layout.addWidget(label_color)
        self.combo_color = QComboBox()
        self.combo_color.addItem("flat color")
        self.combo_color.addItem("intensity")
        self.combo_color.addItem("RGB")
        self.combo_color.addItem("gray")
        self.combo_color.setCurrentIndex(self.color_mode)
        self.combo_color.currentIndexChanged.connect(self._on_color_mode)
        layout.addWidget(self.combo_color)

        label_rgb = QLabel("Color:")
        label_rgb.setToolTip(
            "Use hex color, i.e. #FF4500, or named color, i.e. 'red'")
        layout.addWidget(label_rgb)
        self.edit_rgb = QLineEdit()
        self.edit_rgb.setToolTip(
            "Use hex color, i.e. #FF4500, or named color, i.e. 'red'")
        self.edit_rgb.setText(self.color)
        self.edit_rgb.textChanged.connect(self._on_color)
        layout.addWidget(self.edit_rgb)

        self.slider_v = RangeSlider()
        self.slider_v.setRange(0, 255)
        self.slider_v.rangeChanged.connect(self._on_range)
        layout.addWidget(self.slider_v)

    def _on_range(self, lower, upper):
        self.vmin = lower
        self.vmax = upper
        self.need_update_setting = True

    def _on_color_mode(self, index):
        self.color_mode = index
        self.edit_rgb.hide()
        self.slider_v.hide()
        if (index == self.MODE_TABLE['FLAT']):  # flat color
            self.edit_rgb.show()
        elif (index == self.MODE_TABLE['I']):  # intensity
            self.slider_v.show()
        elif (index == self.MODE_TABLE['GRAY']):  # grayscale
            self.slider_v.show()

        self.need_update_setting = True

    def set_color_mode(self, color_mode):
        if color_mode in {'FLAT', 'RGB', 'I', 'GRAY'}:
            try:
                self.combo_color.setCurrentIndex(self.MODE_TABLE[color_mode])
            except:
                self.color_mode = self.MODE_TABLE[color_mode]
                self.need_update_setting = True
        else:
            print(f"Invalid color mode: {color_mode}")

    def _on_point_type_selection(self, index):
        self.point_type = list(self.POINT_TYPE_TABLE.keys())[index]
        if self.point_type == 'PIXEL':
            self.box_size.setPrefix("Set size (pixel): ")
        else:
            self.box_size.setPrefix("Set size (cm): ")
        # self.size = 1
        # self.box_size.setValue(self.size)
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
            self.flat_rgb = text_to_rgba(color, flat=True)
            self.need_update_setting = True
        except ValueError:
            print(
                f"Invalid color: {color}, please use matplotlib color format")

    def set_size(self, size):
        self.size = size
        self.need_update_setting = True

    def set_transform(self, transform):
        self.T = transform
        self.need_update_setting = True

    def clear(self):
        data = np.empty((0), self.DATA_TYPE)
        self.set_data(data)

    def set_data(self, data, append=False):
        if not isinstance(data, np.ndarray):
            raise ValueError("Input data must be a numpy array.")

        if data.dtype in {np.dtype('float32'), np.dtype('float64')}:
            if data.size == 0:
                data = np.empty((0), self.DATA_TYPE)
            elif data.ndim == 2 and data.shape[1] >= 3:
                xyz = data[:, :3]
                if data.shape[1] >= 4:
                    color = data[:, 3].view(np.uint32)
                else:
                    color = np.zeros(data.shape[0], dtype=np.uint32)
                data = np.rec.fromarrays(
                    [xyz, color[:data.shape[0]]], dtype=self.DATA_TYPE)

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
        set_uniform(self.program, int(self.size), 'point_size')
        set_uniform(self.program, int(
            self.POINT_TYPE_TABLE[self.point_type]), 'point_type')
        set_uniform(self.program, self.T, 'model_matrix')
        glUseProgram(0)
        self.need_update_setting = False

    def update_render_buffer(self):
        if self.wait_add_data is None:
            return
        self.mutex.acquire()
        try:
            new_data = self.wait_add_data
            self.wait_add_data = None
            new_count = new_data.shape[0]

            # Clamp batch to half max_cloud_size so one downsample always makes room.
            max_batch = self.max_cloud_size // 2
            if new_count > max_batch:
                new_data = new_data[:max_batch]
                new_count = max_batch
                print(f"[Cloud Item] Batch truncated to {max_batch} points (half of max)")

            # Phase 1 — grow buffer on demand up to max_cloud_size.
            # glDeleteBuffers only happens here (growth phase), never in steady-state.
            new_buff_top = self.add_buff_loc + new_count
            if new_buff_top > self.buff_capacity and self.buff_capacity < self.max_cloud_size:
                new_capacity = max(self.buff_capacity, self.CAPACITY)
                while new_buff_top > new_capacity:
                    new_capacity += self.CAPACITY
                new_capacity = min(new_capacity, self.max_cloud_size)

                new_vbo = glGenBuffers(1)
                glBindBuffer(GL_ARRAY_BUFFER, new_vbo)
                glBufferData(GL_ARRAY_BUFFER, new_capacity * self.STRIDE, None, GL_DYNAMIC_DRAW)
                glBindBuffer(GL_ARRAY_BUFFER, 0)
                if self.add_buff_loc > 0:
                    glBindBuffer(GL_COPY_READ_BUFFER, self.vbo)
                    glBindBuffer(GL_COPY_WRITE_BUFFER, new_vbo)
                    glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER,
                                        0, 0, self.add_buff_loc * self.STRIDE)
                    glBindBuffer(GL_COPY_READ_BUFFER, 0)
                    glBindBuffer(GL_COPY_WRITE_BUFFER, 0)
                glDeleteBuffers(1, [self.vbo])
                self.vbo = new_vbo
                self.buff_capacity = new_capacity

            # Phase 2 — GPU downsample until there is room (only at max capacity).
            while self.add_buff_loc + new_count > self.buff_capacity:
                self._gpu_downsample()

            # Upload new slice
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferSubData(GL_ARRAY_BUFFER,
                            self.add_buff_loc * self.STRIDE,
                            new_count * self.STRIDE,
                            new_data)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            self.valid_buff_top = self.add_buff_loc + new_count
        finally:
            self.mutex.release()

    def _gpu_downsample(self):
        """GPU-only in-place stride-2 decimation. No temp buffer needed:
        dst[i] = src[i*2], and i <= i*2 always, so no write overtakes its read."""
        half = self.valid_buff_top // 2
        if half == 0:
            return

        glUseProgram(self.downsample_program)
        # Single buffer binding — read from buf[i*2], write to buf[i] in-place.
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, self.vbo)
        glUniform1ui(
            glGetUniformLocation(self.downsample_program, 'num_dst_points'),
            half)
        groups = (half + 255) // 256
        glDispatchCompute(groups, 1, 1)
        glUseProgram(0)

        # Ensure in-place compute writes are visible to the vertex fetch stage.
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT
                        | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT)

        self.valid_buff_top = half
        self.add_buff_loc = half
        self.downsample_count += 1
        print(f"[CloudItem] GPU downsampled to {half} points (pass #{self.downsample_count})")

    def initialize_gl(self):
        vertex_shader = open(
            self.path + '/../shaders/cloud_vert.glsl', 'r').read()
        fragment_shader = open(
            self.path + '/../shaders/cloud_frag.glsl', 'r').read()
        self.program = shaders.compileProgram(
            shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER),
        )

        # Clamp user-set max_cloud_size by the GL driver's SSBO size limit.
        driver_max = glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE) // self.STRIDE
        if self.max_cloud_size > driver_max:
            print(f"[CloudItem] max_cloud_size clamped {self.max_cloud_size} → {driver_max} (GL driver limit)")
            self.max_cloud_size = driver_max

        # ── Main GPU buffer ──────────────────────────────────────────────────────
        # Start small; grows on demand (Phase 1) up to max_cloud_size,
        # then GPU-downsamples in-place (Phase 2). Multiple items are safe
        # because each only uses the memory its data actually needs.
        initial = min(self.CAPACITY, self.max_cloud_size)
        self.vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glBufferData(GL_ARRAY_BUFFER, initial * self.STRIDE, None, GL_DYNAMIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        self.buff_capacity = initial

        # ── Compute shader for GPU-only downsample ───────────────────────────────
        comp_src = open(
            self.path + '/../shaders/downsample_comp.glsl', 'r').read()
        comp = shaders.compileShader(comp_src, GL_COMPUTE_SHADER)
        self.downsample_program = shaders.compileProgram(comp)

    def paint(self):
        self.update_render_buffer()
        self.update_setting()

        glEnable(GL_BLEND)
        glEnable(GL_PROGRAM_POINT_SIZE)
        glEnable(GL_POINT_SPRITE)
        glEnable(GL_DEPTH_TEST)

        if self.alpha < 0.9:
            glDepthFunc(GL_ALWAYS)
        else:
            glDepthFunc(GL_LESS)

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
