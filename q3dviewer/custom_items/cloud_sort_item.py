"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.custom_items.cloud_io_item import CloudIOItem
from pathlib import Path
import os
from q3dviewer.Qt.QtWidgets import QSpinBox, QCheckBox, QSlider, QHBoxLayout, QLabel
from q3dviewer.point_sort import PointSorter
from q3dviewer.utils import set_uniform
from OpenGL.GL import shaders
from OpenGL.GL import *
import numpy as np
from q3dviewer.Qt.QtWidgets import QPushButton, QLabel, QLineEdit, QMessageBox


class CloudSortItem(CloudIOItem):
    """
    A OpenGL point cloud item with input/output capabilities.
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

    def __init__(self, **kwargs):
        # Extract use_depth_sorting from kwargs before passing to parent
        use_depth_sorting = kwargs.pop('use_depth_sorting', False)

        # Initialize parent class first
        super().__init__(**kwargs)

        # Depth sorting (always available: CUDA or CPU fallback)
        self.sorter = PointSorter()
        self.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])
        self.use_depth_sorting = use_depth_sorting
        self._force_sort_once = False  # Flag for manual sort trigger

        if use_depth_sorting:
            mode = "CUDA GPU" if self.sorter.is_using_cuda() else "CPU"
            print(f"[CloudSortItem] Depth sorting enabled ({mode} mode)")

    def add_setting(self, layout):
        super().add_setting(layout)

        # Sorting mode label
        mode_text = "CUDA GPU" if self.sorter.is_using_cuda() else "CPU"
        label_mode = QLabel(f"Sorting Mode: {mode_text}")
        label_mode.setStyleSheet("color: gray; font-style: italic;")
        layout.addWidget(label_mode)

        # Depth sorting checkbox
        self.check_depth_sort = QCheckBox("Enable Depth Sorting")
        self.check_depth_sort.setChecked(self.use_depth_sorting)
        self.check_depth_sort.stateChanged.connect(self._on_depth_sorting)
        layout.addWidget(self.check_depth_sort)

    def _on_depth_sorting(self, state):
        """Toggle depth sorting on/off."""
        self.use_depth_sorting = (state != 0)
        # Clear cache when toggling to force re-sort
        self.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])

    def __del__(self):
        try:
            self.sorter.unregister()
        except:
            pass

    def update_render_buffer(self):
        if self.wait_add_data is None:
            return
        self.mutex.acquire()
        try:
            new_data = self.wait_add_data
            self.wait_add_data = None

            while new_data.shape[0] > self.max_cloud_size * 0.9:
                new_data = new_data[::2]
                print(f"[Cloud Item] new data downsampled to {new_data.shape[0]} points")

            # Phase 1 — grow buffer on demand up to max_cloud_size.
            # glDeleteBuffers only happens here (growth phase), never in steady-state.
            new_buff_top = self.add_buff_loc + new_data.shape[0]
            vbo_reallocated = False  # [sort]
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
                vbo_reallocated = True  # [sort]

            # Phase 2 — GPU downsample until there is room (only at max capacity).
            while self.add_buff_loc + new_data.shape[0] > self.buff_capacity:
                self._gpu_downsample()
                self.sorter.unregister()  # [sort]
                self.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])  # [sort]

            # Upload new slice
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferSubData(GL_ARRAY_BUFFER,
                            self.add_buff_loc * self.STRIDE,
                            new_data.shape[0] * self.STRIDE,
                            new_data)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            self.valid_buff_top = self.add_buff_loc + new_data.shape[0]

            if vbo_reallocated:  # [sort] VBO handle changed — sorter must re-register.
                self.sorter.unregister()
                self.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])
        finally:
            self.mutex.release()

    def force_sort(self):
        self._force_sort_once = True
        mode = "CUDA GPU" if self.sorter.is_using_cuda() else "CPU"
        print(f"[CloudSortItem] Force sort by {mode}")

    def _perform_depth_sort(self, view_matrix, force=False):
        """Execute depth sorting on VBO (CUDA or CPU)."""
        if self.valid_buff_top == 0:
            return

        if not self.sorter.is_registered():
            # Register with current buff_capacity (= actual buffer size).
            # During growth this is re-registered after each vbo_reallocated.
            # After reaching max_cloud_size, buff_capacity is stable.
            self.sorter.register(
                int(self.vbo),
                self.buff_capacity
            )

        depth_coeffs = (view_matrix @ self.T)[2, :3]

        # Check if sorting is needed based on camera rotation
        rotation_diff = np.abs(depth_coeffs - self.last_depth_coeffs).max()
        if rotation_diff < 0.1 and not force:
            return

        # Perform sorting (CUDA or CPU, reorders VBO data in-place)
        self.sorter.sort_by_depth(
            depth_coeffs,
            self.valid_buff_top
        )

        # Update cache
        self.last_depth_coeffs = depth_coeffs.copy()

    def paint(self):
        self.update_render_buffer()
        self.update_setting()

        # Perform depth sorting if enabled or force requested
        if self.use_depth_sorting or self._force_sort_once:
            view_matrix = self.glwidget().view_matrix
            self._perform_depth_sort(view_matrix, force=self._force_sort_once)
            self._force_sort_once = False

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
        glVertexAttribIPointer(
            1, 1, GL_UNSIGNED_INT, self.STRIDE, ctypes.c_void_p(12))
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
