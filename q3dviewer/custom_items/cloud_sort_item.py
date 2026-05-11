"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.custom_items.cloud_io_item import CloudIOItem
from pathlib import Path
import os
from q3dviewer.Qt.QtWidgets import QSpinBox, QCheckBox, QSlider, QHBoxLayout, QLabel
from q3dviewer.cuda_sort import CUDAPointSorter
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

        # CUDA depth sorting (optional)
        self.cuda_sorter = None
        self.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])
        self.use_depth_sorting = False

        # Try to initialize CUDA sorter
        self.cuda_sorter = CUDAPointSorter()
        if not self.cuda_sorter.is_available():
            print("[CloudSortItem] CUDA not available, depth sorting disabled")
            self.cuda_sorter = None
            self.use_depth_sorting = False
        else:
            # Only enable depth sorting if requested and CUDA is available
            self.use_depth_sorting = use_depth_sorting
            if use_depth_sorting:
                print("[CloudSortItem] CUDA available, depth sorting enabled")

    def add_setting(self, layout):
        super().add_setting(layout)

        # GPU depth sorting checkbox
        self.check_depth_sort = QCheckBox("GPU Depth Sorting (CUDA)")
        self.check_depth_sort.setChecked(self.use_depth_sorting)
        self.check_depth_sort.stateChanged.connect(self._on_depth_sorting)
        layout.addWidget(self.check_depth_sort)

        if self.cuda_sorter is None:
            self.check_depth_sort.setEnabled(False)
            self.check_depth_sort.setToolTip(
                "CUDA not available, cannot enable depth sorting")

    def _on_depth_sorting(self, state):
        """Toggle depth sorting on/off."""
        new_state = (state != 0)
        if new_state and not self.cuda_sorter:
            print("[CloudSortItem] CUDA not available, cannot enable depth sorting")
            self.check_depth_sort.setChecked(False)
            return
        self.use_depth_sorting = new_state
        # Clear cache when toggling to force re-sort
        self.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])

    def __del__(self):
        if self.cuda_sorter is not None:
            try:
                self.cuda_sorter.unregister()
            except:
                pass

    def update_render_buffer(self):
        if self.wait_add_data is None:
            return
        self.mutex.acquire()
        try:
            new_data = self.wait_add_data
            self.wait_add_data = None
            new_count = new_data.shape[0]
            new_buff_top = self.add_buff_loc + new_count

            # Hard-cap at max_cloud_size — avoids expensive CPU readback
            if new_buff_top > self.max_cloud_size:
                new_buff_top = self.max_cloud_size
                new_count = new_buff_top - self.add_buff_loc
                if new_count <= 0:
                    print("[Cloud Item] Max cloud size reached, ignoring data.")
                    return
                new_data = new_data[:new_count]
                print("[Cloud Item] Truncated to max cloud size %d" % self.max_cloud_size)

            vbo_reallocated = False
            if new_buff_top > self.buff_capacity:
                # Grow GPU buffer — pure GPU-to-GPU, zero CPU roundtrip
                new_capacity = max(self.buff_capacity, self.CAPACITY)
                while new_buff_top > new_capacity:
                    new_capacity += self.CAPACITY
                new_capacity = min(new_capacity, self.max_cloud_size)

                new_vbo = glGenBuffers(1)
                glBindBuffer(GL_ARRAY_BUFFER, new_vbo)
                glBufferData(GL_ARRAY_BUFFER, new_capacity * self.STRIDE,
                             None, GL_DYNAMIC_DRAW)
                glBindBuffer(GL_ARRAY_BUFFER, 0)

                # GPU-to-GPU copy of existing valid data
                if self.add_buff_loc > 0:
                    glBindBuffer(GL_COPY_READ_BUFFER, self.vbo)
                    glBindBuffer(GL_COPY_WRITE_BUFFER, new_vbo)
                    glCopyBufferSubData(
                        GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER,
                        0, 0, self.add_buff_loc * self.STRIDE)
                    glBindBuffer(GL_COPY_READ_BUFFER, 0)
                    glBindBuffer(GL_COPY_WRITE_BUFFER, 0)

                glDeleteBuffers(1, [self.vbo])
                self.vbo = new_vbo
                self.buff_capacity = new_capacity
                vbo_reallocated = True

            # Upload only the new slice — O(new_data), not O(total)
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferSubData(GL_ARRAY_BUFFER,
                            self.add_buff_loc * self.STRIDE,
                            new_count * self.STRIDE,
                            new_data)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            self.valid_buff_top = new_buff_top

            # CUDA sorter must be unregistered only when VBO handle changes
            if vbo_reallocated and self.cuda_sorter:
                self.cuda_sorter.unregister()
                self.last_depth_coeffs = np.array([np.inf, np.inf, np.inf])
        finally:
            self.mutex.release()

    def _perform_depth_sort(self, view_matrix):
        """Execute depth sorting on VBO using CUDA."""
        if self.valid_buff_top == 0:
            return

        if self.cuda_sorter is None:
            return

        if not self.cuda_sorter.is_registered():
            # Register VBO with CUDA
            self.cuda_sorter.register(
                int(self.vbo),
                self.valid_buff_top
            )

        depth_coeffs = (view_matrix @ self.T)[2, :3]

        # Check if sorting is needed based on camera rotation
        rotation_diff = np.abs(depth_coeffs - self.last_depth_coeffs).max()
        if rotation_diff < 0.1:
            return

        # Perform CUDA sorting (reorders VBO data in-place)
        self.cuda_sorter.sort_by_depth(
            depth_coeffs,
            self.valid_buff_top
        )

        # Update cache
        self.last_depth_coeffs = depth_coeffs.copy()

    def paint(self):
        self.update_render_buffer()
        self.update_setting()

        # Perform depth sorting if enabled
        if self.use_depth_sorting:
            view_matrix = self.glwidget().view_matrix
            self._perform_depth_sort(view_matrix)

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
