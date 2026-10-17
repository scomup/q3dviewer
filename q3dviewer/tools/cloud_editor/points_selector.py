"""GPU point selection for rectangle and radius modes."""

from pathlib import Path

import numpy as np
from OpenGL.GL import *
from OpenGL.GL import shaders


class PointsSelector:
    """Select point indices using one GPU compute shader."""

    LOCAL_SIZE = 256
    RECTANGLE_MODE = 0
    RADIUS_MODE = 1
    UINT32_BYTES = np.dtype(np.uint32).itemsize

    def __init__(self, view_handler):
        self.view_handler = view_handler
        self._points = None
        self._program = None
        self._points_buffer = None
        self._indices_buffer = None
        self._count_buffer = None
        self._index_capacity = 0
        self._points_buffer_dirty = True

    def set_points(self, points):
        points = np.asarray(points, dtype=np.float32)
        if points.size == 0:
            self._points = np.empty((0, 3), dtype=np.float32)
            self._points_buffer_dirty = True
            return
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('points must have shape (N, 3)')
        self._points = np.ascontiguousarray(points)
        self._points_buffer_dirty = True
        self._ensure_resources()

    def select(self, mode, data):
        if self._points is None or self._points.shape[0] == 0:
            return np.empty(0, dtype=np.int64)

        self.view_handler.makeCurrent()
        try:
            self._ensure_resources()
            return self._dispatch(mode, data)
        finally:
            self.view_handler.doneCurrent()

    def _ensure_resources(self):
        self._ensure_program()
        self._ensure_buffers()
        self._upload_points_if_needed()
        self._ensure_index_buffer()

    def _ensure_program(self):
        if self._program is None:
            shader_path = (Path(__file__).resolve().parent.parent
                           .parent / 'shaders' / 'points_select_comp.glsl')
            source = shader_path.read_text(encoding='utf-8')
            compute_shader = shaders.compileShader(source, GL_COMPUTE_SHADER)
            self._program = shaders.compileProgram(compute_shader)

    def _ensure_buffers(self):
        if self._points_buffer is None:
            self._points_buffer = glGenBuffers(1)
        if self._indices_buffer is None:
            self._indices_buffer = glGenBuffers(1)
        if self._count_buffer is None:
            self._count_buffer = glGenBuffers(1)
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._count_buffer)
            glBufferData(
                GL_SHADER_STORAGE_BUFFER,
                self.UINT32_BYTES,
                None,
                GL_DYNAMIC_READ)

    def _upload_points_if_needed(self):
        if self._points_buffer_dirty:
            points = np.empty((self._points.shape[0], 4), dtype=np.float32)
            points[:, :3] = self._points
            points[:, 3] = 1.0
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._points_buffer)
            glBufferData(
                GL_SHADER_STORAGE_BUFFER, points.nbytes, points, GL_STATIC_DRAW)
            self._points_buffer_dirty = False

    def _ensure_index_buffer(self):
        point_count = self._points.shape[0]
        if self._index_capacity >= point_count:
            return
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._indices_buffer)
        glBufferData(
            GL_SHADER_STORAGE_BUFFER,
            point_count * self.UINT32_BYTES,
            None,
            GL_DYNAMIC_READ)
        self._index_capacity = point_count

    def _dispatch(self, mode, data):
        rectangle_ndc, center, radius = self._selection_parameters(mode, data)
        glUseProgram(self._program)
        try:
            self._set_uniforms(
                mode, rectangle_ndc, center, radius)
            self._bind_selection_buffers()
            self._run_compute()
            return self._read_selected_indices()
        finally:
            self._unbind_selection_buffers()

    def _selection_parameters(self, mode, data):
        if mode == self.RECTANGLE_MODE:
            return (
                self._rectangle_ndc(data),
                np.zeros(3, dtype=np.float32),
                0.0,
            )
        if mode == self.RADIUS_MODE:
            center, radius = data
            return (
                np.zeros(4, dtype=np.float32),
                center,
                radius,
            )
        raise ValueError(f'Unsupported selection mode: {mode}')

    def _set_uniforms(self, mode, rectangle_ndc, center, radius):
        projection_view = (
            self.view_handler.projection_matrix @ self.view_handler.view_matrix)
        glUniform1ui(self._uniform_location('selection_mode'), mode)
        glUniformMatrix4fv(
            self._uniform_location('projection_view'), 1, GL_FALSE,
            projection_view.T.astype(np.float32))
        glUniform4f(
            self._uniform_location('rectangle_ndc'), *rectangle_ndc)
        glUniform3f(
            self._uniform_location('selection_center'), *center)
        glUniform1f(
            self._uniform_location('selection_radius'), radius)
        glUniform1ui(
            self._uniform_location('point_count'), self._points.shape[0])

    def _uniform_location(self, name):
        return glGetUniformLocation(self._program, name)

    def _bind_selection_buffers(self):
        zero = np.zeros(1, dtype=np.uint32)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._count_buffer)
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, zero.nbytes, zero)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, self._points_buffer)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self._indices_buffer)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self._count_buffer)

    def _run_compute(self):
        groups = (self._points.shape[0] +
                  self.LOCAL_SIZE - 1) // self.LOCAL_SIZE
        glDispatchCompute(groups, 1, 1)
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

    def _read_selected_indices(self):
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._count_buffer)
        count_data = glGetBufferSubData(
            GL_SHADER_STORAGE_BUFFER, 0, self.UINT32_BYTES)
        selected_count = int(np.frombuffer(
            count_data, dtype=np.uint32, count=1)[0])
        if selected_count == 0:
            return np.empty(0, dtype=np.int64)

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._indices_buffer)
        selected_data = glGetBufferSubData(
            GL_SHADER_STORAGE_BUFFER,
            0,
            selected_count * self.UINT32_BYTES)
        return np.frombuffer(
            selected_data,
            dtype=np.uint32,
            count=selected_count).astype(np.int64, copy=True)

    def _unbind_selection_buffers(self):
        for binding in range(3):
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, 0)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0)
        glUseProgram(0)

    def _rectangle_ndc(self, selection_rect):
        pixel_ratio = self.view_handler.devicePixelRatioF()
        width = self.view_handler.current_width()
        height = self.view_handler.current_height()
        left = selection_rect.left() * pixel_ratio
        right = selection_rect.right() * pixel_ratio
        top = selection_rect.top() * pixel_ratio
        bottom = selection_rect.bottom() * pixel_ratio
        return np.array([
            left / width * 2.0 - 1.0,
            right / width * 2.0 - 1.0,
            1.0 - bottom / height * 2.0,
            1.0 - top / height * 2.0,
        ], dtype=np.float32)
