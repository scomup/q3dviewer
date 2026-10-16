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

    def select_rectangle(self, selection_rect):
        return self._select(
            mode=self.RECTANGLE_MODE,
            selection_rect=selection_rect)

    def select_radius(self, center, radius):
        return self._select(
            mode=self.RADIUS_MODE,
            center=np.asarray(center, dtype=np.float32),
            radius=float(radius))

    def _select(self, mode, selection_rect=None, center=None, radius=0.0):
        if self._points is None or self._points.shape[0] == 0:
            return np.empty(0, dtype=np.int64)

        self.view_handler.makeCurrent()
        try:
            self._ensure_resources()
            return self._dispatch(mode, selection_rect, center, radius)
        finally:
            self.view_handler.doneCurrent()

    def _ensure_resources(self):
        if self._program is None:
            shader_path = (Path(__file__).resolve().parent.parent
                           .parent / 'shaders' / 'points_select_comp.glsl')
            source = shader_path.read_text(encoding='utf-8')
            compute_shader = shaders.compileShader(source, GL_COMPUTE_SHADER)
            self._program = shaders.compileProgram(compute_shader)

        if self._points_buffer is None:
            self._points_buffer = glGenBuffers(1)
        if self._indices_buffer is None:
            self._indices_buffer = glGenBuffers(1)
        if self._count_buffer is None:
            self._count_buffer = glGenBuffers(1)

        if self._points_buffer_dirty:
            points = np.empty((self._points.shape[0], 4), dtype=np.float32)
            points[:, :3] = self._points
            points[:, 3] = 1.0
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._points_buffer)
            glBufferData(
                GL_SHADER_STORAGE_BUFFER, points.nbytes, points, GL_STATIC_DRAW)
            self._points_buffer_dirty = False

        point_count = self._points.shape[0]
        if self._index_capacity < point_count:
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._indices_buffer)
            glBufferData(
                GL_SHADER_STORAGE_BUFFER,
                point_count * np.dtype(np.uint32).itemsize,
                None,
                GL_DYNAMIC_READ)
            self._index_capacity = point_count

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._count_buffer)
        glBufferData(
            GL_SHADER_STORAGE_BUFFER,
            np.dtype(np.uint32).itemsize,
            None,
            GL_DYNAMIC_READ)

    def _dispatch(self, mode, selection_rect, center, radius):
        projection_view = (
            self.view_handler.projection_matrix @ self.view_handler.view_matrix)
        if mode == self.RECTANGLE_MODE:
            rectangle_ndc = self._rectangle_ndc(selection_rect)
        else:
            rectangle_ndc = np.zeros(4, dtype=np.float32)
        glUseProgram(self._program)
        glUniform1ui(
            glGetUniformLocation(self._program, 'selection_mode'), mode)
        glUniformMatrix4fv(
            glGetUniformLocation(self._program, 'projection_view'),
            1,
            GL_FALSE,
            projection_view.T.astype(np.float32))
        glUniform4f(
            glGetUniformLocation(self._program, 'rectangle_ndc'),
            *rectangle_ndc)
        glUniform3f(
            glGetUniformLocation(self._program, 'selection_center'),
            *(center if center is not None else np.zeros(3, dtype=np.float32)))
        glUniform1f(
            glGetUniformLocation(self._program, 'selection_radius'), radius)
        glUniform1ui(
            glGetUniformLocation(self._program, 'point_count'),
            self._points.shape[0])

        zero = np.zeros(1, dtype=np.uint32)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._count_buffer)
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, zero.nbytes, zero)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, self._points_buffer)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self._indices_buffer)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self._count_buffer)

        groups = (self._points.shape[0] + self.LOCAL_SIZE - 1) // self.LOCAL_SIZE
        glDispatchCompute(groups, 1, 1)
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._count_buffer)
        count_data = glGetBufferSubData(
            GL_SHADER_STORAGE_BUFFER, 0, np.dtype(np.uint32).itemsize)
        selected_count = int(np.frombuffer(
            count_data, dtype=np.uint32, count=1)[0])
        if selected_count == 0:
            selected = np.empty(0, dtype=np.int64)
        else:
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self._indices_buffer)
            selected_data = glGetBufferSubData(
                GL_SHADER_STORAGE_BUFFER,
                0,
                selected_count * np.dtype(np.uint32).itemsize)
            selected = np.frombuffer(
                selected_data,
                dtype=np.uint32,
                count=selected_count).astype(np.int64, copy=True)

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0)
        glUseProgram(0)
        return selected

    def _rectangle_ndc(self, selection_rect):
        if selection_rect is None:
            return np.zeros(4, dtype=np.float32)
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
