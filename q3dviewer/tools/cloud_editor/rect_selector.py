"""GPU rectangle selection for point clouds."""

from pathlib import Path

import numpy as np
from OpenGL.GL import *
from OpenGL.GL import shaders


class RectSelector:
    """Select point indices inside a screen-space rectangle on the GPU."""

    LOCAL_SIZE = 256

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

    def select(self, selection_rect):
        if self._points is None or self._points.shape[0] == 0:
            return np.empty(0, dtype=np.int64)

        self.view_handler.makeCurrent()
        try:
            self._ensure_resources()
            return self._dispatch(selection_rect)
        finally:
            self.view_handler.doneCurrent()

    def _ensure_resources(self):
        if self._program is None:
            shader_path = (Path(__file__).resolve().parent.parent
                           .parent / 'shaders' / 'rectangle_select_comp.glsl')
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

    def _dispatch(self, selection_rect):
        view_matrix = self.view_handler.view_matrix
        if self.view_handler.need_recalc_view:
            from q3dviewer.utils.maths import calc_view_matrix
            view_matrix = calc_view_matrix(
                self.view_handler.center,
                self.view_handler.dist,
                self.view_handler.euler)
        projection_view = (
            self.view_handler.get_projection_matrix() @ view_matrix)

        rectangle_ndc = self._rectangle_ndc(selection_rect)
        glUseProgram(self._program)
        glUniformMatrix4fv(
            glGetUniformLocation(self._program, 'projection_view'),
            1,
            GL_FALSE,
            projection_view.T.astype(np.float32))
        glUniform4f(
            glGetUniformLocation(self._program, 'rectangle_ndc'),
            *rectangle_ndc)
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
