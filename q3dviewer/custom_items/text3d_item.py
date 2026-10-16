"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import ctypes

from OpenGL.GL import *
from OpenGL.GL import shaders
import numpy as np
from q3dviewer.base_item import BaseItem
from q3dviewer.utils import set_uniform

try:
    from OpenGL.GLUT import glutBitmapCharacter, glutInit
    from OpenGL.GLUT import (
        GLUT_BITMAP_HELVETICA_10,
        GLUT_BITMAP_HELVETICA_12,
        GLUT_BITMAP_HELVETICA_18,
        GLUT_BITMAP_TIMES_ROMAN_24,
    )
    GLUT_AVAILABLE = True
except ImportError:
    GLUT_AVAILABLE = False
    print("Warning: GLUT not available. Text will not be rendered.")

def get_glut_font(font_size):
    """Get GLUT font based on size, only if GLUT is available"""
    if not GLUT_AVAILABLE:
        return None
    
    # Map requested font_size to a GLUT font object
    if font_size <= 10:
        return GLUT_BITMAP_HELVETICA_10
    elif font_size <= 12:
        return GLUT_BITMAP_HELVETICA_12
    elif font_size <= 18:
        return GLUT_BITMAP_HELVETICA_18
    elif font_size <= 24:
        return GLUT_BITMAP_TIMES_ROMAN_24
    else:
        return GLUT_BITMAP_TIMES_ROMAN_24  # Largest available


_VERTEX_SHADER = """
#version 330 core
layout(location = 0) in vec3 position;
uniform mat4 view;
uniform mat4 projection;
uniform float point_size;
void main() {
    gl_Position = projection * view * vec4(position, 1.0);
    gl_PointSize = point_size;
}
"""

_FRAGMENT_SHADER = """
#version 330 core
uniform vec4 color;
out vec4 fragment_color;
void main() {
    fragment_color = color;
}
"""
    

# draw points with color (x, y, z, color)
class Text3DItem(BaseItem):
    """
    A OpenGL 3d text and mark item.

    Attributes:
        data: list storing text data. Each element is a dict with keys:
            'text': str, the text to display
            'position': (x, y, z), the 3D position of the text
            'color': (r, g, b, a), the color of the text
            'font_size': float, the font size of the text
            'point_size': float, size of point to draw at position (0 for no point)
            'line_width': float, width of line to draw between points (0 for no line)

    """
    def __init__(self, data=None):
        super().__init__()
        self._disable_setting = True
        self.data_list = [] if data is None else data
        self.program = None
        self.vbo = None
        self._points = np.empty((0, 3), dtype=np.float32)
        self._data_dirty = True
        self._gpu_dirty = True

    def add_setting(self, layout):
        pass # No settings for Text3DItem

    def set_data(self, data, append=False):
        if not append:
            self.data_list = []
        self.data_list.extend(data)
        self._data_dirty = True
        self._gpu_dirty = True
        self.notify_changed()

    def clear_data(self):
        self.data_list = []
        self._data_dirty = True
        self._gpu_dirty = True
        self.notify_changed()


    def initialize_gl(self):
        """Initialize modern point/line rendering and optional GLUT text."""
        global GLUT_AVAILABLE
        if GLUT_AVAILABLE:
            try:
                # Check if glutInit is actually callable before calling it
                if hasattr(glutInit, '__call__') and bool(glutInit):
                    glutInit()
                else:
                    print("Warning: glutInit not callable, using fallback text rendering")
                    GLUT_AVAILABLE = False
            except Exception as e:
                print(f"Warning: GLUT initialization failed: {e}. Using fallback text rendering.")
                GLUT_AVAILABLE = False
        self.program = shaders.compileProgram(
            shaders.compileShader(_VERTEX_SHADER, GL_VERTEX_SHADER),
            shaders.compileShader(_FRAGMENT_SHADER, GL_FRAGMENT_SHADER),
        )
        self.vbo = glGenBuffers(1)

    def _prepare_points(self):
        if not self._data_dirty:
            return
        positions = []
        for item in self.data_list:
            if isinstance(item, dict):
                positions.append(item.get('position', (0.0, 0.0, 0.0)))
        if positions:
            self._points = np.ascontiguousarray(positions, dtype=np.float32)
        else:
            self._points = np.empty((0, 3), dtype=np.float32)
        self._data_dirty = False
        self._gpu_dirty = True

    def _draw_points_and_lines(self):
        self._prepare_points()
        if self.program is None or self._points.shape[0] == 0:
            return

        first = self.data_list[0] if self.data_list else {}
        color = np.asarray(
            first.get('color', (1.0, 1.0, 1.0, 1.0)), dtype=np.float32)
        point_size = float(first.get('point_size', 0.0))
        line_width = float(first.get('line_width', 0.0))
        glUseProgram(self.program)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        if self._gpu_dirty:
            glBufferData(
                GL_ARRAY_BUFFER,
                self._points.nbytes,
                self._points,
                GL_DYNAMIC_DRAW)
            self._gpu_dirty = False
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 12, ctypes.c_void_p(0))
        widget = self.glwidget()
        set_uniform(self.program, widget.view_matrix, 'view')
        set_uniform(self.program, widget.projection_matrix, 'projection')
        glUniform4f(glGetUniformLocation(self.program, 'color'), *color)
        glUniform1f(
            glGetUniformLocation(self.program, 'point_size'),
            max(point_size, 1.0))
        glDisable(GL_DEPTH_TEST)
        glDepthMask(GL_FALSE)
        glEnable(GL_BLEND)
        glEnable(GL_PROGRAM_POINT_SIZE)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        if line_width > 0.0 and self._points.shape[0] >= 2:
            glLineWidth(line_width)
            glDrawArrays(GL_LINE_STRIP, 0, self._points.shape[0])
        if point_size > 0.0:
            glUniform1f(
                glGetUniformLocation(self.program, 'point_size'), point_size)
            glDrawArrays(GL_POINTS, 0, self._points.shape[0])
        glDisable(GL_BLEND)
        glDisable(GL_PROGRAM_POINT_SIZE)
        glDepthMask(GL_TRUE)
        glEnable(GL_DEPTH_TEST)
        glDisableVertexAttribArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glUseProgram(0)


    def paint(self):
        self._draw_points_and_lines()

        if not GLUT_AVAILABLE:
            return

        # Keep legacy text labels available when GLUT is installed.
        glDisable(GL_DEPTH_TEST)
        glDepthMask(GL_FALSE)
        for item in self.data_list:
            if isinstance(item, dict):
                text = item.get('text', '')
                pos = item.get('position', (0.0, 0.0, 0.0))
                font_size = item.get('font_size', 24)
                color = item.get('color', (1.0, 1.0, 1.0, 1.0))
            elif isinstance(item, str):
                text = item
                pos = (0.0, 0.0, 0.0)
                font_size = 24
                color = (1.0, 1.0, 1.0, 1.0)
            else:
                continue
            if isinstance(pos, np.ndarray):
                pos = tuple(pos.astype(float))
            glColor4f(*color)
            glRasterPos3f(pos[0] + 0.02, pos[1] + 0.02, pos[2] + 0.02)
            font = get_glut_font(font_size)
            if font is not None:
                for char in text:
                    glutBitmapCharacter(font, ord(char))
        glDepthMask(GL_TRUE)
        glEnable(GL_DEPTH_TEST)