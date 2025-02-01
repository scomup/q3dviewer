"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
import numpy as np
from OpenGL.GL import shaders
from q3dviewer.utils import *


# Vertex and Fragment shader source code
vertex_shader_source = """
#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texCoord;

out vec2 TexCoord;

uniform mat4 view_matrix;
uniform mat4 project_matrix;
uniform mat4 model_matrix;

void main()
{
    gl_Position = project_matrix * view_matrix * model_matrix * vec4(position, 1.0);
    TexCoord = texCoord;
}
"""

fragment_shader_source = """
#version 330 core
in vec2 TexCoord;
out vec4 color;
uniform sampler2D ourTexture;
void main()
{
    color = texture(ourTexture, TexCoord);
}
"""


class FrameItem(BaseItem):
    def __init__(self, T=np.eye(4), size=(1, 0.8), width=3, img=None, color='#0000FF'):
        BaseItem.__init__(self)
        self.w, self.h = size
        self.width = width
        self.T = T
        self.img = img
        self.texture = None
        self.need_updating = False
        self.set_color(color)

    def initialize_gl(self):
        # Rectangle vertices and texture coordinates
        hw = self.w / 2
        hh = self.h / 2
        self.vertices = np.array([
            # positions          # texture coords
            [-hw,  hh,  0.0,  0.0, 0.0],  # bottom-left
            [ hw,  hh,  0.0,  1.0, 0.0],  # bottom-right
            [ hw, -hh,  0.0,  1.0, 1.0],  # top-right
            [-hw, -hh,  0.0,  0.0, 1.0],  # top-left
            [ 0.0,  0.0, hh * 0.66, 0.0, 0.0],  # center -Z is the front.
        ], dtype=np.float32)

        self.indices = np.array([
            0, 1, 2,  # first triangle
            2, 3, 0   # second triangle
        ], dtype=np.uint32)

        self.vao = glGenVertexArrays(1)
        self.vbo = glGenBuffers(1)
        self.ebo = glGenBuffers(1)

        glBindVertexArray(self.vao)

        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glBufferData(GL_ARRAY_BUFFER, self.vertices.itemsize *
                     5 * 4, self.vertices, GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     self.indices.nbytes, self.indices, GL_STATIC_DRAW)

        # Vertex positions
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              20, ctypes.c_void_p(0))
        glEnableVertexAttribArray(0)
        # Texture coordinates
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
                              20, ctypes.c_void_p(12))
        glEnableVertexAttribArray(1)
        project_matrix = self.glwidget().get_projection_matrix()
        # Compile shaders and create shader program
        self.program = shaders.compileProgram(
            shaders.compileShader(vertex_shader_source, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader_source, GL_FRAGMENT_SHADER),
        )
        glUseProgram(self.program)
        set_uniform(self.program, np.eye(4), 'model_matrix')
        set_uniform(self.program, project_matrix, 'project_matrix')
        glUseProgram(0)
        self.texture = glGenTextures(1)
        self.set_data(img=self.img)
        
        # Define line vertices
        self.line_vertices = np.array([
            self.vertices[0, :3], self.vertices[1, :3],
            self.vertices[1, :3], self.vertices[2, :3],
            self.vertices[2, :3], self.vertices[3, :3],
            self.vertices[3, :3], self.vertices[0, :3],
            self.vertices[4, :3], self.vertices[0, :3],
            self.vertices[4, :3], self.vertices[1, :3],
            self.vertices[4, :3], self.vertices[2, :3],
            self.vertices[4, :3], self.vertices[3, :3]
        ], dtype=np.float32)

        self.line_vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.line_vbo)
        glBufferData(GL_ARRAY_BUFFER, self.line_vertices.nbytes, self.line_vertices, GL_STATIC_DRAW)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        glBindVertexArray(0)

    def set_transform(self, Twc, is_opencv_coord=False):
        if is_opencv_coord:
            # convert the opencv camera coordinate to the opengl camera coordinate
            M_conv = np.array([
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1]
            ])
            Twc = Twc @ M_conv
        self.Twc = Twc

    def set_data(self, img=None, transform=None, is_opencv_coord=False):
        if transform is not None:
            self.set_transform(transform, is_opencv_coord)
        self.img = img
        self.need_updating = True

    def update_img_buffer(self):
        if self.need_updating:
            self.texture = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, self.texture)
            if self.img.ndim == 2:
                # Convert grayscale to RGBA
                self.img = np.stack((self.img,) * 3 + (np.ones_like(self.img) * 255,), axis=-1)
            elif self.img.shape[2] == 3:
                # Add an alpha channel
                alpha_channel = np.ones((self.img.shape[0], self.img.shape[1], 1), dtype=np.uint8) * 255
                self.img = np.concatenate((self.img, alpha_channel), axis=2)
            img = np.ascontiguousarray(self.img, dtype=np.uint8)
            # Load image
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img.shape[1],
                         img.shape[0], 0, GL_RGBA, GL_UNSIGNED_BYTE, img)
            glGenerateMipmap(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, 0)
            self.need_updating = False
        
    def set_color(self, color):
        if isinstance(color, str):
            self.rgba = hex_to_rgba(color)
        elif isinstance(color, list):
            self.rgba = color
        elif isinstance(color, tuple):
            self.rgba = list(color)
        else:
            raise ValueError("Invalid color format")

    def set_line_width(self, width):
        self.width = width

    def paint(self):
        self.view_matrix = self.glwidget().view_matrix
        project_matrix = self.glwidget().projection_matrix

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        if self.img is not None:
            self.update_img_buffer()
            glUseProgram(self.program)
            set_uniform(self.program, self.view_matrix, 'view_matrix')
            set_uniform(self.program, project_matrix, 'project_matrix')
            set_uniform(self.program, self.T, 'model_matrix')
            glBindVertexArray(self.vao)
            glBindVertexArray(0)
            glUseProgram(0)

        # if self.texture is not None:
        #     glBindTexture(GL_TEXTURE_2D, self.texture)
        #     glMultMatrixf(self.T.T)  # Apply the transformation matrix
        #     glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, None)
        #     glBindTexture(GL_TEXTURE_2D, 0)
        
        glLineWidth(self.width)
        # set color to bule
        glColor4f(*self.rgba)
        glBindBuffer(GL_ARRAY_BUFFER, self.line_vbo)
        glEnableClientState(GL_VERTEX_ARRAY)
        glMultMatrixf(self.T.T)
        glVertexPointer(3, GL_FLOAT, 0, None)
        glDrawArrays(GL_LINES, 0, len(self.line_vertices))
        glDisableClientState(GL_VERTEX_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_BLEND)

