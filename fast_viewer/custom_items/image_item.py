import pyqtgraph.opengl as gl
from OpenGL.GL import *
# from PyQt5 import QtCore, QtGui, QtWidgets
import numpy as np
from OpenGL.GL import shaders
from PIL import Image


# Vertex and Fragment shader source code
vertex_shader_source = """
#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texCoord;

out vec2 TexCoord;

uniform mat4 view_matrix;
uniform mat4 project_matrix;

void main()
{
    gl_Position = vec4(position, 1.0);
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


def set_uniform_mat4(shader, content, name):
    content = content.T
    glUniformMatrix4fv(
        glGetUniformLocation(shader, name),
        1,
        GL_FALSE,
        content.astype(np.float32)
    )


class ImageItem(gl.GLGraphicsItem.GLGraphicsItem):
    def __init__(self, pos=np.array([10, 10]), size=np.array([1280/2, 720/2])):
        gl.GLGraphicsItem.GLGraphicsItem.__init__(self)
        self.pos = pos  # bottom-left
        self.size = size
        self.image = None

    def initializeGL(self):
        # Rectangle vertices and texture coordinates
        width = self._GLGraphicsItem__view.deviceWidth()
        height = self._GLGraphicsItem__view.deviceHeight()
        x0, y0 = self.pos
        x1, y1 = self.pos + self.size
        x0 = x0 / width * 2 - 1
        y0 = y0 / height * 2 - 1
        x1 = x1 / width * 2 - 1
        y1 = y1 / height * 2 - 1

        self.vertices = np.array([
            # positions          # texture coords
            [x0, y0,  0.0,  0.0, 0.0],  # bottom-left
            [x1, y0,  0.0,  1.0, 0.0],  # bottom-right
            [x1,  y1,  0.0,  1.0, 1.0],  # top-right
            [x0,  y1,  0.0,  0.0, 1.0],   # top-left
        ], dtype=np.float32)

        indices = np.array([
            0, 1, 2,  # first triangle
            2, 3, 0   # second triangle
        ], dtype=np.uint32)

        self.vao = glGenVertexArrays(1)
        vbo = glGenBuffers(1)
        ebo = glGenBuffers(1)

        glBindVertexArray(self.vao)

        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(GL_ARRAY_BUFFER, self.vertices.itemsize *
                     5 * 4, self.vertices, GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     indices.nbytes, indices, GL_STATIC_DRAW)

        # Vertex positions
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,
                              20, ctypes.c_void_p(0))
        glEnableVertexAttribArray(0)

        # Texture coordinates
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE,
                              20, ctypes.c_void_p(12))
        glEnableVertexAttribArray(1)

        # Compile shaders and create shader program
        self.program = shaders.compileProgram(
            shaders.compileShader(vertex_shader_source, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader_source, GL_FRAGMENT_SHADER),
        )

        self.texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.texture)
        glBindVertexArray(0)

    def setData(self, data):
        if isinstance(data, np.ndarray):
            self.image = Image.fromarray(data)
        elif isinstance(data, Image.Image):
            self.image = data
        else:
            raise NotImplementedError
            print("not support image type")

    def paint(self):
        if self.image is not None:
            self.image = self.image.transpose(Image.FLIP_TOP_BOTTOM)
            img_data = self.image.convert("RGBA").tobytes()
            glBindTexture(GL_TEXTURE_2D, self.texture)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.image.width,
                         self.image.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, img_data)
            glGenerateMipmap(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, 0)
            self.image = None

        glEnable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glUseProgram(self.program)
        glBindVertexArray(self.vao)
        glBindTexture(GL_TEXTURE_2D, self.texture)
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, None)
        glBindTexture(GL_TEXTURE_2D, 0)
        glBindVertexArray(0)
        glUseProgram(0)

        glDisable(GL_DEPTH_TEST)
        glDisable(GL_BLEND)
