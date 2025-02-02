"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
import numpy as np
from OpenGL.GL import shaders
from PIL import Image as PIL_Image
from PySide6.QtWidgets import QLabel, QSpinBox, QCheckBox


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


class ImageItem(BaseItem):
    def __init__(self, pos=np.array([0, 0]), size=np.array([1280/2, 720/2])):
        BaseItem.__init__(self)
        self.pos = pos  # bottom-left
        self.size = size
        self.image = np.zeros((self.size[0], self.size[1], 4), dtype=np.uint8)
        self.alpha = 255

    def initialize_gl(self):
        # Rectangle vertices and texture coordinates
        width = self.glwidget().current_width()
        height = self.glwidget().current_height()
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

    def set_data(self, data):
        if isinstance(data, np.ndarray):
            pass
        elif isinstance(data, PIL_Image.Image):
            data = np.array(data)
        else:
            print("not support image type")
            raise NotImplementedError

        if data.ndim == 2:  # Grayscale image
            data = np.stack((data,) * 3 + (np.ones_like(data) * 255,), axis=-1)
        elif data.shape[-1] == 3:  # RGB image
            alpha_channel = np.ones(
                (data.shape[0], data.shape[1], 1),
                dtype=data.dtype) * self.alpha
            data = np.concatenate((data, alpha_channel), axis=-1)
        self.image = data

    def paint(self):
        if self.image is not None:
            img_data = self.image
            img_data = np.flipud(img_data)  # Flip the image vertically
            img_data = img_data.tobytes()
            glBindTexture(GL_TEXTURE_2D, self.texture)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.image.shape[1],
                         self.image.shape[0], 0,
                         GL_RGBA, GL_UNSIGNED_BYTE, img_data)
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

    def add_setting(self, layout):
        spinbox_alpha = QSpinBox()
        spinbox_alpha.setPrefix("Alpha: ")
        spinbox_alpha.setSingleStep(1)
        spinbox_alpha.setRange(0, 255)
        spinbox_alpha.setValue(self.alpha)
        spinbox_alpha.valueChanged.connect(self.set_alpha)
        layout.addWidget(spinbox_alpha)
        
        checkbox_show = QCheckBox("Show Image")
        checkbox_show.setChecked(True)
        checkbox_show.stateChanged.connect(self.set_visible)
        layout.addWidget(checkbox_show)

    def set_alpha(self, alpha):
        self.alpha = alpha
