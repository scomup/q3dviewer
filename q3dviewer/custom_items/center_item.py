import ctypes

import numpy as np
from OpenGL.GL import *
from OpenGL.GL import shaders
from q3dviewer.Qt.QtCore import Qt
from q3dviewer.Qt.QtWidgets import QCheckBox, QHBoxLayout, QLabel, QLineEdit, QSlider
from q3dviewer.base_item import BaseItem
from q3dviewer.utils import set_uniform, text_to_rgba


_MESH_VERT_SRC = """
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 instancePos;
uniform float scale;
out vec3 vFragPos;
out vec3 vNormal;
void main() {
    vec3 worldPos = aPos * scale + instancePos;
    vFragPos = worldPos;
    vNormal = aNormal;
    gl_Position = projection * view * vec4(worldPos, 1.0);
}
"""

_MESH_FRAG_SRC = """
#version 330 core
out vec4 FragColor;
in vec3 vFragPos;
in vec3 vNormal;
uniform vec3 uColor;
uniform vec3 viewPos;
uniform float alpha;
void main() {
    vec3 norm = normalize(vNormal);
    vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));
    float ambient = 0.3;
    float diffuse = 0.7 * max(abs(dot(norm, lightDir)), 0.0);
    vec3 viewDir = normalize(viewPos - vFragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float specular = 0.2 * pow(max(abs(dot(viewDir, reflectDir)), 0.0), 32.0);
    FragColor = vec4((ambient + diffuse + specular) * uColor, alpha);
}
"""


class CenterItem(BaseItem):
    def __init__(self, enabled=True, alpha=0.5, color='cyan'):
        super().__init__()
        self.enabled = enabled
        self.alpha = float(alpha)
        self.color = color
        self.flat_rgb = text_to_rgba(color, flat=True)
        self.pending = False
        self.program = None
        self.vao = None
        self.vbo = None
        self.ebo = None
        self.index_count = 0

    def switch(self, state):
        state = bool(state)
        if self.pending != state:
            self.pending = state
            self.notify_changed()

    def add_setting(self, layout):
        show_center = QCheckBox("Show Center Point")
        show_center.setChecked(self.enabled)
        show_center.stateChanged.connect(self.set_enabled)
        layout.addWidget(show_center)

        color_layout = QHBoxLayout()
        color_layout.addWidget(QLabel("Color:"))
        color_edit = QLineEdit(str(self.color))
        color_edit.setToolTip("Use hex color or named color")
        color_edit.textChanged.connect(self.set_color)
        color_layout.addWidget(color_edit)
        layout.addLayout(color_layout)

        alpha_layout = QHBoxLayout()
        alpha_layout.addWidget(QLabel("Alpha:"))
        alpha_slider = QSlider(Qt.Horizontal)
        alpha_slider.setRange(0, 100)
        alpha_slider.setValue(int(self.alpha * 100))
        alpha_slider.valueChanged.connect(lambda value: self.set_alpha(value / 100.0))
        alpha_layout.addWidget(alpha_slider)
        layout.addLayout(alpha_layout)

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        if not self.enabled:
            self.pending = False
        self.notify_changed()

    def set_alpha(self, alpha):
        self.alpha = float(alpha)
        self.notify_changed()

    def set_color(self, color):
        if isinstance(color, str):
            try:
                flat_rgb = text_to_rgba(color, flat=True)
            except ValueError:
                return
            self.color = color
        elif isinstance(color, (int, np.integer)):
            flat_rgb = int(color)
            self.color = f'#{flat_rgb:06x}'
        else:
            raise TypeError("Color must be a string or packed integer")
        self.flat_rgb = flat_rgb
        self.notify_changed()

    def save_setting(self):
        return {
            'enabled': self.enabled,
            'alpha': self.alpha,
            'flat_rgb': int(self.flat_rgb),
        }

    def load_setting(self, setting):
        if setting and 'enabled' in setting:
            self.set_enabled(setting['enabled'])
        if setting and 'alpha' in setting:
            self.set_alpha(setting['alpha'])
        if setting and 'flat_rgb' in setting:
            self.set_color(setting['flat_rgb'])

    def initialize_gl(self):
        vertices, indices = self._make_sphere()
        self.index_count = len(indices)
        self.program = shaders.compileProgram(
            shaders.compileShader(_MESH_VERT_SRC, GL_VERTEX_SHADER),
            shaders.compileShader(_MESH_FRAG_SRC, GL_FRAGMENT_SHADER),
        )
        self.vao = glGenVertexArrays(1)
        self.vbo = glGenBuffers(1)
        self.ebo = glGenBuffers(1)
        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW)
        stride = 6 * 4
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(0))
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(12))
        glBindVertexArray(0)

    @staticmethod
    def _make_sphere(slices=12, stacks=8):
        vertices = []
        indices = []
        for stack in range(stacks + 1):
            phi = np.pi * stack / stacks
            sin_phi = np.sin(phi)
            cos_phi = np.cos(phi)
            for slice_index in range(slices + 1):
                theta = 2.0 * np.pi * slice_index / slices
                normal = np.array([
                    sin_phi * np.cos(theta),
                    sin_phi * np.sin(theta),
                    cos_phi,
                ], dtype=np.float32)
                vertices.append(np.concatenate((normal, normal)))
        for stack in range(stacks):
            for slice_index in range(slices):
                first = stack * (slices + 1) + slice_index
                second = first + slices + 1
                indices.extend((first, second, first + 1,
                                second, second + 1, first + 1))
        return np.asarray(vertices, dtype=np.float32), np.asarray(indices, dtype=np.uint32)

    def paint(self):
        if not self.enabled or not self.pending or self.program is None:
            return
        widget = self.glwidget()
        glUseProgram(self.program)
        glBindVertexArray(self.vao)
        set_uniform(self.program, widget.view_matrix, 'view')
        set_uniform(self.program, widget.projection_matrix, 'projection')
        set_uniform(self.program, np.asarray(widget.center, dtype=np.float32), 'instancePos')
        focal = widget.get_K()[0, 0]
        point_size = np.clip(focal / widget.dist, 10, 100)
        sphere_radius = point_size * widget.dist / (2.0 * focal)
        set_uniform(self.program, float(sphere_radius), 'scale')
        set_uniform(self.program, np.asarray(widget.center, dtype=np.float32), 'viewPos')
        color = np.array([
            (self.flat_rgb >> 16 & 0xff) / 255.0,
            (self.flat_rgb >> 8 & 0xff) / 255.0,
            (self.flat_rgb & 0xff) / 255.0,
        ], dtype=np.float32)
        set_uniform(self.program, color, 'uColor')
        set_uniform(self.program, self.alpha, 'alpha')
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_DEPTH_TEST)
        glDrawElements(GL_TRIANGLES, self.index_count, GL_UNSIGNED_INT, None)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_BLEND)
        glBindVertexArray(0)
        glUseProgram(0)
