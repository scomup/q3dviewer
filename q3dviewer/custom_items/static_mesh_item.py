#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
from OpenGL.GL import shaders
from q3dviewer.Qt.QtWidgets import QLabel, QCheckBox, QDoubleSpinBox, QSlider, QHBoxLayout, QLineEdit

import os
from q3dviewer.utils import set_uniform, text_to_rgba


class StaticMeshItem(BaseItem):
    """
    A OpenGL mesh item for rendering static 3D triangular meshes.
    Optimized for static geometry with triangle-only rendering.
    Data format: Nx9 numpy array (3 vertices per triangle, no good flag needed)
    
    Attributes:
        color (str or tuple): Accepts any valid matplotlib color (e.g., 'red', '#FF4500', (1.0, 0.5, 0.0)).
        wireframe (bool): If True, renders the mesh in wireframe mode.
    """
    def __init__(self, color='lightblue', wireframe=False):
        super(StaticMeshItem, self).__init__()
        self.wireframe = wireframe
        self.color = color
        self.flat_rgb = text_to_rgba(color, flat=True)
        
        # Static mesh buffer: N x 9 numpy array
        # Each row: [v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z]
        self.vertices = None
        self.num_triangles = 0
        
        # OpenGL objects
        self.vao = None
        self.vbo = None
        self.program = None
        
        # Fixed rendering parameters
        self.enable_lighting = True
        self.line_width = 1.0
        self.light_color = [1.0, 1.0, 1.0]
        self.ambient_strength = 0.1
        self.diffuse_strength = 1.2
        self.specular_strength = 0.1
        self.shininess = 32.0
        self.alpha = 1.0
        
        # Settings flag
        self.need_update_setting = True
        self.need_update_buffer = True
        self.path = os.path.dirname(__file__)
    
    def add_setting(self, layout):
        """Add UI controls for mesh visualization"""
        # Wireframe toggle
        self.wireframe_box = QCheckBox("Wireframe Mode")
        self.wireframe_box.setChecked(self.wireframe)
        self.wireframe_box.toggled.connect(self.update_wireframe)
        layout.addWidget(self.wireframe_box)

        # Enable lighting toggle
        self.lighting_box = QCheckBox("Enable Lighting")
        self.lighting_box.setChecked(self.enable_lighting)
        self.lighting_box.toggled.connect(self.update_enable_lighting)
        layout.addWidget(self.lighting_box)

        # Color setting
        label_rgb = QLabel("Color:")
        label_rgb.setToolTip("Use hex color, i.e. #FF4500, or named color, i.e. 'red'")
        layout.addWidget(label_rgb)
        self.edit_rgb = QLineEdit()
        self.edit_rgb.setToolTip("Use hex color, i.e. #FF4500, or named color, i.e. 'red'")
        self.edit_rgb.setText(self.color)
        self.edit_rgb.textChanged.connect(self._on_color)
        layout.addWidget(self.edit_rgb)

        # Material property controls for Phong lighting
        if self.enable_lighting:
            # Ambient strength control
            ambient_layout = QHBoxLayout()
            ambient_label = QLabel("Ambient Strength:")
            ambient_layout.addWidget(ambient_label)
            self.ambient_slider = QSlider()
            self.ambient_slider.setOrientation(1)  # Qt.Horizontal
            self.ambient_slider.setRange(0, 100)
            self.ambient_slider.setValue(int(self.ambient_strength * 100))
            self.ambient_slider.valueChanged.connect(lambda v: self.update_ambient_strength(v / 100.0))
            ambient_layout.addWidget(self.ambient_slider)
            layout.addLayout(ambient_layout)

            # Diffuse strength control
            diffuse_layout = QHBoxLayout()
            diffuse_label = QLabel("Diffuse Strength:")
            diffuse_layout.addWidget(diffuse_label)
            self.diffuse_slider = QSlider()
            self.diffuse_slider.setOrientation(1)
            self.diffuse_slider.setRange(0, 200)
            self.diffuse_slider.setValue(int(self.diffuse_strength * 100))
            self.diffuse_slider.valueChanged.connect(lambda v: self.update_diffuse_strength(v / 100.0))
            diffuse_layout.addWidget(self.diffuse_slider)
            layout.addLayout(diffuse_layout)

            # Specular strength control
            specular_layout = QHBoxLayout()
            specular_label = QLabel("Specular Strength:")
            specular_layout.addWidget(specular_label)
            self.specular_slider = QSlider()
            self.specular_slider.setOrientation(1)
            self.specular_slider.setRange(0, 100)
            self.specular_slider.setValue(int(self.specular_strength * 100))
            self.specular_slider.valueChanged.connect(lambda v: self.update_specular_strength(v / 100.0))
            specular_layout.addWidget(self.specular_slider)
            layout.addLayout(specular_layout)

            # Shininess control
            shininess_layout = QHBoxLayout()
            shininess_label = QLabel("Shininess:")
            shininess_layout.addWidget(shininess_label)
            self.shininess_slider = QSlider()
            self.shininess_slider.setOrientation(1)
            self.shininess_slider.setRange(1, 256)
            self.shininess_slider.setValue(int(self.shininess))
            self.shininess_slider.valueChanged.connect(lambda v: self.update_shininess(float(v)))
            shininess_layout.addWidget(self.shininess_slider)
            layout.addLayout(shininess_layout)

    def _on_color(self, color):
        try:
            self.color = color
            self.flat_rgb = text_to_rgba(color, flat=True)
            self.need_update_setting = True
        except ValueError:
            pass

    def update_wireframe(self, value):
        self.wireframe = value
        
    def update_enable_lighting(self, value):
        self.enable_lighting = value
        self.need_update_setting = True
        
    def update_line_width(self, value):
        self.line_width = value
        self.need_update_setting = True
        
    def update_ambient_strength(self, value):
        self.ambient_strength = value
        self.need_update_setting = True
        
    def update_diffuse_strength(self, value):
        self.diffuse_strength = value
        self.need_update_setting = True
        
    def update_specular_strength(self, value):
        self.specular_strength = value
        self.need_update_setting = True
        
    def update_shininess(self, value):
        self.shininess = value
        self.need_update_setting = True

    def update_alpha(self, value):
        """Update mesh alpha (opacity)"""
        self.alpha = float(value)
        self.need_update_setting = True

    def set_data(self, data):
        """
        Set complete mesh data at once.
        
        Args:
            data: Nx9 numpy array where N is the number of triangles
                  Each row: [v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z]
        """
        print("Setting static mesh data with {} triangles".format(len(data)))
        if not isinstance(data, np.ndarray):
            raise ValueError("Data must be a numpy array")

        # Check shape
        if data.ndim != 2 or data.shape[1] != 9:
            # Try to reshape if it's Nx3 (vertex list)
            if data.ndim == 2 and data.shape[1] == 3 and data.shape[0] % 3 == 0:
                data = data.reshape(-1, 9)
            else:
                raise ValueError(f"Invalid data shape {data.shape}. Expected Nx9 or (N*3)x3")

        self.vertices = data.astype(np.float32)
        self.num_triangles = len(self.vertices)
        self.need_update_buffer = True

    def clear_mesh(self):
        """Clear all mesh data"""
        self.vertices = None
        self.num_triangles = 0
        self.need_update_buffer = True

    def initialize_gl(self):
        """OpenGL initialization - load triangle shader"""
        frag_shader = open(self.path + '/../shaders/mesh_frag.glsl', 'r').read()
        
        try:
            vert_shader = open(self.path + '/../shaders/triangle_mesh_vert.glsl', 'r').read()
            geom_shader = open(self.path + '/../shaders/triangle_mesh_geom.glsl', 'r').read()
            self.program = shaders.compileProgram(
                shaders.compileShader(vert_shader, GL_VERTEX_SHADER),
                shaders.compileShader(geom_shader, GL_GEOMETRY_SHADER),
                shaders.compileShader(frag_shader, GL_FRAGMENT_SHADER),
            )
        except Exception as e:
            print(f"Error compiling static mesh shader: {e}")
            raise

    def update_render_buffer(self):
        """
        Update GPU buffer with triangle data.
        Each triangle: 9 floats (3 vertices x 3 coordinates)
        """
        if self.num_triangles == 0 or self.vertices is None:
            return
        
        # Initialize buffers on first call
        if self.vao is None:
            self.vao = glGenVertexArrays(1)
            self.vbo = glGenBuffers(1)
        
        if not self.need_update_buffer:
            return
            
        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        
        # Upload all triangle data
        glBufferData(GL_ARRAY_BUFFER,
                    self.vertices.nbytes,
                    self.vertices,
                    GL_STATIC_DRAW)
        
        # Setup vertex attributes
        # Triangle: [v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z]
        # 9 floats = 36 bytes stride
        stride = 36
        
        # v0 (location 1) - vec3
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(0))
        glVertexAttribDivisor(1, 1)
        
        # v1 (location 2) - vec3
        glEnableVertexAttribArray(2)
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(12))
        glVertexAttribDivisor(2, 1)
        
        # v2 (location 3) - vec3
        glEnableVertexAttribArray(3)
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(24))
        glVertexAttribDivisor(3, 1)
        
        glBindVertexArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        self.need_update_buffer = False
        
    def update_setting(self):
        """Set rendering parameters"""
        if not self.need_update_setting:
            return
        
        set_uniform(self.program, int(self.enable_lighting), 'if_light')
        set_uniform(self.program, 1, 'two_sided')
        set_uniform(self.program, np.array(self.light_color, dtype=np.float32), 'light_color')
        set_uniform(self.program, float(self.ambient_strength), 'ambient_strength')
        set_uniform(self.program, float(self.diffuse_strength), 'diffuse_strength')
        set_uniform(self.program, float(self.specular_strength), 'specular_strength')
        set_uniform(self.program, float(self.shininess), 'shininess')
        set_uniform(self.program, float(self.alpha), 'alpha')
        set_uniform(self.program, int(self.flat_rgb), 'flat_rgb')
        self.need_update_setting = False

    def paint(self):
        """
        Render the static mesh using instanced rendering with geometry shader.
        Each triangle instance is rendered as a point, geometry shader generates 1 triangle.
        """
        if self.num_triangles == 0 or self.vertices is None:
            return
        
        glUseProgram(self.program)
    
        self.update_render_buffer()
        self.update_setting()
        
        view_matrix = self.glwidget().view_matrix
        set_uniform(self.program, view_matrix, 'view')
        project_matrix = self.glwidget().projection_matrix
        set_uniform(self.program, project_matrix, 'projection')
        view_pos = self.glwidget().center
        set_uniform(self.program, np.array(view_pos), 'view_pos')
            
        # Enable blending and depth testing
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_DEPTH_TEST)
        glDisable(GL_CULL_FACE)  # two-sided rendering
        
        # Set line width
        glLineWidth(self.line_width)
        
        # Bind VAO
        glBindVertexArray(self.vao)
                
        if self.wireframe:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        else:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        
        # Draw using instanced rendering
        # Input: POINTS (one per triangle instance)
        # Geometry shader generates 1 triangle (3 vertices) per point
        glDrawArraysInstanced(GL_POINTS, 0, 1, self.num_triangles)
            
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glBindVertexArray(0)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_BLEND)
        glUseProgram(0)
