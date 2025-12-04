#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""


import numpy as np
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
from OpenGL.GL import shaders
from q3dviewer.Qt.QtWidgets import QLabel, QCheckBox, QDoubleSpinBox, QSlider, QHBoxLayout, QLineEdit, QComboBox
import matplotlib.colors as mcolors
import os
from q3dviewer.utils import set_uniform, text_to_rgba



class MeshItem(BaseItem):
    """
    An OpenGL mesh item for rendering triangulated 3D surfaces.

    Attributes:
        color (str or tuple): The flat color to use when `color_mode` is 'FLAT'. 
            Accepts any valid matplotlib color (e.g., 'lightblue', 'red', '#FF4500', (1.0, 0.5, 0.0)).
        wireframe (bool): If True, render the mesh in wireframe mode (edges only). 
            If False, render filled triangles. Default is False.
        enable_lighting (bool): Whether to enable Phong lighting for the mesh. 
            If True, the mesh will be shaded based on light direction and material properties.
            If False, the mesh will use flat shading with object colors only. Default is True.
        color_mode (str): The coloring mode for mesh vertices.
            - 'FLAT': Single flat color for all vertices (uses the `color` attribute).
            - 'I': Color by intensity channel from per-vertex colors (rainbow gradient).
            - 'RGB': Per-vertex RGB color from per-vertex color data.
        alpha (float): The transparency of the mesh, in the range [0, 1], 
            where 0 is fully transparent and 1 is fully opaque. Default is 1.0.
        line_width (float): The width of lines when rendering in wireframe mode. 
            Range is typically 0.5 to 5.0. Default is 1.0.
        
    Material Properties (Phong Lighting):
        ambient_strength (float): Ambient light contribution [0.0-1.0]. Default is 0.1.
        diffuse_strength (float): Diffuse light contribution [0.0-2.0]. Default is 1.2.
        specular_strength (float): Specular highlight contribution [0.0-2.0]. Default is 0.1.
        shininess (float): Specular shininess exponent [1-256]. Higher values = smaller highlights. Default is 32.0.
        
    Methods:
        set_data(verts, faces, colors=None): Set mesh geometry and optional per-vertex colors.
            - verts: np.ndarray of shape (N, 3) - vertex positions
            - faces: np.ndarray of shape (M, 3) with uint32 indices - triangle indices
            - colors: np.ndarray of shape (N,) with uint32 IRGB format (optional)
                     uint32 format: I (bits 24-31), R (bits 16-23), G (bits 8-15), B (bits 0-7)
                     
    Example:
        # Create a simple triangle mesh with per-vertex colors
        verts = np.array([[0,0,0], [1,0,0], [0,1,0]], dtype=np.float32)
        faces = np.array([[0,1,2]], dtype=np.uint32)
        colors = np.array([
            (255 << 24) | (255 << 16) | (0 << 8) | 0,  # Red, intensity=255
            (200 << 24) | (0 << 16) | (255 << 8) | 0,  # Green, intensity=200
            (150 << 24) | (0 << 16) | (0 << 8) | 255   # Blue, intensity=150
        ], dtype=np.uint32)
        
        mesh = q3d.MeshItem(color='lightblue', color_mode='RGB', enable_lighting=True)
        mesh.set_data(verts, faces, colors)
    """
    def __init__(self, color='lightblue', wireframe=False, enable_lighting=True, color_mode='FLAT'):
        super(MeshItem, self).__init__()
        self.color = color
        self.flat_rgb = text_to_rgba(color, flat=True)
        self.wireframe = wireframe
        self.enable_lighting = enable_lighting
        
        # Mesh data
        self.triangles = None
        self.normals = None
        self.vertex_colors = None  # Per-vertex colors (uint32 IRGB format)
        
        self.mode_table = {'FLAT': 0, 'I': 1, 'RGB': 2}
        self.color_mode = self.mode_table[color_mode]
        self.vmin = 0
        self.vmax = 255
        
        # OpenGL objects
        self.vao = None
        self.vbo_vertices = None
        self.vbo_normals = None
        self.vbo_colors = None
        self.program = None
        
        # Rendering parameters
        self.line_width = 1.0
        self.light_pos = [1.0, 1.0, 1.0]
        self.light_color = [1.0, 1.0, 1.0]
        
        # Phong lighting material properties
        self.ambient_strength = 0.1
        self.diffuse_strength = 1.2
        self.specular_strength = 0.1
        self.shininess = 32.0
        # Alpha (opacity)
        self.alpha = 1.0
        
        # Buffer initialization flag
        self.need_update_buffer = True
        self.need_update_setting = True
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
    
        # Line width control
        line_width_label = QLabel("Line Width:")
        layout.addWidget(line_width_label)
        self.line_width_box = QDoubleSpinBox()
        self.line_width_box.setRange(0.5, 5.0)
        self.line_width_box.setSingleStep(0.5)
        self.line_width_box.setValue(self.line_width)
        self.line_width_box.valueChanged.connect(self.update_line_width)
        layout.addWidget(self.line_width_box)
        
        # Alpha control
        alpha_label = QLabel("Alpha:")
        layout.addWidget(alpha_label)
        alpha_box = QDoubleSpinBox()
        alpha_box.setRange(0.0, 1.0)
        alpha_box.setSingleStep(0.05)
        alpha_box.setValue(self.alpha)
        alpha_box.valueChanged.connect(self.update_alpha)
        layout.addWidget(alpha_box)
        

        # Color mode selection
        label_color = QLabel("Color Mode:")
        layout.addWidget(label_color)
        self.combo_color = QComboBox()
        self.combo_color.addItem("flat color")
        self.combo_color.addItem("intensity")
        self.combo_color.addItem("RGB")
        self.combo_color.setCurrentIndex(self.color_mode)
        self.combo_color.currentIndexChanged.connect(self._on_color_mode)
        layout.addWidget(self.combo_color)

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
            # Ambient strength control (slider 0-100 mapped to 0.0-1.0)
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

            # Diffuse strength control (slider 0-200 mapped to 0.0-2.0)
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

            # Specular strength control (slider 0-200 mapped to 0.0-2.0)
            specular_layout = QHBoxLayout()
            specular_label = QLabel("Specular Strength:")
            specular_layout.addWidget(specular_label)
            self.specular_slider = QSlider()
            self.specular_slider.setOrientation(1)
            self.specular_slider.setRange(0, 200)
            self.specular_slider.setValue(int(self.specular_strength * 100))
            self.specular_slider.valueChanged.connect(lambda v: self.update_specular_strength(v / 100.0))
            specular_layout.addWidget(self.specular_slider)
            layout.addLayout(specular_layout)

            # Shininess control (slider 1-256 mapped to 1-256)
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

    def _on_color_mode(self, index):
        self.color_mode = index
        self.edit_rgb.setVisible(index == self.mode_table['FLAT'])
        self.need_update_setting = True

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
            
    def set_data(self, verts, faces, colors=None):
        """
        verts: np.ndarray of shape (N, 3)
        faces: np.ndarray of shape (M, 3) with uint32 indices
        colors: np.ndarray of shape (N,) with uint32 IRGB format (optional)
                uint32 contains: I (bits 24-31), R (bits 16-23), G (bits 8-15), B (bits 0-7)
        """
        verts = np.asarray(verts, dtype=np.float32)
        faces = np.asarray(faces, dtype=np.uint32)
        triangles = verts[faces.flatten()]

        if colors is not None:
            colors = np.asarray(colors, dtype=np.uint32)
            if len(colors) == len(verts):
                # Expand per-vertex colors to per-triangle-vertex
                self.vertex_colors = colors[faces.flatten()]
            else:
                self.vertex_colors = None
        else:
            self.vertex_colors = None
            
        self.triangles = np.asarray(triangles, dtype=np.float32)
        self.normals = self.calculate_normals()
        self.need_update_buffer = True
        
    def calculate_normals(self):
        if self.triangles is None or len(self.triangles) == 0:
            return None
            
        # Ensure we have complete triangles
        num_vertices = len(self.triangles)
        num_triangles = num_vertices // 3
        if num_triangles == 0:
            return None
            
        # Reshape vertices into triangles (N, 3, 3) where N is number of triangles
        vertices_reshaped = self.triangles[:num_triangles * 3].reshape(-1, 3, 3)
        
        v0 = vertices_reshaped[:, 0, :]
        v1 = vertices_reshaped[:, 1, :]
        v2 = vertices_reshaped[:, 2, :]
        
        # Calculate edges for all triangles at once
        edge1 = v1 - v0
        edge2 = v2 - v0
        
        face_normals = np.cross(edge1, edge2)
        
        norms = np.linalg.norm(face_normals, axis=1, keepdims=True)
        norms[norms < 1e-6] = 1.0
        face_normals = face_normals / norms
        
        normals_per_vertex = np.repeat(face_normals[:, np.newaxis, :], 3, axis=1)
        normals = normals_per_vertex.reshape(-1, 3)        
        return normals.astype(np.float32)
            
    def initialize_gl(self):
        """OpenGL initialization"""
        vertex_shader = open(self.path + '/../shaders/mesh_vert.glsl', 'r').read()
        fragment_shader = open(self.path + '/../shaders/mesh_frag.glsl', 'r').read()

        program = shaders.compileProgram(
            shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER),
        )
        self.program = program

    def update_render_buffer(self):
        """Initialize OpenGL buffers"""
        if not self.need_update_buffer:
            return
            
        # Generate VAO and VBOs
        if self.vao is None:
            self.vao = glGenVertexArrays(1)
            self.vbo_vertices = glGenBuffers(1)
            self.vbo_normals = glGenBuffers(1)
            self.vbo_colors = glGenBuffers(1)
        
        glBindVertexArray(self.vao)
        
        # Vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_vertices)
        glBufferData(GL_ARRAY_BUFFER, self.triangles.nbytes, self.triangles, GL_STATIC_DRAW)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, None)
        
        # Normal buffer
        if self.normals is not None:
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo_normals)
            glBufferData(GL_ARRAY_BUFFER, self.normals.nbytes, self.normals, GL_STATIC_DRAW)
            glEnableVertexAttribArray(1)
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, None)
        
        # Color buffer (uint32 IRGB format)
        if self.vertex_colors is not None:
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo_colors)
            glBufferData(GL_ARRAY_BUFFER, self.vertex_colors.nbytes, self.vertex_colors, GL_STATIC_DRAW)
            glEnableVertexAttribArray(2)
            glVertexAttribIPointer(2, 1, GL_UNSIGNED_INT, 0, None)
        
        glBindVertexArray(0)
        self.need_update_buffer = False
        
    def update_setting(self):
        if (self.need_update_setting is False):
            return
        set_uniform(self.program, int(self.enable_lighting), 'if_light')
        set_uniform(self.program, 1, 'two_sided')
        set_uniform(self.program, np.array(self.light_color), 'light_color')
        set_uniform(self.program, float(self.ambient_strength), 'ambient_strength')
        set_uniform(self.program, float(self.diffuse_strength), 'diffuse_strength')
        set_uniform(self.program, float(self.specular_strength), 'specular_strength')
        set_uniform(self.program, float(self.shininess), 'shininess')
        set_uniform(self.program, float(self.alpha), 'alpha')
        set_uniform(self.program, int(self.flat_rgb), 'flat_rgb')
        set_uniform(self.program, int(self.color_mode), 'color_mode')
        set_uniform(self.program, float(self.vmin), 'vmin')
        set_uniform(self.program, float(self.vmax), 'vmax')
        self.need_update_setting = False

    def paint(self):
        """Render the mesh using modern OpenGL with shaders"""
        if self.triangles is None or len(self.triangles) == 0:
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
        
        # Bind VAO and render
        glBindVertexArray(self.vao)
        
        if len(self.triangles) > 0:
            # Render faces
            if self.wireframe:
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            else:
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
                
            # Draw triangles
            glDrawArrays(GL_TRIANGLES, 0, len(self.triangles))
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
                
        glBindVertexArray(0)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_BLEND)
        glUseProgram(0)
        
