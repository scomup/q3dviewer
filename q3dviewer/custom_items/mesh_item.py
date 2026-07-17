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
from q3dviewer.Qt.QtCore import Qt
import os
from q3dviewer.utils import set_uniform, text_to_rgba
import time


class MeshItem(BaseItem):
    """
    A dynamic OpenGL mesh item for rendering 3D triangular meshes.
    This item only supports keyed incremental updates (QUAD_DTYPE).
    Attributes:
        color (str or tuple): Accepts any valid matplotlib color (e.g., 'red', '#FF4500', (1.0, 0.5, 0.0)).
        wireframe (bool): If True, renders the mesh in wireframe mode.
    """
    # Class-level constants
    FACE_CAPACITY = 1000000    # Initial capacity for faces
    BIG_INT = 2**31 - 1         # Sentinel value for dirty region tracking
    QUAD_DTYPE = np.dtype([
        ('key', np.int64),
        ('vertices', np.float32, (12,)),
        ('good', np.uint8)
    ])
    
    def __init__(self, color='lightblue', wireframe=False):
        super(MeshItem, self).__init__()
        self.wireframe = wireframe
        self.color = color
        self.flat_rgb = text_to_rgba(color, flat=True)
        
        # Faces buffer: N x 13 numpy array
        # Each row: [v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z, good]
        self.faces = np.zeros((self.FACE_CAPACITY, 13), dtype=np.float32)
        
        # valid_f_top: pointer to end of valid faces
        self.valid_f_top = 0
        
        # Dirty region tracking for efficient GPU updates
        # dirty_min: start index of modified region (inclusive)
        # dirty_max: end index of modified region (exclusive)
        self.dirty_min = self.BIG_INT
        self.dirty_max = 0
        
        # key2index: mapping from face_key to face buffer index
        self.key2index = {}  # {face_key: face_index}
        
        # OpenGL objects
        self.vao = None
        self.vbo = None
        self.program = None
        self._gpu_face_capacity = 0    # Track GPU buffer capacity
        self._force_full_upload = False
        
        # Fixed rendering parameters (not adjustable via UI)
        self.enable_lighting = True
        self.line_width = 1.0
        self.light_pos = [1.0, 1.0, 1.0]
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
        # Only keep wireframe toggle - all other parameters are fixed
        self.wireframe_box = QCheckBox("Wireframe Mode")
        self.wireframe_box.setChecked(self.wireframe)
        self.wireframe_box.toggled.connect(self.update_wireframe)
        layout.addWidget(self.wireframe_box)

        # Enable lighting toggle
        self.lighting_box = QCheckBox("Enable Lighting")
        self.lighting_box.setChecked(self.enable_lighting)
        self.lighting_box.toggled.connect(self.update_enable_lighting)
        layout.addWidget(self.lighting_box)

        label_rgb = QLabel("Color:")
        label_rgb.setToolTip("Use hex color, i.e. #FF4500, or named color, i.e. 'red'")
        layout.addWidget(label_rgb)
        self.edit_rgb = QLineEdit()
        self.edit_rgb.setToolTip("Use hex color, i.e. #FF4500, or named color, i.e. 'red'")
        self.edit_rgb.setText(self.color)
        self.edit_rgb.textChanged.connect(self._on_color)
        layout.addWidget(self.edit_rgb)

        # Alpha transparency control
        alpha_layout = QHBoxLayout()
        alpha_label = QLabel("Alpha:")
        alpha_layout.addWidget(alpha_label)
        self.alpha_slider = QSlider()
        self.alpha_slider.setOrientation(Qt.Horizontal)
        self.alpha_slider.setRange(0, 100)
        self.alpha_slider.setValue(int(self.alpha * 100))
        self.alpha_slider.valueChanged.connect(lambda v: self.set_alpha(v / 100.0))
        alpha_layout.addWidget(self.alpha_slider)
        layout.addLayout(alpha_layout)

        # Material property controls for Phong lighting
        if self.enable_lighting:
            # Ambient strength control (slider 0-100 mapped to 0.0-1.0)
            ambient_layout = QHBoxLayout()
            ambient_label = QLabel("Ambient Strength:")
            ambient_layout.addWidget(ambient_label)
            self.ambient_slider = QSlider()
            self.ambient_slider.setOrientation(Qt.Horizontal)
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
            self.diffuse_slider.setOrientation(Qt.Horizontal)
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
            self.specular_slider.setOrientation(Qt.Horizontal)
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
            self.shininess_slider.setOrientation(Qt.Horizontal)
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

    def set_alpha(self, value):
        """Update mesh alpha (opacity)"""
        self.alpha = float(value)
        self.need_update_setting = True

    def set_data(self, data):
        """
        Set dynamic mesh data.

        Args:
            data: Structured numpy array with fields:
                  [('key', int64), ('vertices', float32, (12,)), ('good', uint8)]
        """
        if not isinstance(data, np.ndarray):
            raise ValueError("Data must be a numpy array")
        if data.dtype != self.QUAD_DTYPE:
            raise ValueError(
                "MeshItem only supports dynamic QUAD_DTYPE data: "
                "[('key', int64), ('vertices', float32, (12,)), ('good', uint8)]"
            )

        self.set_incremental_data(data)


    def set_incremental_data(self, fs):
        """
        Incrementally update mesh with new face data.
        Args:
            fs: Structured numpy array with dtype:
                [('key', np.int64), ('vertices', np.float32, (12,)), ('good', np.uint8)]
                - key: unique identifier for the face
                - vertices: 12 floats representing 4 vertices (v0, v1, v2, v3)
                - good: 0 or 1, whether to render this face
        Updates:
            - faces: updates existing faces or appends new ones
            - key2index: tracks face_key -> face_index mapping
        """
        if fs is None or len(fs) == 0:
            return
        
        if not isinstance(fs, np.ndarray) or fs.dtype.names is None:
            raise ValueError("fs must be a structured numpy array with fields: key, vertices, good")

        # Prepare face data: convert structured array to Nx13 format
        n_faces = len(fs)
        face_data = np.zeros((n_faces, 13), dtype=np.float32)
        
        # Copy vertices (12 floats -> positions 0:12)
        face_data[:, :12] = fs['vertices']
        
        # Copy good flag (position 12)
        face_data[:, 12] = fs['good'].astype(np.float32)
        
        # Extract keys
        keys = fs['key']
        
        # Optimization: Separate updates from new insertions
        update_mask = np.array([key in self.key2index for key in keys], dtype=bool)
        new_mask = ~update_mask

        # Ensure enough capacity only for truly new faces.
        n_new = int(np.count_nonzero(new_mask))
        expanded = False
        while self.valid_f_top + n_new > len(self.faces):
            self._expand_face_buffer()
            expanded = True
        if expanded:
            # VBO will be reallocated on next frame; old contents must be re-uploaded.
            self._force_full_upload = True
        
        # Batch update existing faces
        if np.any(update_mask):
            update_keys = keys[update_mask]
            update_indices = np.array([self.key2index[key] for key in update_keys], dtype=np.int32)
            self.faces[update_indices] = face_data[update_mask]
            
            # Update dirty region for modified faces
            self.dirty_min = min(self.dirty_min, int(np.min(update_indices)))
            self.dirty_max = max(self.dirty_max, int(np.max(update_indices) + 1))
            self.need_update_buffer = True
        
        # Batch insert new faces
        if np.any(new_mask):
            new_keys = keys[new_mask]
            new_face_data = face_data[new_mask]
            n_new = len(new_keys)
            
            # Update dirty region for new faces
            start_index = self.valid_f_top            
            # Insert data
            self.faces[start_index: start_index + n_new] = new_face_data
            
            # Update key2index mapping for new faces
            for i, face_key in enumerate(new_keys):
                self.key2index[face_key] = start_index + i
            self.valid_f_top += n_new
            self.dirty_min = min(self.dirty_min, start_index)
            self.dirty_max = max(self.dirty_max, start_index + n_new)
            self.need_update_buffer = True
    
    def _expand_face_buffer(self):
        """Expand the faces buffer when capacity is reached"""
        new_capacity = len(self.faces) + self.FACE_CAPACITY
        new_buffer = np.zeros((new_capacity, 13), dtype=np.float32)
        new_buffer[:len(self.faces)] = self.faces
        self.faces = new_buffer
    
    def clear_mesh(self):
        """Clear all mesh data and reset buffers"""
        self.valid_f_top = 0
        self.dirty_min = self.BIG_INT
        self.dirty_max = 0
        self.key2index.clear()
        if hasattr(self, 'indices_array'):
            self.indices_array = np.array([], dtype=np.uint32)

    def initialize_gl(self):
        """OpenGL initialization"""
        # Use instanced mesh shaders with geometry shader for GPU-side triangle generation
        vert_shader = open(self.path + '/../shaders/mesh_vert.glsl', 'r', encoding='utf-8').read()
        geom_shader = open(self.path + '/../shaders/mesh_geom.glsl', 'r', encoding='utf-8').read()
        frag_shader = open(self.path + '/../shaders/mesh_frag.glsl', 'r', encoding='utf-8').read()
        try:
            program = shaders.compileProgram(
                shaders.compileShader(vert_shader, GL_VERTEX_SHADER),
                shaders.compileShader(geom_shader, GL_GEOMETRY_SHADER),
                shaders.compileShader(frag_shader, GL_FRAGMENT_SHADER),
            )
            self.program = program
        except Exception as e:
            raise

    def update_render_buffer(self):
        """
        Update GPU buffer with face data (no separate vertex buffer).
        Each face contains embedded vertex positions (13 floats).
        Geometry shader generates triangles on GPU from face vertices.
        Dynamically resizes GPU buffer when Python buffer expands.
        """
        if self.valid_f_top == 0:
            return
        
        # Initialize buffers on first call
        if self.vao is None:
            self.vao = glGenVertexArrays(1)
            self.vbo = glGenBuffers(1)
            self._gpu_face_capacity = 0
        
        # Check if we need to reallocate VBO for faces
        vbo_reallocated = False
        if self._gpu_face_capacity < len(self.faces):
            glBindVertexArray(self.vao)
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferData(GL_ARRAY_BUFFER,
                        self.faces.nbytes,
                        None,
                        GL_DYNAMIC_DRAW)
            
            # Setup face attributes (per-instance)
            # Face data: [v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z, good]
            # 13 floats = 52 bytes stride
            
            # v0 (location 1) - vec3
            glEnableVertexAttribArray(1)
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 52, ctypes.c_void_p(0))
            glVertexAttribDivisor(1, 1)
            
            # v1 (location 2) - vec3
            glEnableVertexAttribArray(2)
            glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 52, ctypes.c_void_p(12))
            glVertexAttribDivisor(2, 1)
            
            # v2 (location 3) - vec3
            glEnableVertexAttribArray(3)
            glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 52, ctypes.c_void_p(24))
            glVertexAttribDivisor(3, 1)
            
            # v3 (location 4) - vec3
            glEnableVertexAttribArray(4)
            glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 52, ctypes.c_void_p(36))
            glVertexAttribDivisor(4, 1)
            
            # good flag (location 5) - float
            glEnableVertexAttribArray(5)
            glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, 52, ctypes.c_void_p(48))
            glVertexAttribDivisor(5, 1)
            
            glBindVertexArray(0)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            self._gpu_face_capacity = len(self.faces)
            vbo_reallocated = True
        
        # Upload faces to VBO (only dirty region)
        if self.need_update_buffer:
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)

            # After VBO realloc or CPU buffer expansion, re-upload full valid range.
            if self._force_full_upload or vbo_reallocated:
                start_index = 0
                end_index = int(self.valid_f_top)
            else:
                # Calculate the dirty range [dirty_min, dirty_max)
                start_index = int(self.dirty_min)
                end_index = int(self.dirty_max)
            count = end_index - start_index

            if count > 0:
                # Upload only modified region (or full valid region when required)
                glBufferSubData(GL_ARRAY_BUFFER,
                               start_index * 13 * 4,  # offset in bytes
                               count * 13 * 4,         # size in bytes
                               self.faces[start_index:end_index])
            
            glBindBuffer(GL_ARRAY_BUFFER, 0)
            self.need_update_buffer = False
            self._force_full_upload = False
            
            # Reset dirty region
            self.dirty_min = self.BIG_INT
            self.dirty_max = 0
        
    def update_setting(self):
        """Set fixed rendering parameters (called once during initialization)"""
        if not self.need_update_setting:
            return
        # Set fixed uniforms for instanced shaders
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
        Render the mesh using instanced rendering with geometry shader.
        Each face instance is rendered as a point, geometry shader generates 2 triangles.
        GPU filters faces based on good flag.
        """
        if self.valid_f_top == 0:
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
        
        # Adjust depth test based on alpha transparency
        if self.alpha < 0.95:
            glDepthFunc(GL_ALWAYS)
        else:
            glDepthFunc(GL_LESS)
        
        glDisable(GL_CULL_FACE)  # two-sided rendering
        
        # Set line width
        glLineWidth(self.line_width)
        
        # Bind VAO (vertex positions are now in VBO attributes)
        glBindVertexArray(self.vao)
                
        if self.wireframe:
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        else:
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        
        # Draw using instanced rendering
        # Input: POINTS (one per face instance)
        # Geometry shader generates 2 triangles (6 vertices) per point
        glDrawArraysInstanced(GL_POINTS, 0, 1, self.valid_f_top)
            
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
        glBindVertexArray(0)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_BLEND)
        glUseProgram(0)
        
