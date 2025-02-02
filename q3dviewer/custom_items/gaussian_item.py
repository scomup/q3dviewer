"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
import numpy as np
import os
from PySide6.QtWidgets import QComboBox, QLabel
from OpenGL.GL import shaders
from q3dviewer.utils import *


def div_round_up(x, y):
    return int((x + y - 1) / y)


class GaussianItem(BaseItem):
    def __init__(self, **kwds):
        super().__init__()
        self.need_updateGS = False
        self.sh_dim = 0
        self.gs_data = np.empty([0])
        self.prev_Rz = np.array([np.inf, np.inf, np.inf])
        self.path = os.path.dirname(__file__)
        try:
            import torch
            if not torch.cuda.is_available():
                raise ImportError
            self.cuda_pw = None
            self.sort = self.torch_sort
        except ImportError:
            self.sort = self.openg_sort

    def add_setting(self, layout):
        label_render_mode = QLabel("Render Mode:")
        layout.addWidget(label_render_mode)
        combo = QComboBox()
        combo.addItem("render normal guassian")
        combo.addItem("render ball")
        combo.addItem("render inverse guassian")
        combo.currentIndexChanged.connect(self.onComboboxSelection)
        layout.addWidget(combo)

    def onComboboxSelection(self, index):
        glUseProgram(self.program)
        set_uniform(self.program, index, 'render_mod')
        glUseProgram(0)

    def initialize_gl(self):
        fragment_shader = open(
            self.path + '/../shaders/gau_frag.glsl', 'r').read()
        vertex_shader = open(
            self.path + '/../shaders/gau_vert.glsl', 'r').read()
        sort_shader = open(
            self.path + '/../shaders/sort_by_key.glsl', 'r').read()
        prep_shader = open(self.path + '/../shaders/gau_prep.glsl', 'r').read()

        self.sort_program = shaders.compileProgram(
            shaders.compileShader(sort_shader, GL_COMPUTE_SHADER))

        self.prep_program = shaders.compileProgram(
            shaders.compileShader(prep_shader, GL_COMPUTE_SHADER))

        self.program = shaders.compileProgram(
            shaders.compileShader(vertex_shader, GL_VERTEX_SHADER),
            shaders.compileShader(fragment_shader, GL_FRAGMENT_SHADER),
        )
        self.vao = glGenVertexArrays(1)

        # trade a gaussian as a square (4 2d points)
        square_vert = np.array([-1, 1, 1, 1, 1, -1, -1, -1], dtype=np.float32)
        indices = np.array([0, 1, 2, 0, 2, 3], dtype=np.uint32)

        # set the vertices for square
        vbo = glGenBuffers(1)
        glBindVertexArray(self.vao)
        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(GL_ARRAY_BUFFER, square_vert.nbytes,
                     square_vert, GL_STATIC_DRAW)
        pos = glGetAttribLocation(self.program, 'vert')
        glVertexAttribPointer(pos, 2, GL_FLOAT, False, 0, None)
        glEnableVertexAttribArray(pos)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        # the vert's indices for drawing square
        self.ebo = glGenBuffers(1)
        glBindVertexArray(self.vao)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     indices.nbytes, indices, GL_STATIC_DRAW)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        glBindVertexArray(0)

        # add SSBO for gaussian data
        self.ssbo_gs = glGenBuffers(1)
        self.ssbo_gi = glGenBuffers(1)
        self.ssbo_dp = glGenBuffers(1)
        self.ssbo_pp = glGenBuffers(1)

        width = self.glwidget().current_width()
        height = self.glwidget().current_height()

        # set constant parameter for gaussian shader
        project_matrix = self.glwidget().get_projection_matrix()
        focal_x = project_matrix[0, 0] * width / 2
        focal_y = project_matrix[1, 1] * height / 2
        glUseProgram(self.prep_program)
        set_uniform(self.prep_program,
                    project_matrix, 'projection_matrix')
        set_uniform(self.prep_program, np.array([focal_x, focal_y]), 'focal')
        glUseProgram(0)

        glUseProgram(self.program)
        set_uniform(self.program, np.array([width, height]), 'win_size')
        set_uniform(self.program, 0, 'render_mod')
        glUseProgram(0)

        # opengl settings
        glDisable(GL_CULL_FACE)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    def updateGS(self):
        if (self.need_updateGS):
            # compute sorting size
            self.num_sort = int(2**np.ceil(np.log2(self.gs_data.shape[0])))

            # set input gaussian data
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.ssbo_gs)
            glBufferData(GL_SHADER_STORAGE_BUFFER, self.gs_data.nbytes,
                         self.gs_data.reshape(-1), GL_STATIC_DRAW)
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, self.ssbo_gs)
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0)

            # set depth for sorting
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.ssbo_dp)
            glBufferData(GL_SHADER_STORAGE_BUFFER,
                         self.num_sort * 4, None, GL_STATIC_DRAW)
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, self.ssbo_dp)
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0)

            # set index for sorting (the index need be initialized)
            gi = np.arange(self.num_sort, dtype=np.uint32)
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.ssbo_gi)
            glBufferData(GL_SHADER_STORAGE_BUFFER,
                         self.num_sort * 4, gi, GL_STATIC_DRAW)
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.ssbo_gi)
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0)

            # set preprocess buffer
            # the dim of preprocess data is 12 u(3),
            # covinv(3), color(3), area(2), alpha(1)
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.ssbo_pp)
            glBufferData(GL_SHADER_STORAGE_BUFFER,
                         self.gs_data.shape[0] * 4 * 12,
                         None, GL_STATIC_DRAW)
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, self.ssbo_pp)
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0)

            glUseProgram(self.prep_program)
            set_uniform(self.prep_program, self.sh_dim, 'sh_dim')
            set_uniform(self.prep_program,
                        self.gs_data.shape[0], 'gs_num')
            glUseProgram(0)
            self.need_updateGS = False

    def paint(self):
        # get current view matrix
        self.view_matrix = self.glwidget().view_matrix

        # if gaussian data is update, renew vao, ssbo, etc...
        self.updateGS()

        if (self.gs_data.shape[0] == 0):
            return

        # preprocess and sort gaussian by compute shader.
        self.preprocessGS()
        self.try_sort()
        glEnable(GL_BLEND)
        # draw by vert shader
        glUseProgram(self.program)
        # bind vao and ebo
        glBindVertexArray(self.vao)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        # draw instances
        glDrawElementsInstanced(
            GL_TRIANGLES, 6, GL_UNSIGNED_INT, None, self.gs_data.shape[0])
        # upbind vao and ebo
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)
        glBindVertexArray(0)
        glUseProgram(0)
        glDisable(GL_BLEND)

    def try_sort(self):
        # don't sort if the depths are not change.
        Rz = self.view_matrix[2, :3]
        if (np.linalg.norm(self.prev_Rz - Rz) > 0.1):
            # import torch
            # torch.cuda.synchronize()
            # start = time.time()
            self.sort()
            self.prev_Rz = Rz
            # torch.cuda.synchronize()
            # end = time.time()
            # time_diff = end - start
            # print(time_diff)

    def openg_sort(self):
        glUseProgram(self.sort_program)
        # can we move this loop to gpu?
        # level = level*2
        for level in 2**np.arange(1, int(np.ceil(np.log2(self.num_sort))+1)):
            # stage =stage / 2
            for stage in level/2**np.arange(1, np.log2(level)+1):
                set_uniform(self.sort_program, int(level), 'level')
                set_uniform(self.sort_program, int(stage), 'stage')
                glDispatchCompute(div_round_up(self.num_sort//2, 256), 1, 1)
                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)
        # glFinish()
        glUseProgram(0)

    def torch_sort(self):
        import torch
        if self.cuda_pw is None:
            self.cuda_pw = torch.tensor(self.gs_data[:, :3]).cuda()
        Rz = torch.tensor(self.view_matrix[2, :3].astype(np.float32)).cuda()
        depth = Rz @ self.cuda_pw.T
        index = torch.argsort(depth).type(torch.int32).cpu().numpy()
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, self.ssbo_gi)
        glBufferData(GL_SHADER_STORAGE_BUFFER,
                     index.nbytes, index, GL_STATIC_DRAW)
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, self.ssbo_gi)
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0)
        return index

    def preprocessGS(self):
        glUseProgram(self.prep_program)
        set_uniform(self.prep_program, self.view_matrix, 'view_matrix')
        glDispatchCompute(div_round_up(self.gs_data.shape[0], 256), 1, 1)
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT)
        glUseProgram(0)

    def set_data(self, **kwds):
        if 'gs_data' in kwds:
            self.need_updateGS = False
            gs_data = kwds.pop('gs_data')
            self.gs_data = np.ascontiguousarray(gs_data, dtype=np.float32)
            self.sh_dim = self.gs_data.shape[-1] - (3 + 4 + 3 + 1)
            self.prev_Rz = np.array([np.inf, np.inf, np.inf])
            self.cuda_pw = None
            self.need_updateGS = True
