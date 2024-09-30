import pyqtgraph.opengl as gl
from OpenGL.GL import *
import numpy as np
import threading


class TrajectoryItem(gl.GLGridItem):
    def __init__(self, width=1, color=(0, 1, 0, 1)):
        super(TrajectoryItem, self).__init__()
        self.width = width
        self.buff = np.empty((0, 3), np.float32)
        self.wait_add_data = None
        self.mutex = threading.Lock()
        self.CAPACITY = 100000
        self.valid_buff_top = 0
        self.color = color

    def addSetting(self, layout):
        pass

    def setData(self, data, append=True):
        self.mutex.acquire()
        data = data.astype(np.float32).reshape(-1, 3)
        if (append is False):
            self.wait_add_data = data
            self.add_buff_loc = 0
        else:
            if (self.wait_add_data is None):
                self.wait_add_data = data
            else:
                self.wait_add_data = np.concatenate([self.wait_add_data, data])
            self.add_buff_loc = self.valid_buff_top
        self.mutex.release()


    def updateRenderBuffer(self):
        if(self.wait_add_data is None):
            return
        self.mutex.acquire()

        new_buff_top = self.add_buff_loc + self.wait_add_data.shape[0]
        if new_buff_top > self.buff.shape[0]:
            buff_capacity = self.buff.shape[0]
            while (new_buff_top > buff_capacity):
                buff_capacity += self.CAPACITY
            self.buff = np.empty((buff_capacity, 3), np.float32)
            self.buff[self.add_buff_loc:new_buff_top] = self.wait_add_data
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferData(GL_ARRAY_BUFFER, self.buff.nbytes, self.buff, GL_DYNAMIC_DRAW)
            glBindBuffer(GL_ARRAY_BUFFER, 0)
        else:
            self.buff[self.add_buff_loc:new_buff_top] = self.wait_add_data
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
            glBufferSubData(GL_ARRAY_BUFFER, self.add_buff_loc * 12,
                            self.wait_add_data.shape[0] * 12, self.wait_add_data)
        self.valid_buff_top = new_buff_top
        self.wait_add_data = None
        self.mutex.release()

    def initializeGL(self):
        self.vbo = glGenBuffers(1)


    def paint(self):
        self.setupGLState()
        self.updateRenderBuffer()
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointer(3, GL_FLOAT, 0, None)
        glLineWidth(self.width)
        glColor4f(*self.color)  # z is blue

        glDrawArrays(GL_LINE_STRIP, 0, self.valid_buff_top)
        glDisableClientState(GL_VERTEX_ARRAY)

        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glUseProgram(0)
