"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from PyQt5 import QtCore
import numpy as np

class BaseItem(QtCore.QObject):
    _next_id = 0
    
    def __init__(self):
        super().__init__()
        self._id = BaseItem._next_id
        BaseItem._next_id += 1
        self.__view = None
        self.__transform = np.eye(4)
        self.__visible = True
        self.__initialized = False
        
    def set_glwidget(self, v):
        self.__view = v
        
    def view(self):
        return self.__view

    def transform(self):
        return self.__transform
    
    def hide(self):
        self.__visible = False
        
    def show(self):
        self.__visible = True
    
    def set_visible(self, vis):
        self.__visible = vis
        
    def visible(self):
        return self.__visible
    
    def initialize(self):
        if not self.__initialized:
            self.initialize_gl()
    
    def initialize_gl(self):
        """
        Initialize OpenGL resources for the item.
        This method should be overridden by subclasses to set up any necessary OpenGL resources.
        """
        pass

    def paint(self):
        """
        Render the item using OpenGL.
        This method should be overridden by subclasses to perform the actual rendering.
        """
        pass



