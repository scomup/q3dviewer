"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from PySide6 import QtCore
import numpy as np

class BaseItem(QtCore.QObject):
    _next_id = 0
    
    def __init__(self):
        super().__init__()
        self._id = BaseItem._next_id
        BaseItem._next_id += 1
        self._glwidget = None
        self._visible = True
        self._initialized = False
        self._disable_setting = False
        
    def set_glwidget(self, v):
        self._glwidget = v
        
    def glwidget(self):
        return self._glwidget
    
    def hide(self):
        self._visible = False
        
    def show(self):
        self._visible = True
    
    def set_visible(self, vis):
        self._visible = vis
        
    def visible(self):
        return self._visible
    
    def initialize(self):
        if not self._initialized:
            self.initialize_gl()
            self._initialized = True

    def is_initialized(self):
        return self._initialized

    def add_setting(self, layout):
        """
        Add setting widgets to the layout.
        This method should be overridden by subclasses to add any necessary setting widgets to the layout.
        """
        pass
    
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

    def disable_setting(self):
        self._disable_setting = True



