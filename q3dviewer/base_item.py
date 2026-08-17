"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""
from q3dviewer.Qt.QtCore import QObject


class BaseItem(QObject):
    _next_id = 0
    
    def __init__(self):
        super().__init__()
        self._id = BaseItem._next_id
        BaseItem._next_id += 1
        self._glwidget = None
        self._visible = True
        self._initialized = False
        self._disable_setting = False
        self._is_setting_changed = True
        
    def set_glwidget(self, v):
        self._glwidget = v
        self.notify_changed()
        
    def glwidget(self):
        return self._glwidget
    
    def hide(self):
        self._visible = False
        self.notify_changed()
        
    def show(self):
        self._visible = True
        self.notify_changed()
    
    def set_visible(self, vis):
        self._visible = vis
        self.notify_changed()
        
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

    def notify_changed(self):
        self._is_setting_changed = True
        glwidget = self.glwidget()
        if glwidget is not None and glwidget.auto_update:
            glwidget.mark_view_dirty()

    def is_setting_changed(self):
        return self._is_setting_changed

    def clear_setting_changed(self):
        self._is_setting_changed = False



