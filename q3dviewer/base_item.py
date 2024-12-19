from PyQt5 import QtCore
import numpy as np

class BaseItem(QtCore.QObject):  # Renamed BaseGLItem to BaseItem
    _next_id = 0
    
    def __init__(self):
        super().__init__()
        self._id = BaseItem._next_id
        BaseItem._next_id += 1
        self.__view = None
        self.__transform = np.eye(4)
        self.__visible = True
        self.__initialized = False
       
    def parent_item(self):
        return None
        
    def child_items(self):
        return list()
        
    def _set_view(self, v):
        self.__view = v
        
    def view(self):
        return self.__view

    def transform(self):
        return self.__transform
    
    def hide(self):
        self.__visible = False
        
    def show(self):
        self.__visible = True
    
    def setVisible(self, vis):
        self.__visible = vis
        
    def visible(self):
        return self.__visible
    
    def initialize(self):
        self.initializeGL()
        self.__initialized = True

    def isInitialized(self):
        return self.__initialized
    
    def initializeGL(self):
        pass

    def paint(self):
        pass



