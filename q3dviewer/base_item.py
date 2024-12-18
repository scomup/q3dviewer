from PyQt5 import QtCore
from pyqtgraph import Transform3D


class BaseItem(QtCore.QObject):  # Renamed BaseGLItem to BaseItem
    _nextId = 0
    
    def __init__(self):
        super().__init__()
        self._id = BaseItem._nextId
        BaseItem._nextId += 1
        self.__view = None
        self.__children: set[BaseItem] = set()
        self.__transform = Transform3D()
        self.__visible = True
        self.__initialized = False
   
    def depthValue(self):
        return 0
    
    def parentItem(self):
        return None
        
    def childItems(self):
        return list()
        
    def _setView(self, v):
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



