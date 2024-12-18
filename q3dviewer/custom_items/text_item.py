"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from PyQt5 import QtCore, QtGui
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *


class GL2DTextItem(BaseItem):
    """Draws text over opengl 3D."""

    def __init__(self, **kwds):
        """All keyword arguments are passed to setData()"""
        BaseItem.__init__(self)
        glopts = kwds.pop('glOptions', 'additive')
        self.setGLOptions(glopts)
        self.pos = (100, 100)
        self.color = QtCore.Qt.GlobalColor.white
        self.text = ''
        self.font = QtGui.QFont('Helvetica', 16)
        self.setData(**kwds)

    def setData(self, **kwds):
        args = ['pos', 'color', 'text', 'size', 'font']
        for k in kwds.keys():
            if k not in args:
                raise ValueError('Invalid keyword argument: %s\
                    (allowed arguments are %s)' % (k, str(args)))
        for arg in args:
            if arg in kwds:
                value = kwds[arg]
                if arg == 'pos':
                    self.pos = value
                elif arg == 'color':
                    value = value
                elif arg == 'font':
                    if isinstance(value, QtGui.QFont) is False:
                        raise TypeError('"font" must be QFont.')
                elif arg == 'size':
                    self.font.setPointSize(value)
                setattr(self, arg, value)
        self.update()

    def paint(self):
        if len(self.text) < 1:
            return

        text_pos = QtCore.QPointF(*self.pos)
        painter = QtGui.QPainter(self.view())
        painter.setPen(self.color)
        painter.setFont(self.font)
        painter.setRenderHints(QtGui.QPainter.RenderHint.Antialiasing |
                               QtGui.QPainter.RenderHint.TextAntialiasing)
        painter.drawText(text_pos, self.text)
        painter.end()
