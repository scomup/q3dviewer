"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from PySide6 import QtCore, QtGui
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
from q3dviewer.utils.maths import hex_to_rgba


class Text2DItem(BaseItem):
    """Draws text over opengl 3D."""

    def __init__(self, **kwds):
        """All keyword arguments are passed to set_data()"""
        BaseItem.__init__(self)
        self.pos = (20, 50)
        self.color = '#ffffff'
        self.rgb = hex_to_rgba(self.color)
        self.text = ''
        self.font = QtGui.QFont('Helvetica', 16)
        self.set_data(**kwds)

    def set_data(self, **kwds):
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
                    self.set_color(value)
                elif arg == 'font':
                    if isinstance(value, QtGui.QFont) is False:
                        raise TypeError('"font" must be QFont.')
                elif arg == 'size':
                    self.font.setPointSize(value)
                setattr(self, arg, value)

    def set_color(self, color):
        try:
            self.rgb = hex_to_rgba(color)
            self.color = color
        except ValueError:
            pass

    def paint(self):
        if len(self.text) < 1:
            return

        text_pos = QtCore.QPointF(*self.pos)
        painter = QtGui.QPainter(self.glwidget())
        painter.setPen(QtGui.QColor(*[int(c * 255) for c in self.rgb[:3]]))
        painter.setFont(self.font)
        painter.setRenderHints(QtGui.QPainter.RenderHint.Antialiasing |
                               QtGui.QPainter.RenderHint.TextAntialiasing)
        painter.drawText(text_pos, self.text)
        painter.end()
