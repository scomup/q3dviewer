"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.Qt import QtCore, QtGui
from q3dviewer.base_item import BaseItem
from q3dviewer.utils import text_to_rgba


class Text2DItem(BaseItem):
    """
    A OpenGL 2D text item.

    Attributes:
        pos: (x, y), the 2D position of the text in pixels
        text: str, the text to display
        font: QFont, the font of the text
        color: str, the color of the text in matplotlib format
        size: int, the font size of the text
    """

    def __init__(self, **kwds):
        """All keyword arguments are passed to set_data()"""
        BaseItem.__init__(self)
        self.pos = (20, 50)
        self.text = ''
        self.font = QtGui.QFont('Helvetica', 16)
        self.rgb = text_to_rgba('white')
        if 'pos' in kwds:
            self.pos = kwds['pos']
        if 'text' in kwds:
            self.text = kwds['text']
        if 'font' in kwds:
            self.font = kwds['font']
        if 'color' in kwds:
            self.set_color(kwds['color'])
        if 'size' in kwds:
            self.font.setPointSize(kwds['size'])

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
            self.rgb = text_to_rgba(color)
        except ValueError:
            print("Invalid color format. Use mathplotlib color format.")

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
