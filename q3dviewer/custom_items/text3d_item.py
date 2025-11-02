"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""


from turtle import position
from q3dviewer.base_item import BaseItem
from OpenGL.GL import *
from OpenGL.GLUT import glutBitmapCharacter, glutInit
from OpenGL.GLUT import (
    GLUT_BITMAP_HELVETICA_10,
    GLUT_BITMAP_HELVETICA_12,
    GLUT_BITMAP_HELVETICA_18,
    # GLUT_BITMAP_TIMES_ROMAN_10,
    GLUT_BITMAP_TIMES_ROMAN_24,
)

def get_glut_font(font_size):
    # Map requested font_size to a GLUT font object
    if font_size <= 10:
        return GLUT_BITMAP_HELVETICA_10
    elif font_size <= 12:
        return GLUT_BITMAP_HELVETICA_12
    elif font_size <= 18:
        return GLUT_BITMAP_HELVETICA_18
    elif font_size <= 24:
        return GLUT_BITMAP_TIMES_ROMAN_24
    else:
        return GLUT_BITMAP_TIMES_ROMAN_24  # Largest available
    

# draw points with color (x, y, z, color)
class Text3DItem(BaseItem):
    """
    A OpenGL 3d text and mark item.

    Attributes:
        data: list storing text data. Each element is a dict with keys:
            'text': str, the text to display
            'position': (x, y, z), the 3D position of the text
            'color': (r, g, b, a), the color of the text
            'font_size': float, the font size of the text
            'point_size': float, size of point to draw at position (0 for no point)
            'line_width': float, width of line to draw between points (0 for no line)

    """
    def __init__(self, data=[]):
        super().__init__()
        self._disable_setting = True
        self.data_list = data # map of {'text': str, 'position': (x,y,z), 'color': (r,g,b,a), 'size': float}

    def add_setting(self, layout):
        pass # No settings for Text3DItem

    def set_data(self, data, append=False):
        if not append:
            self.data_list = []
        self.data_list.extend(data)

    def clear_data(self):
        self.data_list = []


    def initialize_gl(self):
        glutInit()
        # super().initialize_gl()


    def paint(self):
        for item in self.data_list:
            # Handle both dictionary and string formats
            if isinstance(item, dict):
                text = item.get('text', '')
                pos = item.get('position', (0.0, 0.0, 0.0))
                font_size = item.get('font_size', 24)
                color = item.get('color', (1.0, 1.0, 1.0, 1.0))
                point_size = item.get('point_size', 0.0)
            elif isinstance(item, str):
                # If item is a string, treat it as text with default position and color
                text = item
                pos = (0.0, 0.0, 0.0)
                color = (1.0, 1.0, 1.0, 1.0)
                font_size = 24
                point_size = 0.0
            else:
                print(f"Warning: Unsupported item type: {type(item)}")
                continue

            glColor4f(*color)
            offset = 0.02
            pos_text = (pos[0] + offset, pos[1]+ offset, pos[2]+ offset)
            glRasterPos3f(*pos_text)

            if point_size > 0.0:
                # draw a point at the position
                glPointSize(point_size)
                glBegin(GL_POINTS)
                glVertex3f(*pos)
                glEnd()
            font = get_glut_font(font_size)
            for ch in text:
                glutBitmapCharacter(font, ord(ch))

        # draw lines between points
        for i in range(len(self.data_list) - 1):
            item1 = self.data_list[i]
            item2 = self.data_list[i + 1]
            if isinstance(item1, dict) and isinstance(item2, dict):
                pos1 = item1.get('position', (0.0, 0.0, 0.0))
                pos2 = item2.get('position', (0.0, 0.0, 0.0))
                line_width = item1.get('line_width', 0.0)
                color = item1.get('color', (1.0, 1.0, 1.0, 1.0))
                if line_width > 0.0:
                    glColor4f(*color)
                    glLineWidth(line_width)
                    glBegin(GL_LINES)
                    glVertex3f(*pos1)
                    glVertex3f(*pos2)
                    glEnd()