from OpenGL.GL import *  # noqa
from math import radians, tan
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from q3dviewer.utils import frustum, euler_to_matrix, makeT, m_get_roll


class BaseGLWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        QtWidgets.QOpenGLWidget.__init__(self, parent)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
        self.reset()
        self._fov = 60
        self.items = []
        self.keyTimer = QtCore.QTimer()
        self.color = np.array([0, 0, 0, 0])
        self.Twb = makeT(euler_to_matrix([0, 0, 0]), np.array([-0, -50, 20]))
        self.Tbc = makeT(euler_to_matrix([np.pi/3, 0, 0]), np.array([0, 0, 0]))
        self.active_keys = set()

    def keyPressEvent(self, ev: QtGui.QKeyEvent):
        if ev.key() == QtCore.Qt.Key_Up or  \
            ev.key() == QtCore.Qt.Key_Down or \
            ev.key() == QtCore.Qt.Key_Left or \
            ev.key() == QtCore.Qt.Key_Right or \
            ev.key() == QtCore.Qt.Key_Z or \
            ev.key() == QtCore.Qt.Key_X or \
            ev.key() == QtCore.Qt.Key_A or \
            ev.key() == QtCore.Qt.Key_D or \
            ev.key() == QtCore.Qt.Key_W or \
            ev.key() == QtCore.Qt.Key_S:
            self.active_keys.add(ev.key())
        self.active_keys.add(ev.key())

    def keyReleaseEvent(self, ev: QtGui.QKeyEvent):
        self.active_keys.discard(ev.key())

    def current_width(self):
        """
        Return the current width of the widget.
        """
        return int(self.width() * self.devicePixelRatioF())

    def current_height(self):
        """
        Return the current height of the widget.
        """
        return int(self.height() * self.devicePixelRatioF())

    def reset(self):
        pass

    def add_item(self, item):
        """
        Add the item to the glwidget.
        """
        self.items.append(item)
        item.set_glwidget(self)
        self.update()
        
    def remove_item(self, item):
        """
        Remove the item from the glwidget.
        """
        self.items.remove(item)
        item.set_glwidget(None)
        self.update()

    def clear(self):
        """
        Remove all items from the glwidget.
        """
        for item in self.items:
            item.set_glwidget(None)
        self.items = []
        self.update()        
        
    def initializeGL(self):
        """
        the method is herted from QOpenGLWidget, 
        and it is called when the widget is first shown.
        """
        for item in self.items:
            item.initialize()

    def mouseReleaseEvent(self, ev):
        if hasattr(self, 'mousePos'):
            delattr(self, 'mousePos')

    def wheelEvent(self, ev):
        delta = ev.angleDelta().x()
        if delta == 0:
            delta = ev.angleDelta().y()
        delta = self.get_z() * delta * 0.003
        self.Twb[:3, 3] += self.Twb[:3, :3] @ self.Tbc[:3, :3] @ np.array([0, 0, -delta])
        self.update()

    def mouseMoveEvent(self, ev):
        lpos = ev.localPos()
        if not hasattr(self, 'mousePos'):
            self.mousePos = lpos
        diff = lpos - self.mousePos
        self.mousePos = lpos
        if ev.buttons() == QtCore.Qt.MouseButton.RightButton:
            dR = euler_to_matrix([-diff.y() * 0.005, 0, 0])
            self.Tbc[:3, :3] = self.Tbc[:3, :3] @ dR
            dR = euler_to_matrix([0, 0, -diff.x() * 0.005])
            self.Twb[:3, :3] = self.Twb[:3, :3] @ dR
        elif ev.buttons() == QtCore.Qt.MouseButton.LeftButton:
            fx, fy = self.get_focal()
            z = self.get_z()
            roll = np.abs(m_get_roll(self.Tbc))
            if (roll < 1.3):
                dtrans = np.array([-diff.x() * z / fx, diff.y() * z / fy, 0])
                self.Twb[:3, 3] += self.Twb[:3, :3] @ dtrans
                print(f"Translating by {dtrans}")
            else:
                dtrans = np.array([-diff.x() * 100 / fx, 0, diff.y() * 100 / fy])
                self.Twb[:3, 3] += self.Twb[:3, :3] @ dtrans
                print(f"2 Translating by {dtrans}")
        self.update()

    def paintGL(self):
        self.set_model_projection()
        self.set_model_view()
        bgcolor = self.color
        glClearColor(*bgcolor)
        glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT )
        for item in self.items:
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glPushAttrib(GL_ALL_ATTRIB_BITS)
            item.paint()
            glPopAttrib()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()
    
    def update_movement(self):
        if self.active_keys == {}:
            return
        rotation_speed = 0.01
        trans_speed = 1
        z = self.get_z()
        if z < 20:
            trans_speed = z * 0.05
        if QtCore.Qt.Key_Up in self.active_keys:
            dR = euler_to_matrix([rotation_speed, 0, 0])
            self.Tbc[:3, :3] = self.Tbc[:3, :3] @ dR
        if QtCore.Qt.Key_Down in self.active_keys:
            dR = euler_to_matrix([-rotation_speed, 0, 0])
            self.Tbc[:3, :3] = self.Tbc[:3, :3] @ dR
        if QtCore.Qt.Key_Left in self.active_keys:
            dR = euler_to_matrix([0, 0, rotation_speed])
            self.Twb[:3, :3] = self.Twb [:3, :3]@ dR
        if QtCore.Qt.Key_Right in self.active_keys:
            dR = euler_to_matrix([0, 0, -rotation_speed])
            self.Twb[:3, :3] = self.Twb[:3, :3] @ dR
        if QtCore.Qt.Key_Z in self.active_keys:
            self.Twb[:3, 3] += self.Twb[:3, :3] @ self.Tbc[:3, :3] @ np.array([0, 0, +trans_speed])
        if QtCore.Qt.Key_X in self.active_keys:
            self.Twb[:3, 3] += self.Twb[:3, :3] @ self.Tbc[:3, :3] @ np.array([0, 0, -trans_speed])
        if QtCore.Qt.Key_A in self.active_keys:
            self.Twb[:3, 3] += self.Twb[:3, :3] @ np.array([-trans_speed, 0, 0])
        if QtCore.Qt.Key_D in self.active_keys:
            self.Twb[:3, 3] += self.Twb[:3, :3] @ np.array([trans_speed, 0, 0])
        if QtCore.Qt.Key_W in self.active_keys:
            self.Twb[:3, 3] += self.Twb[:3, :3] @ np.array([0, trans_speed, 0])
        if QtCore.Qt.Key_S in self.active_keys:
            self.Twb[:3, 3] += self.Twb[:3, :3] @ np.array([0, -trans_speed, 0])

    def get_z(self):
        return np.abs(self.Twb[2, 3])

    def set_model_view(self):
        m = self.get_view_matrix()
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(m.T)
        
    def get_view_matrix(self):
        return np.linalg.inv(self.Twb @ self.Tbc)

    def set_cam_position(self, **kwargs):
        pos = kwargs.get('pos', None)
        distance = kwargs.get('distance', None)
        if pos is not None:
            self.Twb[:3, 3] = pos
        if distance is not None:
            self.Twb[2, 3] = distance

    def set_color(self, color):
        self.color = color
        self.update()

    def update(self):
        self.update_movement()
        super().update()

    def set_model_projection(self):
        m = self.get_projection_matrix()
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixf(m.T)

    def get_projection_matrix(self):
        w, h = self.current_width(), self.current_height()
        dist = self.get_z()
        near = dist * 0.001
        far = dist * 10000.
        r = near * tan(0.5 * radians(self._fov))
        t = r * h / w
        matrix = frustum(-r, r, -t, t, near, far)
        return matrix

    def get_focal(self):
        width = self.current_width()
        height = self.current_height()
        fx = 0.5 * width / tan(radians(self._fov) / 2)
        fy = 0.5 * height / tan(radians(self._fov) / 2)
        return np.array([fx, fy])
            
    def rotate(self, rx=0, ry=0, rz=0):
        # update the euler angles
        self.euler += np.radians(np.array([rx, ry, rz]).astype(np.float32))
        self.euler = (self.euler + np.pi) % (2 * np.pi) - np.pi
        self.update()
