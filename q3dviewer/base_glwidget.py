from OpenGL.GL import *  # noqa
from math import radians, tan
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from q3dviewer.utils import frustum, euler_to_matrix, makeT, m_get_roll, makeRt


class BaseGLWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        QtWidgets.QOpenGLWidget.__init__(self, parent)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
        self.reset()
        self._fov = 60
        self.items = []
        self.keyTimer = QtCore.QTimer()
        self.color = np.array([0, 0, 0, 0])
        self.dist = 40
        self.euler = np.array([np.pi/3, 0, np.pi/4])
        self.center = np.array([0, 0, 0.])
        self.active_keys = set()
        self.show_center = False
        self.enable_show_center = True

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
        
    def remove_item(self, item):
        """
        Remove the item from the glwidget.
        """
        self.items.remove(item)
        item.set_glwidget(None)

    def clear(self):
        """
        Remove all items from the glwidget.
        """
        for item in self.items:
            item.set_glwidget(None)
        self.items = []
        
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

    def update_dist(self, delta):
        self.dist += delta
        if self.dist < 0.1:
            self.dist = 0.1

    def wheelEvent(self, ev):
        delta = ev.angleDelta().x()
        if delta == 0:
            delta = ev.angleDelta().y()
        self.update_dist(-delta * self.dist * 0.001)
        self.show_center = True

    def mouseMoveEvent(self, ev):
        lpos = ev.localPos()
        if not hasattr(self, 'mousePos'):
            self.mousePos = lpos
        diff = lpos - self.mousePos
        self.mousePos = lpos
        if ev.buttons() == QtCore.Qt.MouseButton.RightButton:
            rot_speed = 0.2
            dyaw = radians(-diff.x() * rot_speed)
            droll = radians(-diff.y() * rot_speed)
            self.rotate(droll, 0, dyaw)
        elif ev.buttons() == QtCore.Qt.MouseButton.LeftButton:
            Rwc = euler_to_matrix(self.euler)
            Kinv = np.linalg.inv(self.get_K())
            dist = max(self.dist, 0.5)
            self.center += Rwc @ Kinv @ np.array([-diff.x(), diff.y(), 0]) * dist
        self.show_center = True

    def paintGL(self):
        self.set_model_projection()
        self.set_model_view()
        bgcolor = self.color
        glClearColor(*bgcolor)
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
        for item in self.items:
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glPushAttrib(GL_ALL_ATTRIB_BITS)
            item.paint()
            glPopAttrib()
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()
        
        # Show center as a point if updated by mouse move event
        if self.enable_show_center and self.show_center:
            point_size = np.clip((self.get_K()[0, 0] / self.dist), 10, 100)
            glPointSize(point_size)
            glBegin(GL_POINTS)
            glColor3f(1.0, 0.0, 0.0)  # Red color for the center point
            glVertex3f(*self.center)
            glEnd()
            self.show_center = False
    
    def update_movement(self):
        if self.active_keys == {}:
            return
        Rwc = euler_to_matrix([0, 0, self.euler[2]])
        rot_speed = 0.5
        trans_speed = max(self.dist * 0.005, 0.1)
        if QtCore.Qt.Key_Up in self.active_keys:
            self.rotate(radians(rot_speed), 0, 0)
        if QtCore.Qt.Key_Down in self.active_keys:
            self.rotate(radians(-rot_speed), 0, 0)
        if QtCore.Qt.Key_Left in self.active_keys:
            self.rotate(0, 0, radians(rot_speed))
        if QtCore.Qt.Key_Right in self.active_keys:
            self.rotate(0, 0, radians(-rot_speed))
        if QtCore.Qt.Key_Z in self.active_keys:
            self.update_dist(trans_speed)
        if QtCore.Qt.Key_X in self.active_keys:
            self.update_dist(-trans_speed)
        if QtCore.Qt.Key_W in self.active_keys:
            self.center += Rwc @ np.array([0, trans_speed, 0])
        if QtCore.Qt.Key_S in self.active_keys:
            self.center += Rwc @ np.array([0, -trans_speed, 0])
        if QtCore.Qt.Key_A in self.active_keys:
            self.center += Rwc @  np.array([-trans_speed, 0, 0])
        if QtCore.Qt.Key_D in self.active_keys:
            self.center += Rwc @  np.array([trans_speed, 0, 0])

    def set_model_view(self):
        m = self.get_view_matrix()
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(m.T)
        
    def get_view_matrix(self):
        tcw = np.array([0, 0, self.dist])
        Rwc = euler_to_matrix(self.euler)
        twc = self.center + Rwc @ tcw
        Twc = makeT(Rwc, twc)
        return np.linalg.inv(Twc)

    def set_cam_position(self, **kwargs):
        pos = kwargs.get('pos', None)
        distance = kwargs.get('distance', None)
        if pos is not None:
            self.center = pos
        if distance is not None:
            self.dist = distance

    def set_color(self, color):
        self.color = color

    def update(self):
        self.update_movement()
        super().update()

    def set_model_projection(self):
        m = self.get_projection_matrix()
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixf(m.T)

    def get_projection_matrix(self):
        w, h = self.current_width(), self.current_height()
        dist = self.dist
        near = dist * 0.001
        far = dist * 10000.
        r = near * tan(0.5 * radians(self._fov))
        t = r * h / w
        matrix = frustum(-r, r, -t, t, near, far)
        return matrix

    def get_K(self):
        project_matrix = self.get_projection_matrix()
        width = self.current_width()
        height = self.current_height()
        fx = project_matrix[0, 0] * width / 2
        fy = project_matrix[1, 1] * height / 2
        cx = width / 2
        cy = height / 2
        K = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ])
        return K
            
    def rotate(self, rx=0, ry=0, rz=0):
        # update the euler angles
        self.euler += np.array([rx, ry, rz])
        self.euler[2] = (self.euler[2] + np.pi) % (2 * np.pi) - np.pi
        self.euler[1] = (self.euler[1] + np.pi) % (2 * np.pi) - np.pi
        self.euler[0] = np.clip(self.euler[0], 0, np.pi)

    def change_show_center(self, state):
        self.enable_show_center = state