"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from OpenGL.GL import *
from math import radians, tan
import numpy as np
from PySide6 import QtCore, QtGui, QtOpenGLWidgets
from q3dviewer.utils.maths import frustum, euler_to_matrix, makeT


class BaseGLWidget(QtOpenGLWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        QtOpenGLWidgets.QOpenGLWidget.__init__(self, parent)
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
        pass
        self.update_model_projection()
        self.update_model_view()
        bgcolor = self.color
        glClearColor(*bgcolor)
        glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)
        for item in self.items:
            if not item.visible():
                continue
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glPushAttrib(GL_ALL_ATTRIB_BITS)
            try:
                item.paint()
            finally:
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
        """
        Update the movement of the camera based on the active keys.
        """
        if not self.active_keys:
            return
        rot_speed = 0.5
        trans_speed = max(self.dist * 0.005, 0.1)
        # Handle rotation keys
        if QtCore.Qt.Key_Up in self.active_keys:
            self.rotate(radians(rot_speed), 0, 0)
        if QtCore.Qt.Key_Down in self.active_keys:
            self.rotate(radians(-rot_speed), 0, 0)
        if QtCore.Qt.Key_Left in self.active_keys:
            self.rotate(0, 0, radians(rot_speed))
        if QtCore.Qt.Key_Right in self.active_keys:
            self.rotate(0, 0, radians(-rot_speed))
        # Handle zoom keys
        xz_keys = {QtCore.Qt.Key_Z, QtCore.Qt.Key_X}
        if self.active_keys & xz_keys:
            Rwc = euler_to_matrix(self.euler)
            if QtCore.Qt.Key_Z in self.active_keys:
                self.center += Rwc @ np.array([0, 0, -trans_speed])
            if QtCore.Qt.Key_X in self.active_keys:
                self.center += Rwc @ np.array([0, 0, trans_speed])
        # Handle translation keys on the z plane
        dir_keys = {QtCore.Qt.Key_W, QtCore.Qt.Key_S, QtCore.Qt.Key_A, QtCore.Qt.Key_D}
        if self.active_keys & dir_keys:
            Rz = euler_to_matrix([0, 0, self.euler[2]])
            if QtCore.Qt.Key_W in self.active_keys:
                self.center += Rz @ np.array([0, trans_speed, 0])
            if QtCore.Qt.Key_S in self.active_keys:
                self.center += Rz @ np.array([0, -trans_speed, 0])
            if QtCore.Qt.Key_A in self.active_keys:
                self.center += Rz @ np.array([-trans_speed, 0, 0])
            if QtCore.Qt.Key_D in self.active_keys:
                self.center += Rz @ np.array([trans_speed, 0, 0])

    def update_model_view(self):
        m = self.get_view_matrix()
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(m.T)
        
    def get_view_matrix(self):
        two = self.center # the origin(center) in the world frame
        tco = np.array([0, 0, self.dist]) # the origin(center) in camera frame
        Rwc = euler_to_matrix(self.euler)
        twc = two + Rwc @ tco
        Rcw = Rwc.T
        tcw = -Rcw @ twc
        Tcw = makeT(Rcw, tcw)
        return Tcw

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

    def update_model_projection(self):
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
