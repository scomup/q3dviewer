from OpenGL.GL import *  # noqa
import OpenGL.GL.framebufferobjects as glfbo  # noqa
from math import cos, radians, sin, tan

import numpy as np

from PyQt5 import QtCore, QtGui, QtWidgets

from q3dviewer.utils import frustum, euler_to_matrix, make_transform, makeT, m_get_roll  # Import euler_to_matrix function
from PyQt5.QtGui import QKeyEvent, QVector3D, QMatrix4x4

class BaseGLWidget(QtWidgets.QOpenGLWidget):
    
    def __init__(self, parent=None):
        QtWidgets.QOpenGLWidget.__init__(self, parent)
        self.setFocusPolicy(QtCore.Qt.FocusPolicy.ClickFocus)
        self._fov = 60
        self.reset()
        self.items = []
        self.noRepeatKeys = [QtCore.Qt.Key.Key_Right, QtCore.Qt.Key.Key_Left, QtCore.Qt.Key.Key_Up, QtCore.Qt.Key.Key_Down, QtCore.Qt.Key.Key_PageUp, QtCore.Qt.Key.Key_PageDown]
        self.keysPressed = {}
        self.keyTimer = QtCore.QTimer()
        self.color = np.array([0, 0, 0, 0])
        self.Twb = makeT(euler_to_matrix([0, 0, 0]), np.array([-0, -50, 20]))
        self.Tbc = makeT(euler_to_matrix([np.pi/3, 0, 0]), np.array([0, 0, 0]))
        self.active_keys = set()

    def keyPressEvent(self, ev: QKeyEvent):
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

    def keyReleaseEvent(self, ev: QKeyEvent):
        self.active_keys.discard(ev.key())

    def get_width(self):
        dpr = self.devicePixelRatioF()
        return int(self.width() * dpr)

    def get_height(self):
        dpr = self.devicePixelRatioF()
        return int(self.height() * dpr)

    def reset(self):
        """
        Initialize the widget state or reset the current state to the original state.
        """
        pass
        # self.opts['center'] = Vector(0,0,0)  ## will always appear at the center of the widget
        # self._dist = 10.0         ## distance of camera from center
        # self._fov = 60                ## horizontal field of view in degrees
        # self.opts['elevation'] = 30          ## camera's angle of elevation in degrees
        # self.opts['azimuth'] = 45            ## camera's azimuthal angle in degrees 
        #                                      ## (rotation around z-axis 0 points along x-axis)
        # self.opts['viewport'] = None         ## glViewport params; None == whole widget
        # self.set_color(np.array([0, 0, 0, 0]))

    def add_item(self, item):
        self.items.append(item)

        if self.isValid():
            item.initialize()
                
        item._set_view(self)
        self.update()
        
    def remove_item(self, item):
        """
        Remove the item from the scene.
        """
        self.items.remove(item)
        item._set_view(None)
        self.update()

    def clear(self):
        """
        Remove all items from the scene.
        """
        for item in self.items:
            item._set_view(None)
        self.items = []
        self.update()        
        
    def initializeGL(self):
        for item in self.items:
            if not item.isInitialized():
                item.initialize()
        
    def set_color(self, color):
        self.color = color
        self.update()

    def update(self):
        self.update_movement()
        super().update()

    def set_model_projection(self, region=None):
        m = self.get_projection_matrix(region)
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixf(m.T)

    def get_projection_matrix(self, region=None):
        w, h = self.get_width(), self.get_height()
        dist = self.get_z()
        near = dist * 0.001
        far = dist * 10000.
        r = near * tan(0.5 * radians(self._fov))
        t = r * h / w
        matrix = frustum(-r, r, -t, t, near, far)
        return matrix

    def get_focal(self):
        width = self.get_width()
        height = self.get_height()
        fx = 0.5 * width / tan(radians(self._fov) / 2)
        fy = 0.5 * height / tan(radians(self._fov) / 2)
        return np.array([fx, fy])

    def mouseReleaseEvent(self, ev):
        if hasattr(self, 'mousePos'):
            delattr(self, 'mousePos')

    def screen_to_world(self, x, y):
        width = self.get_width()
        height = self.get_height()
        projection_matrix = self.get_projection_matrix()
        view_matrix = self.get_view_matrix()
        inv_proj_view = np.linalg.inv(projection_matrix @ view_matrix)
        
        # Normalize screen coordinates to [-1, 1]
        norm_x = (2.0 * x) / width - 1.0
        norm_y = 1.0 - (2.0 * y) / height
        norm_z = -1.0  # Near plane

        screen_point_near = np.array([norm_x, norm_y, norm_z, 1.0])
        world_point_near = inv_proj_view @ screen_point_near
        world_point_near /= world_point_near[3]  # Normalize by w

        norm_z = 1.0  # Far plane
        screen_point_far = np.array([norm_x, norm_y, norm_z, 1.0])
        world_point_far = inv_proj_view @ screen_point_far
        world_point_far /= world_point_far[3]  # Normalize by w

        # Vector from camera to mouse event point
        direction = world_point_far[:3] - world_point_near[:3]

        # Find intersection with z=0 plane
        t = -world_point_near[2] / direction[2]
        intersection_point = world_point_near[:3] + t * direction

        return intersection_point

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

    def wheelEvent(self, ev):
        delta = ev.angleDelta().x()
        if delta == 0:
            delta = ev.angleDelta().y()
        delta = self.get_z() * delta * 0.003
        self.Twb[:3, 3] += self.Twb[:3, :3] @ self.Tbc[:3, :3] @ np.array([0, 0, -delta])
        self.update()

    def set_model_view(self):
        m = self.get_view_matrix()
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(m.T)
        
    def get_view_matrix(self):
        # Create a 4x4 identity matrix
        return np.linalg.inv(self.Twb @ self.Tbc)

    def set_cam_position(self, **kwargs):
        pos = kwargs.get('pos', None)
        distance = kwargs.get('distance', None)
        if pos is not None:
            self.Twb[:3, 3] = pos
        if distance is not None:
            self.Twb[2, 3] = distance


    def paintGL(self, region=None, viewport=None, useItemNames=False):
        """
        viewport specifies the arguments to glViewport. If None, then we use self.opts['viewport']
        region specifies the sub-region of self.opts['viewport'] that should be rendered.
        Note that we may use viewport != self.opts['viewport'] when exporting.
        """
        self.set_model_projection(region=region)
        self.set_model_view()
        bgcolor = self.color
        glClearColor(*bgcolor)
        glClear( GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT )
        self.draw_items(useItemNames=useItemNames)
        
    def draw_items(self, item=None, useItemNames=False):
        if item is None:
            items = [x for x in self.items if x.parent_item() is None]
        else:
            items = item.child_items()
            items.append(item)
        for i in items:
            if not i.visible():
                continue
            if i is item:
                try:
                    glPushAttrib(GL_ALL_ATTRIB_BITS)
                    if useItemNames:
                        glLoadName(i._id)
                        self._itemNames[i._id] = i
                    i.paint()
                except:
                    from .. import debug
                    debug.printExc()
                    print("Error while drawing item %s." % str(item))
                    
                finally:
                    glPopAttrib()
            else:
                glMatrixMode(GL_MODELVIEW)
                glPushMatrix()
                try:
                    self.draw_items(i, useItemNames=useItemNames)
                finally:
                    glMatrixMode(GL_MODELVIEW)
                    glPopMatrix()
            

    def rotate(self, rx=0, ry=0, rz=0):
        # update the euler angles
        self.euler += np.radians(np.array([rx, ry, rz]).astype(np.float32))
        self.euler = (self.euler + np.pi) % (2 * np.pi) - np.pi
        self.update()
