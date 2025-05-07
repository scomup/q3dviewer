#!/usr/bin/env python3

"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

"""
You can use the following code to set the Q3D_QT_IMPL environment variable
# tested for PyQt5, PySide2 and PySide6
# -------------------------------
in your python script before importing QT:
    import os
    os.environ['Q3D_QT_IMPL'] = 'PySide6'
or in your bash:
    export Q3D_QT_IMPL=PyQt5
# -------------------------------
"""

import importlib.util
import builtins
import sys
import os


Q3D_QT_IMPL = os.environ.get('Q3D_QT_IMPL')
Q3D_DEBUG = os.environ.get('Q3D_DEBUG')

if Q3D_QT_IMPL not in ['PyQt5', 'PySide2', 'PySide6']:
    Q3D_QT_IMPL = None

if Q3D_QT_IMPL is None:
    if importlib.util.find_spec('PyQt5') is not None:
        Q3D_QT_IMPL = 'PyQt5'
    elif importlib.util.find_spec('PySide6') is not None:
        Q3D_QT_IMPL = 'PySide6'
    else:
        raise ImportError('No Qt binding found. Please install PySide6.')


def import_module(name):
    module = builtins.__import__(f'{Q3D_QT_IMPL}.{name}').__dict__[name]
    sys.modules[f'{__name__}.{name}'] = module


def load_qt():
    if Q3D_DEBUG is not None:
        print(f'Using {Q3D_QT_IMPL} as Qt binding.')

    # register common Qt modules
    modules = ['QtCore', 'QtGui', 'QtWidgets']
    for name in modules:
        import_module(name)
    # register OpenGL modules for PySide6
    if Q3D_QT_IMPL == 'PySide6':
        import_module('QtOpenGLWidgets')
        sys.modules[f'{__name__}.QtWidgets'].QOpenGLWidget = sys.modules[f'{__name__}.QtOpenGLWidgets'].QOpenGLWidget
    # make PyQt5 and PySide6 modules compatible
    if Q3D_QT_IMPL == 'PyQt5':
        sys.modules[f'{__name__}.QtCore'].Signal = sys.modules[f'{__name__}.QtCore'].pyqtSignal
        sys.modules[f'{__name__}.QtCore'].Slot = sys.modules[f'{__name__}.QtCore'].pyqtSlot
        sys.modules[f'{__name__}.QtCore'].Property = sys.modules[f'{__name__}.QtCore'].pyqtProperty
    # Handle exec() for PySide2
    if Q3D_QT_IMPL == 'PySide2':
        sys.modules[f'{__name__}.QtWidgets'].QApplication.exec = sys.modules[f'{__name__}.QtWidgets'].QApplication.exec_


load_qt()
