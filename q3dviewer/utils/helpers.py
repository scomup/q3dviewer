"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from OpenGL.GL import *


def rainbow(scalars, scalar_min=0, scalar_max=255):
    range = scalar_max - scalar_min
    values = 1.0 - (scalars - scalar_min) / range
    # values = (scalars - scalar_min) / range  # using inverted color
    colors = np.zeros([scalars.shape[0], 3], dtype=np.float32)
    values = np.clip(values, 0, 1)

    h = values * 5.0 + 1.0
    i = np.floor(h).astype(int)
    f = h - i
    f[np.logical_not(i % 2)] = 1 - f[np.logical_not(i % 2)]
    n = 1 - f

    # idx = i <= 1
    colors[i <= 1, 0] = n[i <= 1] * 255
    colors[i <= 1, 1] = 0
    colors[i <= 1, 2] = 255

    colors[i == 2, 0] = 0
    colors[i == 2, 1] = n[i == 2] * 255
    colors[i == 2, 2] = 255

    colors[i == 3, 0] = 0
    colors[i == 3, 1] = 255
    colors[i == 3, 2] = n[i == 3] * 255

    colors[i == 4, 0] = n[i == 4] * 255
    colors[i == 4, 1] = 255
    colors[i == 4, 2] = 0

    colors[i >= 5, 0] = 255
    colors[i >= 5, 1] = n[i >= 5] * 255
    colors[i >= 5, 2] = 0
    return colors


def text_to_rgba(color_text, flat=False):
    """
    Convert a color text to an RGBA tuple.
    
    :param color_text: e.g. '#FF0000', '#FF0000FF', 'red', 'green', 'blue', 'yellow', 
                       'black', 'white', 'magenta', 'cyan', 'r', 'g', 'b', 'y', 'k', 'w', 'm', 'c'
    :return: RGBA tuple, e.g. (1.0, 0.0, 0.0, 1.0)
    """
    from matplotlib.colors import to_rgba

    rgba = to_rgba(color_text)
    if flat:
        r, g, b, _ = (np.array(rgba)*255).astype(np.uint32)
        falt_rgb = ((r << 16) & 0xFF0000) | \
                   ((g << 8) & 0x00FF00) | \
                   ((b << 0) & 0x0000FF)
        return falt_rgb
    else:
        return rgba
    

def set_uniform(shader, content, name):
    location = glGetUniformLocation(shader, name)
    if location == -1:
        raise ValueError(
            f"Uniform '{name}' not found in shader program {shader}.")

    if isinstance(content, int):
        glUniform1i(location, content)
    elif isinstance(content, float):
        glUniform1f(location, content)
    elif isinstance(content, np.ndarray):
        if content.ndim == 1:
            if content.shape[0] == 2:
                glUniform2f(location, *content)
            elif content.shape[0] == 3:
                glUniform3f(location, *content)
            else:
                raise ValueError(
                    f"Unsupported 1D array size: {content.shape}.")
        elif content.ndim == 2:
            if content.shape == (4, 4):
                glUniformMatrix4fv(location, 1, GL_FALSE,
                                   content.T.astype(np.float32))
            else:
                raise ValueError(
                    f"Unsupported 2D array size: {content.shape}.")
        else:
            raise ValueError(f"Unsupported array dimension: {content.ndim}.")
    else:
        raise TypeError(
            f"Unsupported type for uniform '{name}': {type(content)}.")
