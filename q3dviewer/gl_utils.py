from OpenGL.GL import *
import numpy as np

def set_uniform_mat4fv(shader, content, name):
    content = content.T
    glUniformMatrix4fv(
        glGetUniformLocation(shader, name),
        1,
        GL_FALSE,
        content.astype(np.float32)
    )


def set_uniform_1i(shader, content, name):
    glUniform1i(
        glGetUniformLocation(shader, name),
        content
    )


def set_uniform_2f(shader, contents, name):
    glUniform2f(
        glGetUniformLocation(shader, name),
        *contents
    )


def set_uniform_v3(shader, contents, name):
    glUniform3f(
        glGetUniformLocation(shader, name),
        *contents
    )
