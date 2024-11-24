#!/usr/bin/env python3

"""
Copyright 2024  Liu Yang
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import q3dviewer as q3d
from pyqtgraph.opengl import GLMeshItem
from stl import mesh


class MeshViewer(q3d.Viewer):
    def __init__(self):
        super(MeshViewer, self).__init__(name="Mesh Viewer")
        self.setAcceptDrops(True)

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        for url in event.mimeData().urls():
            file_path = url.toLocalFile()
            self.open_mesh_file(file_path)

    def open_mesh_file(self, file):
        mesh_item = self['mesh']
        if mesh_item is None:
            print("Can't find meshitem")
            return

        print("Try to load %s ..." % file)
        stl_mesh = mesh.Mesh.from_file(file)

        vertices = stl_mesh.points.reshape(-1, 3)
        faces = np.arange(vertices.shape[0]).reshape(-1, 3)
        mesh_item.setMeshData(vertexes=vertices, faces=faces)


def main():
    app = q3d.QApplication([])
    viewer = MeshViewer()

    grid_item = q3d.GridItem(size=1000, spacing=20)
    # 'glOptions', 'opaque', 'additive' 'translucent'
    mesh_item = GLMeshItem(smooth=True, drawFaces=True, drawEdges=True,
                           color=(0, 1, 0, 0.2),
                           edgeColor=(1, 1, 1, 1),
                           glOptions='translucent')

    viewer.add_items({'grid': grid_item, 'mesh': mesh_item})

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()
