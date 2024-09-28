#!/usr/bin/env python3

import numpy as np
from q3dviewer.custom_items import *
from q3dviewer.basic_window import *
from q3dviewer.gau_io import load_gs


class GuassianViewer(Viewer):
    def __init__(self):
        super(GuassianViewer, self).__init__(name="Guassian Viewer")
        self.setAcceptDrops(True)

    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.accept()
        else:
            event.ignore()

    def dropEvent(self, event):
        for url in event.mimeData().urls():
            file_path = url.toLocalFile()
            self.openGuassianFile(file_path)

    def openGuassianFile(self, file):
        gau_item = self['gaussian']
        if gau_item is None:
            print("Can't find gaussianitem")
            return

        print("Try to load %s ..." % file)
        gs = load_gs(file)
        gs_data = gs.view(np.float32).reshape(gs.shape[0], -1)
        gau_item.setData(gs_data=gs_data)


def main():
    app = QApplication([])
    viewer = GuassianViewer()

    gird_item = GridItem(size=1000, spacing=20)
    gau_item = GaussianItem()

    viewer.addItems({'grid': gird_item, 'gaussian': gau_item})

    viewer.show()
    app.exec_()


if __name__ == '__main__':
    main()