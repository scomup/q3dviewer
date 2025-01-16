"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.custom_items.cloud_item import CloudItem
from pathlib import Path
import os
from PySide6.QtWidgets import QPushButton, QLabel, QLineEdit, QMessageBox
from q3dviewer.utils.cloud_io import save_pcd, save_ply, save_e57, save_las, load_pcd, load_ply, load_e57, load_las

class CloudIOItem(CloudItem):
    """
    add save/load function to CloudItem
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.save_path = str(Path(os.path.expanduser("~"), "data.pcd"))

    def add_setting(self, layout):
        super().add_setting(layout)

        label4 = QLabel("Save Path:")
        layout.addWidget(label4)
        box4 = QLineEdit()
        box4.setText(self.save_path)
        box4.textChanged.connect(self.set_path)
        layout.addWidget(box4)
        save_button = QPushButton("Save Cloud")
        save_button.clicked.connect(self.save)
        layout.addWidget(save_button)
        self.save_msg = QMessageBox()
        self.save_msg.setIcon(QMessageBox.Information)
        self.save_msg.setWindowTitle("save")
        self.save_msg.setStandardButtons(QMessageBox.Ok)

    def save(self):
        cloud = self.buff[:self.valid_buff_top]
        func = None
        if self.save_path.endswith(".pcd"):
            func = save_pcd
        elif self.save_path.endswith(".ply"):
            func = save_ply
        elif self.save_path.endswith(".e57"):
            func = save_e57
        elif self.save_path.endswith(".las"):
            func = save_las
        elif self.save_path.endswith(".tif") or self.save_path.endswith(".tiff"):
            print("Do not support save as tif type!")
        else:
            print("Unsupported cloud file type!")
        try:
            func(cloud, self.save_path)
            self.save_msg.setText("Save cloud to  %s" % self.save_path)
        except Exception as e:
            print(e)
            self.save_msg.setText("Cannot save to %s" % self.save_path)
            self.save_msg.exec()
        self.save_msg.exec()

    def load(self, file, append=False):
        # print("Try to load %s ..." % file)
        if file.endswith(".pcd"):
            cloud, color_mode = load_pcd(file)
        elif file.endswith(".ply"):
            cloud, color_mode = load_ply(file)
        elif file.endswith(".e57"):
            cloud, color_mode = load_e57(file)
        elif file.endswith(".las"):
            cloud, color_mode = load_las(file)
        else:
            print("Not supported file type.")
            return
        self.set_data(data=cloud, append=append)
        self.set_color_mode(color_mode)
        return cloud

    def set_path(self, path):
        self.save_path = path
