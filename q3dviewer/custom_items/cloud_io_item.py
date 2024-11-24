"""
Copyright 2024  Liu Yang
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.custom_items.cloud_item import CloudItem
from pypcd4 import PointCloud, MetaData
from pathlib import Path
import os
from PyQt5.QtWidgets import QPushButton, QLabel, QLineEdit, QMessageBox
import numpy as np


class CloudIOItem(CloudItem):
    """
    add save/load function to CloudItem
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.save_path = str(Path(os.getcwd(), "data.pcd"))

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

    def save(self):
        cloud = self.buff[:self.valid_buff_top]
        if self.save_path.endswith(".pcd"):
            fields = ("x", "y", "z", "rgb")
            metadata = MetaData.model_validate(
                {
                    "fields": fields,
                    "size": [4, 4, 4, 4],
                    "type": ['F', 'F', 'F', 'U'],
                    "count": [1, 1, 1, 1],
                    "width": cloud.shape[0],
                    "points": cloud.shape[0],
                })
            pc = PointCloud(metadata, cloud)
            try:
                pc.save(self.save_path)
                save_msg = QMessageBox()
                save_msg.setIcon(QMessageBox.Information)
                save_msg.setWindowTitle("save")
                save_msg.setStandardButtons(QMessageBox.Ok)
                save_msg.setText("save cloud to  %s" % self.save_path)
            except:
                save_msg.setText("Cannot save to %s" % self.save_path)
            save_msg.exec()

        elif self.save_path.endswith(".ply"):
            print("Not implment yet!")
        else:
            print("Not supported cloud file type!")

    def load(self, file):
            print("Try to load %s ..." % file)
            pc = PointCloud.from_path(file).pc_data

            try:
                color = pc["rgb"].astype(np.uint32)
                self.set_color_mode('RGB')
            except ValueError:
                try:
                    color = pc["intensity"].astype(np.uint32)
                    self.set_color_mode('I')
                except ValueError:
                    color = pc['z'].astype(np.uint32)
                    self.set_color_mode('#FFFFFF')

            cloud = np.rec.fromarrays(
                [np.stack([pc["x"], pc["y"], pc["z"]], axis=1), color],
                dtype=self.data_type)
            self.set_data(data=cloud)

    def set_path(self, path):
        self.save_path = path
