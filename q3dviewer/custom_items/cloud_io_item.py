"""
Copyright 2024  Liu Yang
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.custom_items.cloud_item import CloudItem
from pathlib import Path
import os
from PyQt5.QtWidgets import QPushButton, QLabel, QLineEdit, QMessageBox
import numpy as np
# from pye57 import E57
from pypcd4 import PointCloud, MetaData
from plyfile import PlyData, PlyElement


def save_as_ply(cloud, save_path):
    dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'u4')]
    ply_element = PlyElement.describe(cloud.view(dtype), 'vertex')
    PlyData([ply_element], byte_order='>').write(save_path)


def save_as_pcd(cloud, save_path):
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
    PointCloud(metadata, cloud).save(self.save_path)


def load_pcd(path):
    pc = PointCloud.from_path(file).pc_data
    if "rgb" in 
    try:
        color = pc["rgb"].astype(np.uint32)
        try:
            color = pc["intensity"].astype(np.uint32)
            self.setColorMode('I')
        except ValueError:
            color = pc['z'].astype(np.uint32)
            self.setFlatRGB('0xffffff')

        cloud = np.rec.fromarrays(
            [np.stack([pc["x"], pc["y"], pc["z"]], axis=1), color],
            dtype=self.data_type)
        self.setData(data=cloud)


class CloudIOItem(CloudItem):
    """
    add save/load function to CloudItem
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.save_path = str(Path(os.path.expanduser("~"), "data.pcd"))

    def addSetting(self, layout):
        super().addSetting(layout)

        label4 = QLabel("Save Path:")
        layout.addWidget(label4)
        box4 = QLineEdit()
        box4.setText(self.save_path)
        box4.textChanged.connect(self.setPath)
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
        if self.save_path.endswith(".pcd"):
            # Save as PCD
            try:
                save_as_pcd(cloud, self.save_path)
                self.save_msg.setText("Save cloud to  %s" % self.save_path)
            except Exception as e:
                self.save_msg.setText("Cannot save to %s" % self.save_path)
            self.save_msg.exec()

        elif self.save_path.endswith(".ply"):
            try:
                save_as_ply(cloud, self.save_path)
                self.save_msg.setText("Save cloud to  %s" % self.save_path)
            except Exception as e:
                self.save_msg.setText("Cannot save to %s" % self.save_path)
            self.save_msg.exec()
            # elif self.save_path.endswith(".e57"):
            # # Save as E57
            # try:
            #     with E57(self.save_path, mode="w") as e57_file:
            #         e57_file.write_pointcloud(
            #             positions=cloud['xyz'],
            #             colors=(cloud['color']))
            #     self.save_msg.setText("Save cloud to  %s" % self.save_path)
            # except Exception as e:
            #     self.save_msg.setText("Cannot save to %s" % self.save_path)
            # self.save_msg.exec()
        else:
            self.showMessage("Unsupported cloud file type!", error=True)

    def load(self, file):
        print("Try to load %s ..." % file)
        pc = PointCloud.from_path(file).pc_data

        try:
            color = pc["rgb"].astype(np.uint32)
            self.setColorMode('RGB')
        except ValueError:
            try:
                color = pc["intensity"].astype(np.uint32)
                self.setColorMode('I')
            except ValueError:
                color = pc['z'].astype(np.uint32)
                self.setFlatRGB('0xffffff')

        cloud = np.rec.fromarrays(
            [np.stack([pc["x"], pc["y"], pc["z"]], axis=1), color],
            dtype=self.data_type)
        self.setData(data=cloud)

    def setPath(self, path):
        self.save_path = path
