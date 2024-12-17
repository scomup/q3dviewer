"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.custom_items.cloud_item import CloudItem
from pathlib import Path
import os
from PyQt5.QtWidgets import QPushButton, QLabel, QLineEdit, QMessageBox
import numpy as np
from pypcd4 import PointCloud, MetaData
from plyfile import PlyData, PlyElement
from pye57 import E57
import laspy


def save_as_ply(cloud, save_path):
    if (np.max((cloud['color']) >> 16 & 0xff)):
        dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('rgb', 'u4')]
    else:
        dtype = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'u4')]
    ply_element = PlyElement.describe(cloud.view(dtype), 'vertex')
    PlyData([ply_element], byte_order='>').write(save_path)


def load_ply(file):
    ply_data = PlyData.read(file)
    vertices = ply_data['vertex']
    names = [x.name for x in vertices.properties]
    x = np.array(vertices['x'], dtype=np.float32)
    y = np.array(vertices['y'], dtype=np.float32)
    z = np.array(vertices['z'], dtype=np.float32)
    if 'rgb' in names:
        color = np.array(vertices['rgb'], dtype=np.uint32)
        color_mode = 'RGB'
    elif 'intensity' in names:
        color = np.array(vertices['intensity'], dtype=np.uint32)
        color_mode = 'I'
    else:
        color = z.astype(np.uint32)
        color_mode = 'FLAT'
    dtype = [('xyz', '<f4', (3,)), ('color', '<u4')]
    cloud = np.rec.fromarrays(
        [np.stack([x, y, z], axis=1), color],
        dtype=dtype)
    return cloud, color_mode


def save_as_pcd(cloud, save_path):
    if (np.max((cloud['color']) >> 16 & 0xff)):
        fields = ('x', 'y', 'z', 'rgb')
    else:
        fields = ('x', 'y', 'z', 'intensity')
    metadata = MetaData.model_validate(
        {
            "fields": fields,
            "size": [4, 4, 4, 4],
            "type": ['F', 'F', 'F', 'U'],
            "count": [1, 1, 1, 1],
            "width": cloud.shape[0],
            "points": cloud.shape[0],
        })
    PointCloud(metadata, cloud).save(save_path)


def load_pcd(file):
    dtype = [('xyz', '<f4', (3,)), ('color', '<u4')]
    pc = PointCloud.from_path(file).pc_data
    if 'rgb' in pc.dtype.names:
        color = pc['rgb'].astype(np.uint32)
        color_mode = 'RGB'
    elif 'intensity' in pc.dtype.names:
        color = pc['intensity'].astype(np.uint32)
        color_mode = 'I'
    else:
        color = pc['z'].astype(np.uint32)
        color_mode = 'FLAT'
    cloud = np.rec.fromarrays(
        [np.stack([pc['x'], pc['y'], pc['z']], axis=1), color],
        dtype=dtype)
    return cloud, color_mode


def save_as_e57(cloud, save_path):
    e57 = E57(save_path, mode='w')
    x = cloud['xyz'][:, 0]
    y = cloud['xyz'][:, 1]
    z = cloud['xyz'][:, 2]
    i = (cloud['color'] & 0xFF000000) >> 24
    r = (cloud['color'] & 0x00FF0000) >> 16
    g = (cloud['color'] & 0x0000FF00) >> 8
    b = (cloud['color'] & 0x000000ff)
    data = {"cartesianX": x, "cartesianY": y, "cartesianZ": z,
            "intensity": i,
            "colorRed": r, "colorGreen": g, "colorBlue": b}
    e57.write_scan_raw(data)
    e57.close()


def load_e57(file_path):
    e57 = E57(file_path, mode="r")
    scans = e57.read_scan(0, ignore_missing_fields=True,
                          intensity=True, colors=True)
    x = scans["cartesianX"]
    y = scans["cartesianY"]
    z = scans["cartesianZ"]
    color = np.zeros([x.shape[0]], dtype=np.uint32)
    if "intensity" in scans:
        color = scans["intensity"].astype(np.uint32)
        color_mode = 'I'
    if all([x in scans for x in ["colorRed", "colorGreen", "colorBlue"]]):
        r = scans["colorRed"].astype(np.uint32)
        g = scans["colorGreen"].astype(np.uint32)
        b = scans["colorBlue"].astype(np.uint32)
        color = (color.astype(np.uint32) << 24) | \
                (r.astype(np.uint32) << 16) | \
                (g.astype(np.uint32) << 8) | \
                (b.astype(np.uint32) << 0)
        color_mode = 'RGB'
    cloud = np.empty(
        len(x), dtype=[('xyz', '<f4', (3,)), ('color', '<u4')])
    cloud['xyz'] = np.stack((x, y, z), axis=-1)
    cloud['color'] = color
    e57.close()
    return cloud, color_mode


def load_las(file):
    with laspy.open(file) as f:
        las = f.read()
        points = np.vstack((las.x, las.y, las.z)).transpose()
        dimensions = las.point_format.dimension_names
        if 'red' in dimensions and 'green' in dimensions and 'blue' in dimensions:
            red = las.red
            green = las.green
            blue = las.blue
            if red.dtype == np.dtype('uint16'):
                red = (red / 65535.0 * 255).astype(np.uint32)
                green = (green / 65535.0 * 255).astype(np.uint32)
                blue = (blue / 65535.0 * 255).astype(np.uint32)
            color = (red << 16) | (green << 8) | blue
            color_mode = 'RGB'
        elif 'intensity' in dimensions:
            color = las.intensity
            color_mode = 'I'
        else:
            color = las.z.astype(np.uint32)
            color_mode = 'FLAT'
        dtype = [('xyz', '<f4', (3,)), ('color', '<u4')]
        cloud = np.rec.fromarrays([points, color], dtype=dtype)
    return cloud, color_mode

def save_as_las(cloud, save_path):
    header = laspy.LasHeader(point_format=3, version="1.2")
    las = laspy.LasData(header)
    las.x = cloud['xyz'][:, 0]
    las.y = cloud['xyz'][:, 1]
    las.z = cloud['xyz'][:, 2]
    if 'rgb' in cloud.dtype.names:
        las.red = (cloud['color'] >> 16) & 0xFF
        las.green = (cloud['color'] >> 8) & 0xFF
        las.blue = cloud['color'] & 0xFF
    elif 'intensity' in cloud.dtype.names:
        las.intensity = cloud['color']
    las.write(save_path)


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
        func = None
        if self.save_path.endswith(".pcd"):
            func = save_as_pcd
        elif self.save_path.endswith(".ply"):
            func = save_as_ply
        elif self.save_path.endswith(".e57"):
            func = save_as_e57
        elif self.save_path.endswith(".las"):
            func = save_as_las
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
        print("Try to load %s ..." % file)
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
        self.setData(data=cloud, append=append)
        self.setColorMode(color_mode)
        return cloud

    def setPath(self, path):
        self.save_path = path
