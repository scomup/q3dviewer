"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

from q3dviewer.custom_items.cloud_item import CloudItem
from pathlib import Path
import os
from PySide6.QtWidgets import QPushButton, QLabel, QLineEdit, QMessageBox
import numpy as np
from pypcd4 import PointCloud, MetaData
from pye57 import E57
import laspy
import meshio


def save_as_ply(cloud, save_path):
    xyz = cloud['xyz']
    i = (cloud['irgb'] & 0xFF000000) >> 24
    rgb = cloud['irgb'] & 0x00FFFFFF
    mesh = meshio.Mesh(points=xyz, cells=[], point_data={
                       "rgb": rgb, "intensity": i})
    mesh.write(save_path, file_format="ply")


def load_ply(file):
    mesh = meshio.read(file)
    xyz = mesh.points
    rgb = np.zeros([xyz.shape[0]], dtype=np.uint32)
    intensity = np.zeros([xyz.shape[0]], dtype=np.uint32)
    color_mode = 'FLAT'
    if "intensity" in mesh.point_data:
        intensity = mesh.point_data["intensity"]
        color_mode = 'I'
    if "rgb" in mesh.point_data:
        rgb = mesh.point_data["rgb"]
        color_mode = 'RGB'
    irgb = (intensity << 24) | rgb
    dtype = [('xyz', '<f4', (3,)), ('irgb', '<u4')]
    cloud = np.rec.fromarrays([xyz, irgb], dtype=dtype)
    return cloud, color_mode


def save_as_pcd(cloud, save_path):
    fields = ('x', 'y', 'z', 'intensity', 'rgb')
    metadata = MetaData.model_validate(
        {
            "fields": fields,
            "size": [4, 4, 4, 4, 4],
            "type": ['F', 'F', 'F', 'U', 'U'],
            "count": [1, 1, 1, 1, 1],
            "width": cloud.shape[0],
            "points": cloud.shape[0],
        })
    i = (cloud['irgb'] & 0xFF000000) >> 24
    rgb = cloud['irgb'] & 0x00FFFFFF

    dtype = [('xyz', '<f4', (3,)), ('intensity', '<u4'), ('rgb', '<u4')]
    tmp = np.rec.fromarrays([cloud['xyz'], i, rgb], dtype=dtype)

    PointCloud(metadata, tmp).save(save_path)


def load_pcd(file):
    dtype = [('xyz', '<f4', (3,)), ('irgb', '<u4')]
    pc = PointCloud.from_path(file).pc_data
    rgb = np.zeros([pc.shape[0]], dtype=np.uint32)
    intensity = np.zeros([pc.shape[0]], dtype=np.uint32)
    color_mode = 'FLAT'
    if 'intensity' in pc.dtype.names:
        intensity = pc['intensity'].astype(np.uint32)
        color_mode = 'I'
    if 'rgb' in pc.dtype.names:
        rgb = pc['rgb'].astype(np.uint32)
        color_mode = 'RGB'
    irgb = (intensity << 24) | rgb
    xyz = np.stack([pc['x'], pc['y'], pc['z']], axis=1)
    cloud = np.rec.fromarrays([xyz, irgb], dtype=dtype)
    return cloud, color_mode


def save_as_e57(cloud, save_path):
    e57 = E57(save_path, mode='w')
    x = cloud['xyz'][:, 0]
    y = cloud['xyz'][:, 1]
    z = cloud['xyz'][:, 2]
    i = (cloud['irgb'] & 0xFF000000) >> 24
    r = (cloud['irgb'] & 0x00FF0000) >> 16
    g = (cloud['irgb'] & 0x0000FF00) >> 8
    b = (cloud['irgb'] & 0x000000ff)
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
    rgb = np.zeros([x.shape[0]], dtype=np.uint32)
    intensity = np.zeros([x.shape[0]], dtype=np.uint32)
    color_mode = 'FLAT'
    if "intensity" in scans:
        intensity = scans["intensity"].astype(np.uint32)
        color_mode = 'I'
    if all([x in scans for x in ["colorRed", "colorGreen", "colorBlue"]]):
        r = scans["colorRed"].astype(np.uint32)
        g = scans["colorGreen"].astype(np.uint32)
        b = scans["colorBlue"].astype(np.uint32)
        rgb = (r << 16) | (g << 8) | b
        color_mode = 'RGB'
    irgb = (intensity << 24) | rgb
    dtype = [('xyz', '<f4', (3,)), ('irgb', '<u4')]
    cloud = np.rec.fromarrays(
        [np.stack([x, y, z], axis=1), irgb],
        dtype=dtype)
    e57.close()
    return cloud, color_mode


def load_las(file):
    with laspy.open(file) as f:
        las = f.read()
        xyz = np.vstack((las.x, las.y, las.z)).transpose()
        dimensions = list(las.point_format.dimension_names)
        color_mode = 'FLAT'
        rgb = np.zeros([las.x.shape[0]], dtype=np.uint32)
        intensity = np.zeros([las.x.shape[0]], dtype=np.uint32)
        if 'intensity' in dimensions:
            intensity = las.intensity.astype(np.uint32)
            color_mode = 'I'
        if 'red' in dimensions and 'green' in dimensions and 'blue' in dimensions:
            red = las.red
            green = las.green
            blue = las.blue
            max_val = np.max([red, green, blue])
            if red.dtype == np.dtype('uint16') and max_val > 255:
                red = (red / 65535.0 * 255).astype(np.uint32)
                green = (green / 65535.0 * 255).astype(np.uint32)
                blue = (blue / 65535.0 * 255).astype(np.uint32)
            rgb = (red << 16) | (green << 8) | blue
            color_mode = 'RGB'
        color = (intensity << 24) | rgb
        dtype = [('xyz', '<f4', (3,)), ('irgb', '<u4')]
        cloud = np.rec.fromarrays([xyz, color], dtype=dtype)
    return cloud, color_mode

def save_as_las(cloud, save_path):
    header = laspy.LasHeader(point_format=3, version="1.2")
    las = laspy.LasData(header)
    las.x = cloud['xyz'][:, 0]
    las.y = cloud['xyz'][:, 1]
    las.z = cloud['xyz'][:, 2]
    las.red = (cloud['irgb'] >> 16) & 0xFF
    las.green = (cloud['irgb'] >> 8) & 0xFF
    las.blue = cloud['irgb'] & 0xFF
    las.intensity = cloud['irgb'] >> 24
    las.write(save_path)


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
