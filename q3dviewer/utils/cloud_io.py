"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
import meshio
from pypcd4 import PointCloud, MetaData
from pye57 import E57
import laspy


def save_ply(cloud, save_path):
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


def save_pcd(cloud, save_path):
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


def save_e57(cloud, save_path):
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

def save_las(cloud, save_path):
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


def gsdata_type(sh_dim):
    return [('pw', '<f4', (3,)),
            ('rot', '<f4', (4,)),
            ('scale', '<f4', (3,)),
            ('alpha', '<f4'),
            ('sh', '<f4', (sh_dim))]


def matrix_to_quaternion_wxyz(matrices):
    m00, m01, m02 = matrices[:, 0, 0], matrices[:, 0, 1], matrices[:, 0, 2]
    m10, m11, m12 = matrices[:, 1, 0], matrices[:, 1, 1], matrices[:, 1, 2]
    m20, m21, m22 = matrices[:, 2, 0], matrices[:, 2, 1], matrices[:, 2, 2]
    t = 1 + m00 + m11 + m22
    s = np.ones_like(m00)
    w = np.ones_like(m00)
    x = np.ones_like(m00)
    y = np.ones_like(m00)
    z = np.ones_like(m00)

    t_positive = t > 0.0000001
    s[t_positive] = 0.5 / np.sqrt(t[t_positive])
    w[t_positive] = 0.25 / s[t_positive]
    x[t_positive] = (m21[t_positive] - m12[t_positive]) * s[t_positive]
    y[t_positive] = (m02[t_positive] - m20[t_positive]) * s[t_positive]
    z[t_positive] = (m10[t_positive] - m01[t_positive]) * s[t_positive]

    c1 = np.logical_and(m00 > m11, m00 > m22)
    cond1 = np.logical_and(np.logical_not(t_positive),
                           np.logical_and(m00 > m11, m00 > m22))

    s[cond1] = 2.0 * np.sqrt(1.0 + m00[cond1] - m11[cond1] - m22[cond1])
    w[cond1] = (m21[cond1] - m12[cond1]) / s[cond1]
    x[cond1] = 0.25 * s[cond1]
    y[cond1] = (m01[cond1] + m10[cond1]) / s[cond1]
    z[cond1] = (m02[cond1] + m20[cond1]) / s[cond1]

    c2 = np.logical_and(np.logical_not(c1), m11 > m22)
    cond2 = np.logical_and(np.logical_not(t_positive), c2)
    s[cond2] = 2.0 * np.sqrt(1.0 + m11[cond2] - m00[cond2] - m22[cond2])
    w[cond2] = (m02[cond2] - m20[cond2]) / s[cond2]
    x[cond2] = (m01[cond2] + m10[cond2]) / s[cond2]
    y[cond2] = 0.25 * s[cond2]
    z[cond2] = (m12[cond2] + m21[cond2]) / s[cond2]

    c3 = np.logical_and(np.logical_not(c1), np.logical_not(c2))
    cond3 = np.logical_and(np.logical_not(t_positive), c3)
    s[cond3] = 2.0 * np.sqrt(1.0 + m22[cond3] - m00[cond3] - m11[cond3])
    w[cond3] = (m10[cond3] - m01[cond3]) / s[cond3]
    x[cond3] = (m02[cond3] + m20[cond3]) / s[cond3]
    y[cond3] = (m12[cond3] + m21[cond3]) / s[cond3]
    z[cond3] = 0.25 * s[cond3]
    return np.array([w, x, y, z]).T


def load_gs_ply(path, T=None):
    mesh = meshio.read(path)
    vertices = mesh.points
    pws = vertices[:, :3]

    alphas = mesh.point_data['opacity']
    alphas = 1 / (1 + np.exp(-alphas))

    scales = np.vstack((mesh.point_data['scale_0'],
                        mesh.point_data['scale_1'],
                        mesh.point_data['scale_2'])).T

    rots = np.vstack((mesh.point_data['rot_0'],
                      mesh.point_data['rot_1'],
                      mesh.point_data['rot_2'],
                      mesh.point_data['rot_3'])).T
    rots /= np.linalg.norm(rots, axis=1)[:, np.newaxis]

    sh_dim = len(mesh.point_data) - 11
    shs = np.zeros([pws.shape[0], sh_dim])
    shs[:, 0] = mesh.point_data['f_dc_0']
    shs[:, 1] = mesh.point_data['f_dc_1']
    shs[:, 2] = mesh.point_data['f_dc_2']

    sh_rest_dim = sh_dim - 3
    if sh_rest_dim > 0:
        for i in range(sh_rest_dim):
            name = f"f_rest_{i}"
            shs[:, 3 + i] = mesh.point_data[name]
        shs[:, 3:] = shs[:, 3:].reshape(-1, 3, sh_rest_dim // 3).transpose([0, 2, 1]).reshape(-1, sh_rest_dim)

    pws = pws.astype(np.float32)
    rots = rots.astype(np.float32)
    scales = np.exp(scales).astype(np.float32)
    alphas = alphas.astype(np.float32)
    shs = shs.astype(np.float32)

    dtypes = gsdata_type(sh_dim)

    gs = np.rec.fromarrays(
        [pws, rots, scales, alphas, shs], dtype=dtypes)

    return gs


def rotate_gaussian(T, gs):
    # Transform to world
    pws = (T @ gs['pw'].T).T
    w = gs['rot'][:, 0]
    x = gs['rot'][:, 1]
    y = gs['rot'][:, 2]
    z = gs['rot'][:, 3]
    R = np.array([
        [1.0 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x * z + y * w)],
        [2*(x*y + z*w), 1.0 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1.0 - 2*(x**2 + y**2)]
    ]).transpose(2, 0, 1)
    R_new = T @ R
    rots = matrix_to_quaternion_wxyz(R_new)
    gs['pw'] = pws
    gs['rot'] = rots
    return gs


def load_gs(fn):
    if fn.endswith('.ply'):
        return load_gs_ply(fn)
    elif fn.endswith('.npy'):
        return np.load(fn)
    else:
        print("%s is not a supported file." % fn)
        exit(0)


def save_gs(fn, gs):
    np.save(fn, gs)


def get_example_gs():
    gs_data = np.array([[0.,  0.,  0.,  # xyz
                         1.,  0.,  0., 0.,  # rot
                         0.05,  0.05,  0.05,  # size
                         1.,
                         1.772484,  -1.772484,  1.772484],
                        [1.,  0.,  0.,
                         1.,  0.,  0., 0.,
                         0.2,  0.05,  0.05,
                         1.,
                         1.772484,  -1.772484, -1.772484],
                        [0.,  1.,  0.,
                         1.,  0.,  0., 0.,
                         0.05,  0.2,  0.05,
                         1.,
                         -1.772484, 1.772484, -1.772484],
                        [0.,  0.,  1.,
                         1.,  0.,  0., 0.,
                         0.05,  0.05,  0.2,
                         1.,
                         -1.772484, -1.772484,  1.772484]
                        ], dtype=np.float32)
    dtypes = gsdata_type(3)
    gs = np.frombuffer(gs_data.tobytes(), dtype=dtypes)
    return gs


if __name__ == "__main__":
    gs = load_gs("/home/liu/tmp.ply")
    print(gs.shape)
