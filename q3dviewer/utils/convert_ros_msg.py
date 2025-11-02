"""
Copyright 2024 Panasonic Advanced Technology Development Co.,Ltd. (Liu Yang)
Distributed under MIT license. See LICENSE for more information.
"""

import numpy as np
from q3dviewer.utils.maths import make_transform

def get_dtype(msg):
    dtype_map = {
        1: 'i1',   # INT8
        2: 'u1',   # UINT8  
        3: 'i2',   # INT16
        4: 'u2',   # UINT16
        5: 'i4',   # INT32
        6: 'u4',   # UINT32
        7: 'f4',   # FLOAT32
        8: 'f8'    # FLOAT64
    }
    
    point_step = msg.point_step
    dtype_list = []
    for field in msg.fields:
        dtype_str = dtype_map.get(field.datatype, 'f4')
        dtype_list.append((field.name, dtype_str, field.offset))
    
    dtype_list.sort(key=lambda x: x[2])
    
    structured_dtype = []
    last_offset = 0
    
    for field_name, field_dtype, offset in dtype_list:
        if offset > last_offset:
            padding_size = offset - last_offset
            if padding_size > 0:
                structured_dtype.append((f'pad{last_offset}', f'u1', padding_size))
        
        structured_dtype.append((field_name, field_dtype))
        last_offset = offset + np.dtype(field_dtype).itemsize
    
    if point_step > last_offset:
        final_padding = point_step - last_offset
        structured_dtype.append((f'pad{last_offset}', f'u1', final_padding))
    return structured_dtype


def convert_pointcloud2_msg(msg):
    # Build dtype with proper offsets for PointCloud2 message
    structured_dtype = get_dtype(msg)
    pc = np.frombuffer(msg.data, dtype=structured_dtype)
    # pc = PointCloud.from_msg(msg).pc_data
    data_type = [('xyz', '<f4', (3,)), ('irgb', '<u4')]
    rgb = np.zeros([pc.shape[0]], dtype=np.uint32)
    intensity = np.zeros([pc.shape[0]], dtype=np.uint32)
    fields = ['xyz']
    if 'intensity' in pc.dtype.names:
        intensity = pc['intensity']
        intensity = np.clip(intensity, 0, 255)
        intensity = intensity.astype(np.uint32)
        fields.append('intensity')
    if 'rgb' in pc.dtype.names:
        rgb = pc['rgb'].view(np.uint32)
        fields.append('rgb')
    irgb = (intensity << 24) | rgb
    xyz = np.stack([pc['x'], pc['y'], pc['z']], axis=1)
    cloud = np.rec.fromarrays([xyz, irgb], dtype=data_type)
    stamp = msg.header.stamp.to_sec()
    return cloud, fields, stamp


def convert_odometry_msg(msg):
    pose = np.array(
        [msg.pose.pose.position.x,
         msg.pose.pose.position.y,
         msg.pose.pose.position.z])
    rotation = np.array([
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w])
    transform = make_transform(pose, rotation)
    stamp = msg.header.stamp.to_sec()
    return transform, stamp


def convert_image_msg(msg, bgr=False):
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
        msg.height, msg.width, -1)
    # if bgr is True return BGR image, else return RGB image
    if (bgr and msg.encoding == 'rgb8') or (not bgr and msg.encoding == 'bgr8'):
        image = image[:, :, ::-1]

    stamp = msg.header.stamp.to_sec()
    return image, stamp
