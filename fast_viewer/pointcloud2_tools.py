import numpy as np
from base64 import standard_b64decode
from operator import attrgetter

DUMMY_FIELD_PREFIX = '__'
INT8 = 1
UINT8 = 2
INT16 = 3
UINT16 = 4
INT32 = 5
UINT32 = 6
FLOAT32 = 7
FLOAT64 = 8
# mappings between PointField types and numpy types
type_mappings = [(INT8, np.dtype('int8')), (UINT8, np.dtype('uint8')), (INT16, np.dtype('int16')),
                 (UINT16, np.dtype('uint16')), (INT32, np.dtype('int32')), (UINT32, np.dtype('uint32')),
                 (FLOAT32, np.dtype('float32')), (FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)
# sizes (in bytes) of PointField types
pftype_sizes = {INT8: 1, UINT8: 1, INT16: 2, UINT16: 2,
                INT32: 4, UINT32: 4, FLOAT32: 4, FLOAT64: 8}


def json_fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    fields = sorted(fields, key=lambda x: x['offset'])
    for f in fields:
        while offset < f["offset"]:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f["datatype"]]
        if f["count"] != 1:
            dtype = np.dtype((dtype, f["count"]))

        np_dtype_list.append((f["name"], dtype))
        offset += pftype_sizes[f["datatype"]] * f["count"]

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def json_pointcloud2_to_array(cloud_msg, squeeze=True):
    '''
    Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the height is 1.

    The reason for using np.frombuffer rather than struct.unpack is speed... especially
    for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = json_fields_to_dtype(cloud_msg["fields"], cloud_msg["point_step"])

    # parse the cloud into an array
    cloud_arr = np.frombuffer(standard_b64decode(cloud_msg["data"]), dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg["height"] == 1:
        return np.reshape(cloud_arr, (cloud_msg["width"],))
    else:
        return np.reshape(cloud_arr, (cloud_msg["height"], cloud_msg["width"]))


def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    fields = sorted(fields, key=attrgetter('offset'))
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_sizes[f.datatype] * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


def pointcloud2_to_array(cloud_msg, squeeze=True):
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

