"""Preserve complete PointCloud2 records while removing invalid XYZ samples."""
import sys
import numpy as np


def finite_records(cloud):
    if cloud.is_bigendian or sys.byteorder != 'little':
        raise ValueError('Only little-endian RealSense point clouds are supported')
    if cloud.height <= 0 or cloud.width <= 0 or cloud.point_step <= 0:
        raise ValueError('Empty point cloud')
    if (cloud.row_step < cloud.width * cloud.point_step or
            len(cloud.data) < cloud.height * cloud.row_step):
        raise ValueError('Malformed point-cloud buffer')
    offsets = []
    for name in ('x', 'y', 'z'):
        field = next((f for f in cloud.fields if f.name == name), None)
        if (field is None or field.datatype != 7 or field.count != 1 or
                field.offset < 0 or field.offset + 4 > cloud.point_step):
            raise ValueError('Point cloud requires FLOAT32 XYZ fields')
        offsets.append(field.offset)
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=['<f4']*3,
                          offsets=offsets, itemsize=cloud.point_step))
    points = np.ndarray((cloud.height, cloud.width), dtype=dtype,
                        buffer=memoryview(cloud.data), strides=(cloud.row_step, cloud.point_step))
    valid = np.isfinite(points['x']) & np.isfinite(points['y']) & np.isfinite(points['z'])
    valid &= (points['x'] != 0) | (points['y'] != 0) | (points['z'] != 0)
    records = np.ndarray((cloud.height, cloud.width, cloud.point_step), dtype=np.uint8,
                         buffer=memoryview(cloud.data),
                         strides=(cloud.row_step, cloud.point_step, 1))
    return int(np.count_nonzero(valid)), records[valid].tobytes()
