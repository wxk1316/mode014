"""PointCloud2 reader honoring datatype, endianness, offsets and row padding."""
import numpy as np

TYPES = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}


def read_xyzi(msg, intensity_scale=1.):
    if msg.width <= 0 or msg.height <= 0 or msg.point_step <= 0:
        raise ValueError('empty/invalid PointCloud2')
    names, formats, offsets = [], [], []
    wanted = {'x', 'y', 'z', 'intensity'}
    endian = '>' if msg.is_bigendian else '<'
    for f in msg.fields:
        if f.name not in wanted:
            continue
        if f.name in names or f.datatype not in TYPES or f.count != 1:
            raise ValueError('unsupported/duplicate point field '+f.name)
        names.append(f.name)
        formats.append(endian+TYPES[f.datatype])
        offsets.append(f.offset)
    if set(names) != wanted:
        raise ValueError('LiDAR must publish scalar x/y/z/intensity fields')
    dtype = np.dtype(dict(names=names, formats=formats, offsets=offsets, itemsize=msg.point_step))
    if msg.row_step < msg.width*msg.point_step or len(msg.data) < msg.row_step*msg.height:
        raise ValueError('truncated or inconsistent PointCloud2')
    cloud = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
    points = np.column_stack([cloud[k].reshape(-1) for k in ('x', 'y', 'z', 'intensity')]).astype(np.float32)
    points[:, 3] *= intensity_scale
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]
    if len(points) == 0:
        raise ValueError('no finite points')
    return np.ascontiguousarray(points), np.flatnonzero(valid)
