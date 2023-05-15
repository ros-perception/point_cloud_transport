# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Encoding and decoding of point clouds compressed with any point cloud transport."""

from ctypes import c_bool, c_uint8, c_uint32, c_char_p, c_size_t, POINTER, byref, sizeof

from sensor_msgs.msg import PointCloud2, PointField

from cras.ctypes_utils import Allocator, StringAllocator, BytesAllocator, LogMessagesAllocator, get_ro_c_buffer
from cras.message_utils import dict_to_dynamic_config_msg
from cras.string_utils import BufferStringIO

from .common import _get_base_library


def _get_library():
    library = _get_base_library()
    # Add function signatures

    library.pointCloudTransportCodecsDecode.restype = c_bool
    library.pointCloudTransportCodecsDecode.argtypes = [
        c_char_p,
        c_char_p, c_char_p, c_size_t, POINTER(c_uint8),
        POINTER(c_uint32), POINTER(c_uint32),
        POINTER(c_size_t), Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        POINTER(c_uint8), POINTER(c_uint32), POINTER(c_uint32),
        Allocator.ALLOCATOR,
        POINTER(c_uint8),
        c_size_t, POINTER(c_uint8),
        Allocator.ALLOCATOR, Allocator.ALLOCATOR,
    ]

    return library


class _NumberAllocator(Allocator):
    """ctypes allocator suitable for allocating numbers. The returned value is a number."""

    def __init__(self, c_type):
        super(_NumberAllocator, self).__init__()
        self._c_type = c_type

    def _alloc(self, size):
        if size != sizeof(self._c_type):
            raise RuntimeError("NumberAllocator can only handle size 1 allocations.")
        return (self._c_type * 1)()

    @property
    def value(self):
        if len(self.allocated) == 0:
            return None
        return self.allocated[0][0]

    @property
    def values(self):
        return [a[0] for a in self.allocated]


def decode(compressed, topic_or_codec, config=None):
    """Decode the given compressed point cloud encoded with any codec into a raw point cloud.

    :param genpy.Message compressed: The compressed point cloud.
    :param str topic_or_codec: Name of the topic this cloud comes from or explicit name of the codec.
    :param config: Configuration of the decoding process.
    :type config: dict or dynamic_reconfigure.msg.Config or None
    :return: Tuple of raw cloud and error string. If the decoding fails, cloud is `None` and error string is filled.
    :rtype: (sensor_msgs.msg.PointCloud2 or None, str)
    """
    codec = _get_library()
    if codec is None:
        return None, "Could not load the codec library."

    field_names_allocator = StringAllocator()
    field_offset_allocator = _NumberAllocator(c_uint32)
    field_datatype_allocator = _NumberAllocator(c_uint8)
    field_count_allocator = _NumberAllocator(c_uint32)
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    raw_height = c_uint32()
    raw_width = c_uint32()
    raw_is_big_endian = c_uint8()
    raw_num_fields = c_size_t()
    raw_point_step = c_uint32()
    raw_row_step = c_uint32()
    raw_is_dense = c_uint8()

    compressed_buf = BufferStringIO()
    compressed.serialize(compressed_buf)
    compressed_buf_len = compressed_buf.tell()
    compressed_buf.seek(0)

    config = dict_to_dynamic_config_msg(config)
    config_buf = BufferStringIO()
    config.serialize(config_buf)
    config_buf_len = config_buf.tell()
    config_buf.seek(0)

    args = [
        topic_or_codec.encode("utf-8"),
        compressed._type.encode("utf-8"), compressed._md5sum.encode("utf-8"), compressed_buf_len,
        get_ro_c_buffer(compressed_buf),
        byref(raw_height), byref(raw_width),
        byref(raw_num_fields), field_names_allocator.get_cfunc(), field_offset_allocator.get_cfunc(),
        field_datatype_allocator.get_cfunc(), field_count_allocator.get_cfunc(),
        byref(raw_is_big_endian), byref(raw_point_step), byref(raw_row_step), data_allocator.get_cfunc(),
        byref(raw_is_dense),
        c_size_t(config_buf_len), get_ro_c_buffer(config_buf),
        error_allocator.get_cfunc(), log_allocator.get_cfunc(),
    ]
    ret = codec.pointCloudTransportCodecsDecode(*args)

    log_allocator.print_log_messages()
    if ret:
        raw = PointCloud2()
        if hasattr(compressed, 'header'):
            raw.header = compressed.header
        raw.height = raw_height.value
        raw.width = raw_width.value
        for i in range(raw_num_fields.value):
            f = PointField()
            f.name = field_names_allocator.values[i]
            f.offset = field_offset_allocator.values[i]
            f.datatype = field_datatype_allocator.values[i]
            f.count = field_count_allocator.values[i]
            raw.fields.append(f)
        raw.is_bigendian = bool(raw_is_big_endian.value)
        raw.point_step = raw_point_step.value
        raw.row_step = raw_row_step.value
        raw.data = data_allocator.value
        import sys
        if sys.version_info[0] == 2:
            raw.data = map(ord, raw.data)
        raw.is_dense = bool(raw_is_dense.value)
        return raw, ""
    return None, error_allocator.value
