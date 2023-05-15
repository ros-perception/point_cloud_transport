# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Encoding and decoding of point clouds compressed with any point cloud transport."""

from ctypes import c_bool, c_uint8, c_uint32, c_char_p, c_size_t, POINTER

from cras import get_msg_type
from cras.ctypes_utils import Allocator, StringAllocator, BytesAllocator, LogMessagesAllocator, get_ro_c_buffer
from cras.string_utils import BufferStringIO

from .common import _get_base_library, _c_array, _dict_to_config


def _get_library():
    library = _get_base_library()
    # Add function signatures

    library.pointCloudTransportCodecsEncode.restype = c_bool
    library.pointCloudTransportCodecsEncode.argtypes = [
        c_char_p,
        c_uint32, c_uint32, c_size_t, POINTER(c_char_p), POINTER(c_uint32), POINTER(c_uint8), POINTER(c_uint32),
        c_uint8, c_uint32, c_uint32, c_size_t, POINTER(c_uint8), c_uint8,
        Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        c_size_t, POINTER(c_uint8),
        Allocator.ALLOCATOR, Allocator.ALLOCATOR,
    ]

    return library


def encode(raw, topic_or_codec, config=None):
    """Encode the given raw image into a compressed image with a suitable codec.

    :param sensor_msgs.msg.PointCloud2 raw: The raw point cloud.
    :param str topic_or_codec: Name of the topic where this cloud should be published or explicit name of the codec.
    :param config: Configuration of the encoding process.
    :type config: dict or dynamic_reconfigure.msg.Config or None
    :return: Tuple of compressed cloud and error string. If the compression fails, cloud is `None` and error string
             is filled.
    :rtype: (genpy.Message or None, str)
    """
    codec = _get_library()
    if codec is None:
        return None, "Could not load the codec library."

    config = _dict_to_config(config)
    config_buf = BufferStringIO()
    config.serialize(config_buf)
    config_buf_len = config_buf.tell()
    config_buf.seek(0)

    type_allocator = StringAllocator()
    md5sum_allocator = StringAllocator()
    data_allocator = BytesAllocator()
    error_allocator = StringAllocator()
    log_allocator = LogMessagesAllocator()

    args = [
        topic_or_codec.encode("utf-8"),
        raw.height, raw.width,
        len(raw.fields),
        _c_array([f.name.encode("utf-8") for f in raw.fields], c_char_p),
        _c_array([f.offset for f in raw.fields], c_uint32),
        _c_array([f.datatype for f in raw.fields], c_uint8),
        _c_array([f.count for f in raw.fields], c_uint32),
        raw.is_bigendian, raw.point_step, raw.row_step,
        len(raw.data), get_ro_c_buffer(raw.data), raw.is_dense,
        type_allocator.get_cfunc(), md5sum_allocator.get_cfunc(), data_allocator.get_cfunc(),
        c_size_t(config_buf_len), get_ro_c_buffer(config_buf),
        error_allocator.get_cfunc(), log_allocator.get_cfunc(),
    ]

    ret = codec.pointCloudTransportCodecsEncode(*args)

    log_allocator.print_log_messages()
    if ret:
        msg_type = get_msg_type(type_allocator.value)
        compressed = msg_type()
        if md5sum_allocator.value != compressed._md5sum:
            return None, "MD5 sum mismatch for %s: %s vs %s" % (
                type_allocator.value, md5sum_allocator.value, compressed._md5sum)
        compressed.deserialize(data_allocator.value)
        compressed.header = raw.header
        return compressed, ""
    return None, error_allocator.value
