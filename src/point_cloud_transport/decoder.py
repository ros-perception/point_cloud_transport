# Copyright (c) 2023, Czech Technical University in Prague
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Encoding and decoding of point clouds compressed with any point cloud transport."""

from sensor_msgs.msg import PointCloud2, PointField


def decode(compressed, topic_or_codec, config=None):
    """
    Decode the given compressed point cloud encoded with any codec into a raw point cloud.

    :param genpy.Message compressed: The compressed point cloud.
    :param str topic_or_codec: Name of the topic this cloud comes from or explicit name
           of the codec.
    :param config: Configuration of the decoding process.
    :type config: dict or dynamic_reconfigure.msg.Config or None
    :return: Tuple of raw cloud and error string. If the decoding fails, cloud is `None`
             and error string is filled.
    :rtype: (sensor_msgs.msg.PointCloud2 or None, str)
    """
    if codec is None:
        return None, 'Could not load the codec library.'

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
        topic_or_codec.encode('utf-8'),
        compressed._type.encode('utf-8'), compressed._md5sum.encode('utf-8'),
        compressed_buf_len,
        get_ro_c_buffer(compressed_buf),
        byref(raw_height), byref(raw_width),
        byref(raw_num_fields), field_names_allocator.get_cfunc(),
        field_offset_allocator.get_cfunc(),
        field_datatype_allocator.get_cfunc(), field_count_allocator.get_cfunc(),
        byref(raw_is_big_endian), byref(raw_point_step),
        byref(raw_row_step), data_allocator.get_cfunc(),
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
        return raw, ''
    return None, error_allocator.value
