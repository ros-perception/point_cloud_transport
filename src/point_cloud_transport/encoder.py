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

def encode(raw, topic_or_codec, config=None):
    """
    Encode the given raw point_cloud into a compressed point_cloud with a suitable codec.

    :param sensor_msgs.msg.PointCloud2 raw: The raw point cloud.
    :param str topic_or_codec: Name of the topic where this cloud should be published or explicit
           name of the codec.
    :param config: Configuration of the encoding process.
    :type config: dict or dynamic_reconfigure.msg.Config or None
    :return: Tuple of compressed cloud and error string. If the compression fails, cloud is `None`
             and error string is filled.
    :rtype: (genpy.Message or None, str)
    """

    config = dict_to_dynamic_config_msg(config)
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
        topic_or_codec.encode('utf-8'),
        raw.height, raw.width,
        len(raw.fields),
        c_array([f.name.encode('utf-8') for f in raw.fields], c_char_p),
        c_array([f.offset for f in raw.fields], c_uint32),
        c_array([f.datatype for f in raw.fields], c_uint8),
        c_array([f.count for f in raw.fields], c_uint32),
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
            return None, 'MD5 sum mismatch for %s: %s vs %s' % (
                type_allocator.value, md5sum_allocator.value, compressed._md5sum)
        compressed.deserialize(data_allocator.value)
        compressed.header = raw.header
        return compressed, ''
    return None, error_allocator.value
