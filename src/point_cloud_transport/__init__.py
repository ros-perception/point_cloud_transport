# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Encoding and decoding of point clouds compressed with any point cloud transport.

Example usage:

.. code-block:: python

    from point_cloud_transport import decode, encode
    from sensor_msgs.msg import PointCloud2

    raw = PointCloud2()
    ... # fill the cloud
    compressed, err = encode(raw, "draco")
    if compressed is None:
      rospy.logerr("Error encoding point cloud: " + err)
      return False
    # work with the compressed instance in variable compressed

    # beware, for decoding, we do not specify "raw", but the codec used for encoding
    raw2, err = decode(compressed, "draco")
    if raw2 is None:
      rospy.logerr("Error decoding point cloud: " + err)
      return False
    # work with the PointCloud2 instance in variable raw2
"""

from ctypes import RTLD_GLOBAL, c_bool, c_uint8, c_uint32, c_char_p, c_size_t, POINTER, byref, sizeof, c_void_p
import time

from dynamic_reconfigure.msg import Config, BoolParameter, DoubleParameter, IntParameter, StrParameter
import dynamic_reconfigure.server
import rospy
from sensor_msgs.msg import PointCloud2, PointField

from cras import get_msg_type
from cras.ctypes_utils import load_library, Allocator, StringAllocator, BytesAllocator, LogMessagesAllocator, \
    get_ro_c_buffer
from cras.string_utils import STRING_TYPE, BufferStringIO


def __dict_to_config(d):
    """Convert configuration dict to :class:`dynamic_reconfigure.msg.Config`.

    :param d: Configuration dict (or already the message, in which case it is just returned).
    :type d: dict or dynamic_reconfigure.msg.Config or None
    :return: The config message.
    :rtype: dynamic_reconfigure.msg.Config
    """
    if d is None:
        return Config()
    if isinstance(d, Config):
        return d
    c = Config()
    for key, value in d.items():
        if isinstance(value, bool):
            c.bools.append(BoolParameter(key, value))
        elif isinstance(value, float):
            c.doubles.append(DoubleParameter(key, value))
        elif isinstance(value, int):
            c.ints.append(IntParameter(key, value))
        elif isinstance(value, STRING_TYPE):
            c.strs.append(StrParameter(key, value))
    return c


__library = None


def _get_library():
    global __library
    if __library is None:
        __library = load_library('point_cloud_transport', mode=RTLD_GLOBAL)
        if __library is None:
            return None

        # Add function signatures

        __library.pointCloudTransportCodecsEncode.restype = c_bool
        __library.pointCloudTransportCodecsEncode.argtypes = [
            c_char_p,
            c_uint32, c_uint32, c_size_t, POINTER(c_char_p), POINTER(c_uint32), POINTER(c_uint8), POINTER(c_uint32),
            c_uint8, c_uint32, c_uint32, c_size_t, POINTER(c_uint8), c_uint8,
            Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
            c_size_t, POINTER(c_uint8),
            Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        ]

        __library.pointCloudTransportCodecsDecode.restype = c_bool
        __library.pointCloudTransportCodecsDecode.argtypes = [
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

        __library.pointCloudTransportGetLoadableTransports.restype = None
        __library.pointCloudTransportGetLoadableTransports.argtypes = [
            Allocator.ALLOCATOR,
        ]

        __library.pointCloudTransportGetTopicsToPublish.restype = None
        __library.pointCloudTransportGetTopicsToPublish.argtypes = [
            c_char_p,
            Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        ]

        __library.pointCloudTransportGetTopicToSubscribe.restype = None
        __library.pointCloudTransportGetTopicToSubscribe.argtypes = [
            c_char_p, c_char_p, Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        ]

    return __library


def _c_array(data, c_type):
    c_data = (c_type * (len(data) + 1))()
    c_data[:-1] = data
    c_data[-1] = 0
    return c_data


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

    config = __dict_to_config(config)
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

    config = __dict_to_config(config)
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


def _get_loadable_transports():
    transport_allocator = StringAllocator()
    name_allocator = StringAllocator()
    pct = _get_library()
    pct.pointCloudTransportGetLoadableTransports(transport_allocator.get_cfunc(), name_allocator.get_cfunc())
    return dict(zip(transport_allocator.values, name_allocator.values))


class _TransportInfo(object):
    def __init__(self, name, topic, data_type, config_data_type=None):
        self.name = name
        self.topic = topic
        self.data_type = data_type
        self.config_data_type = config_data_type


__pkg_modules = dict()


def get_cfg_type(cfg_type_str):
    """Load and return a Python module corresponding to the given ROS dynamic reconfigure type.

    :param str cfg_type_str: Type of the config in textual form (e.g. `compressed_image_transport/CompressedPublisher`).
    :return: The Python module.
    """
    if len(cfg_type_str) == 0:
        return None
    pkg, cfg = cfg_type_str.split('/')
    cfg += 'Config'
    cfg_module = pkg + '.cfg.' + cfg
    if cfg_module not in __pkg_modules:
        import importlib
        __pkg_modules[cfg_module] = importlib.import_module(cfg_module)
    return __pkg_modules[cfg_module]


def _get_topics_to_publish(base_topic):
    transport_allocator = StringAllocator()
    name_allocator = StringAllocator()
    topic_allocator = StringAllocator()
    data_type_allocator = StringAllocator()
    config_type_allocator = StringAllocator()
    pct = _get_library()

    pct.pointCloudTransportGetTopicsToPublish(
        base_topic.encode("utf-8"), transport_allocator.get_cfunc(), name_allocator.get_cfunc(),
        topic_allocator.get_cfunc(), data_type_allocator.get_cfunc(), config_type_allocator.get_cfunc())

    topics = {}

    for i in range(len(transport_allocator.values)):
        try:
            data_type = get_msg_type(data_type_allocator.values[i])
            config_type = get_cfg_type(config_type_allocator.values[i])
            topics[transport_allocator.values[i]] = \
                _TransportInfo(name_allocator.values[i], topic_allocator.values[i], data_type, config_type)
        except ImportError as e:
            rospy.logerr("Import error: " + str(e))

    return topics


def _get_topic_to_subscribe(base_topic, transport):
    name_allocator = StringAllocator()
    topic_allocator = StringAllocator()
    data_type_allocator = StringAllocator()
    config_type_allocator = StringAllocator()
    pct = _get_library()

    pct.pointCloudTransportGetTopicToSubscribe(
        base_topic.encode("utf-8"), transport.encode("utf-8"), name_allocator.get_cfunc(), topic_allocator.get_cfunc(),
        data_type_allocator.get_cfunc(), config_type_allocator.get_cfunc())

    if len(data_type_allocator.values) == 0:
        return None

    try:
        data_type = get_msg_type(data_type_allocator.value)
        config_type = get_cfg_type(config_type_allocator.value)
        return _TransportInfo(name_allocator.value, topic_allocator.value, data_type, config_type)
    except ImportError as e:
        rospy.logerr("Import error: " + str(e))
        return None


class Publisher(object):
    def __init__(self, base_topic, *args, **kwargs):
        self.base_topic = rospy.names.resolve_name(base_topic)
        self.transports = _get_topics_to_publish(self.base_topic)

        blacklist = set(rospy.get_param(self.base_topic + "/disable_pub_plugins", []))

        self.publishers = {}
        self.config_servers = {}
        for transport in self.transports:
            if transport in blacklist:
                continue
            topic_to_publish = self.transports[transport]
            self.publishers[transport] = rospy.Publisher(
                topic_to_publish.topic, topic_to_publish.data_type, *args, **kwargs)
            if topic_to_publish.config_data_type is not None:
                self.config_servers[transport] = dynamic_reconfigure.server.Server(
                    topic_to_publish.config_data_type, lambda conf, _: conf, namespace=topic_to_publish.topic)

    def get_num_subscribers(self):
        sum([p.get_num_connections() for p in self.publishers.values()])

    def get_topic(self):
        return self.base_topic

    def publish(self, raw):
        for transport, transport_info in self.transports.items():
            config = self.config_servers[transport].config if transport in self.config_servers else None
            compressed, err = encode(raw, transport_info.name, config)
            if compressed is not None:
                self.publishers[transport].publish(compressed)
            else:
                rospy.logerr(err)

    def shutdown(self):
        for publisher in self.publishers.values():
            publisher.unregister()


class Subscriber(object):
    def __init__(self, base_topic, callback, callback_args=None, default_transport="raw",
                 parameter_namespace="~", parameter_name="point_cloud_transport", *args, **kwargs):
        self.base_topic = rospy.names.resolve_name(base_topic)
        self.callback = callback
        self.callback_args = callback_args
        self.transport = rospy.get_param(rospy.names.ns_join(parameter_namespace, parameter_name), default_transport)

        transports = _get_loadable_transports()
        if self.transport not in transports and self.transport not in transports.values():
            raise RuntimeError("Point cloud transport '%s' not found." % (self.transport,))

        self.transport_info = _get_topic_to_subscribe(self.base_topic, self.transport)
        if self.transport_info is None:
            raise RuntimeError("Point cloud transport '%s' not found." % (self.transport,))

        self.subscriber = rospy.Subscriber(
            self.transport_info.topic, self.transport_info.data_type, self.cb, *args, **kwargs)
        if self.transport_info.config_data_type is not None:
            self.config_server = dynamic_reconfigure.server.Server(
                self.transport_info.config_data_type, lambda conf, _: conf,
                namespace=rospy.names.ns_join(parameter_namespace, self.transport_info.name))
        else:
            self.config_server = None

    def cb(self, msg):
        config = self.config_server.config if self.config_server is not None else None
        raw, err = decode(msg, self.transport_info.name, config)
        if raw is not None:
            if self.callback_args is None:
                self.callback(raw)
            else:
                self.callback(raw, self.callback_args)
        else:
            rospy.logerr(err)

    def get_num_publishers(self):
        return self.subscriber.get_num_connections()

    def get_topic(self):
        return self.base_topic

    def shutdown(self):
        self.subscriber.unregister()


if __name__ == '__main__':
    def main():
        import rospy
        raw = PointCloud2()
        raw.header.stamp = rospy.Time(10)
        raw.header.frame_id = "test"
        raw.width = raw.height = 2
        raw.is_bigendian = False
        raw.is_dense = True
        raw.point_step = 8
        raw.row_step = raw.point_step * 2
        raw.fields = [
            PointField(name="x", offset=0, datatype=PointField.INT32, count=1),
            PointField(name="y", offset=0, datatype=PointField.INT32, count=1),
        ]

        import struct
        data = struct.pack('i', 0) + struct.pack('i', 1000) + struct.pack('i', 5000) + struct.pack('i', 9999)
        data += struct.pack('i', 0) + struct.pack('i', -1000) + struct.pack('i', -5000) + struct.pack('i', -9999)
        import sys
        if sys.version_info[0] == 2:
            raw.data = map(ord, data)

        rospy.init_node("test_node")
        rospy.loginfo("start")
        time.sleep(1)

        start = time.time()
        for i in range(1):
            compressed, err = encode(raw, "draco", {"encode_speed": 1})
            # compressed, err = encode(raw, "draco")
            if compressed is None:
                print(err)
                break

            # raw2, err = decode(compressed, "draco", {"SkipDequantizationPOSITION": True})
            raw2, err = decode(compressed, "draco")

        end = time.time()
        print(end - start)
        print(bool(raw))
        print(bool(compressed))
        print(err)
        print(raw.fields == raw2.fields)
        print(raw == raw2)

        def cb(msg):
            print(msg == raw)  # they are not equal exactly...

        def cb_ros(msg):
            print(msg == compressed)

        sub = Subscriber("test", cb, default_transport="draco", queue_size=1)
        sub2 = rospy.Subscriber("test", PointCloud2, cb, queue_size=1)
        sub3 = rospy.Subscriber("test/draco", type(compressed), cb_ros, queue_size=1)
        pub = Publisher("test", queue_size=10)
        time.sleep(0.2)
        pub.publish(raw)
        rospy.spin()

    main()
