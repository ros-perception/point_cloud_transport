# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

"""Subscriber automatically converting from any transport to raw."""

from ctypes import c_char_p

import dynamic_reconfigure.server
import rospy

from cras import get_msg_type, get_cfg_module
from cras.ctypes_utils import Allocator, StringAllocator

from .common import _get_base_library, _TransportInfo
from .decoder import decode


def _get_library():
    library = _get_base_library()
    # Add function signatures

    library.pointCloudTransportGetLoadableTransports.restype = None
    library.pointCloudTransportGetLoadableTransports.argtypes = [
        Allocator.ALLOCATOR,
    ]

    library.pointCloudTransportGetTopicToSubscribe.restype = None
    library.pointCloudTransportGetTopicToSubscribe.argtypes = [
        c_char_p, c_char_p, Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR, Allocator.ALLOCATOR,
    ]

    return library


def _get_loadable_transports():
    transport_allocator = StringAllocator()
    name_allocator = StringAllocator()
    pct = _get_library()
    pct.pointCloudTransportGetLoadableTransports(transport_allocator.get_cfunc(), name_allocator.get_cfunc())
    return dict(zip(transport_allocator.values, name_allocator.values))


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
        config_type = get_cfg_module(config_type_allocator.value)
        return _TransportInfo(name_allocator.value, topic_allocator.value, data_type, config_type)
    except ImportError as e:
        rospy.logerr("Import error: " + str(e))
        return None


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
