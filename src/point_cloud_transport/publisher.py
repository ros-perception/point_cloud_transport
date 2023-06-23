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
"""Publisher that automatically publishes to all declared transports."""

from ctypes import c_char_p

from cras import get_cfg_module, get_msg_type
from cras.ctypes_utils import Allocator, StringAllocator

import dynamic_reconfigure.server
import rospy

from .common import _get_base_library, _TransportInfo
from .encoder import encode


def _get_library():
    library = _get_base_library()

    library.pointCloudTransportGetLoadableTransports.restype = None
    library.pointCloudTransportGetLoadableTransports.argtypes = [
        Allocator.ALLOCATOR,
    ]

    library.pointCloudTransportGetTopicsToPublish.restype = None
    library.pointCloudTransportGetTopicsToPublish.argtypes = [
        c_char_p,
        Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        Allocator.ALLOCATOR, Allocator.ALLOCATOR,
        Allocator.ALLOCATOR,
    ]

    return library


def _get_topics_to_publish(base_topic):
    transport_allocator = StringAllocator()
    name_allocator = StringAllocator()
    topic_allocator = StringAllocator()
    data_type_allocator = StringAllocator()
    config_type_allocator = StringAllocator()
    pct = _get_library()

    pct.pointCloudTransportGetTopicsToPublish(
        base_topic.encode('utf-8'), transport_allocator.get_cfunc(), name_allocator.get_cfunc(),
        topic_allocator.get_cfunc(), data_type_allocator.get_cfunc(),
        config_type_allocator.get_cfunc())

    topics = {}

    for i in range(len(transport_allocator.values)):
        try:
            data_type = get_msg_type(data_type_allocator.values[i])
            config_type = get_cfg_module(config_type_allocator.values[i])
            topics[transport_allocator.values[i]] = \
                _TransportInfo(name_allocator.values[i], topic_allocator.values[i],
                               data_type, config_type)
        except ImportError as e:
            rospy.logerr('Import error: ' + str(e))

    return topics


class Publisher(object):

    def __init__(self, base_topic, *args, **kwargs):
        self.base_topic = rospy.names.resolve_name(base_topic)
        self.transports = _get_topics_to_publish(self.base_topic)

        blacklist = set(rospy.get_param(self.base_topic + '/disable_pub_plugins', []))

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
                    topic_to_publish.config_data_type, lambda conf, _: conf,
                    namespace=topic_to_publish.topic)

    def get_num_subscribers(self):
        sum([p.get_num_connections() for p in self.publishers.values()])

    def get_topic(self):
        return self.base_topic

    def publish(self, raw):
        for transport, transport_info in self.transports.items():
            config = (self.config_servers[transport].config if transport in
                      self.config_servers else None)
            compressed, err = encode(raw, transport_info.name, config)
            if compressed is not None:
                self.publishers[transport].publish(compressed)
            else:
                rospy.logerr(err)

    def shutdown(self):
        for publisher in self.publishers.values():
            publisher.unregister()
