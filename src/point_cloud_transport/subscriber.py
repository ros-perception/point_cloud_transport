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

"""Subscriber automatically converting from any transport to raw."""

from rclpy import Node

from .common import _TransportInfo, stringToPointCloud2
from point_cloud_transport import PointCloudCodec

def _get_loadable_transports(pct):
    transports = []
    names = []
    pct.getLoadableTransports(transports, names)
    return dict(zip(transports, names))

def _get_topic_to_subscribe(pct, base_topic, transport_name, logger):
    (topic, name, data_type) = pct.getTopicToSubscribe(base_topic, transport_name)

    if len(data_type) == 0:
        return None

    return _TransportInfo(name.value, topic, data_type)

class Subscriber(Node):
    def __init__(self):
        self.base_topic = "point_cloud"
        self.transport = self.get_parameter_or("transport", "raw")
        self.pct = PointCloudCodec()

        transports = _get_loadable_transports(self.pct)
        if self.transport not in transports and self.transport not in transports.values():
            raise RuntimeError("Point cloud transport '%s' not found." % (self.transport,))

        self.transport_info = _get_topic_to_subscribe(self.pct, self.base_topic, self.transport, self.get_logger())
        if self.transport_info is None:
            raise RuntimeError("Point cloud transport '%s' not found." % (self.transport,))

        # subscribe to compressed, serialized msg
        self.subscriber = self.create_subscription(self.transport_info.topic, self.transport_info.data_type, self.cb, raw=True)

    def cb(self, serialized_buffer):
        cloud_buffer = self.pct.decode(self.transport_info.name, serialized_buffer)
        cloud = stringToPointCloud2(cloud_buffer)

if __name__ == "__main__":
    import rclpy

    try:
        subscriber_node = Subscriber()
        rclpy.spin(subscriber_node)
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()