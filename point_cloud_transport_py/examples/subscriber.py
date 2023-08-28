#!/usr/bin/env python3

# Copyright (c) 2023, John D'Angelo
# Copyright (c) 2023, Czech Technical University in Prague
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

from point_cloud_transport._codec import PointCloudCodec, VectorString
from point_cloud_transport.common import stringToMsgType, stringToPointCloud2, TransportInfo

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def _get_loadable_transports(codec: PointCloudCodec):
    transports = VectorString()
    names = VectorString()
    codec.getLoadableTransports(transports, names)
    return dict(zip(transports, names))


def _get_topic_to_subscribe(codec, base_topic, transport_name):
    (topic, name, data_type) = codec.getTopicToSubscribe(
        base_topic, transport_name)

    if len(data_type) == 0:
        return None

    return TransportInfo(name, topic, data_type)


class Subscriber(Node):

    def __init__(self):
        node_name = 'point_cloud_transport_subscriber'
        super().__init__(node_name)

        self.base_topic = '/pct/point_cloud'
        self.transport = self.get_parameter_or('transport', 'draco')
        self.codec = PointCloudCodec()

        transports = _get_loadable_transports(self.codec)
        if self.transport not in transports and self.transport not in transports.values():
            raise RuntimeError(
                'Point cloud transport "%s" not found.' % (self.transport,))

        self.transport_info = _get_topic_to_subscribe(
            self.codec, self.base_topic, self.transport)
        if self.transport_info is None:
            raise RuntimeError(
                'Point cloud transport "%s" not found.' % (self.transport,))

        # subscribe to compressed, serialized msg
        self.subscriber = self.create_subscription(stringToMsgType(
            self.transport_info.data_type), self.transport_info.topic, self.cb, qos_profile_sensor_data)

    def cb(self, cloud):
        print(cloud.height * cloud.width)


if __name__ == '__main__':
    import rclpy
    import sys

    rclpy.init(args=sys.argv)

    subscriber_node = None
    try:
        subscriber_node = Subscriber()
        rclpy.spin(subscriber_node)
    except Exception as e:
        print(e)
    finally:
        if subscriber_node is not None:
            subscriber_node.destroy_node()
        rclpy.shutdown()
