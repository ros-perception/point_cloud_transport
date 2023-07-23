#!/usr/bin/env python3

# Copyright (c) 2023, John D'Angelo
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

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from point_cloud_transport.common import TransportInfo, pointCloud2ToString, stringToMsgType
from point_cloud_transport._codec import PointCloudCodec, VectorString

def _get_topics_to_publish(codec, base_topic, logger):
    transports = VectorString()
    names = VectorString()
    topics = VectorString()
    data_types = VectorString()

    codec.getTopicsToPublish(
        base_topic, transports,
        topics, names, data_types)

    topics_to_publish = {}

    for i in range(len(transports)):
        try:
            topics_to_publish[transports[i]] = \
                TransportInfo(names[i], topics[i], data_types[i])
        except ImportError as e:
            print('Import error: ' + str(e))

    return topics_to_publish

class Publisher(Node):

    def __init__(self):
        node_name = "point_cloud_transport_publisher"
        super().__init__(node_name)

        self.codec = PointCloudCodec()
        print("Codec created")
        self.topics_to_publish = _get_topics_to_publish(self.codec, "point_cloud", self.get_logger())
        print("Topics to publish: \n", self.topics_to_publish)

        blacklist = set(self.get_parameter_or('disable_pub_plugins', []))
        print("Blacklist: \n" + str(blacklist))

        self.transport_publishers = {}
        for transport in self.topics_to_publish:
            if transport in blacklist:
                continue
            topic_to_publish = self.topics_to_publish[transport]
            self.transport_publishers[transport] = self.create_publisher(stringToMsgType(topic_to_publish.data_type), topic_to_publish.topic, 1)

    def publish(self, raw : PointCloud2):
        for transport, transport_info in self.topics_to_publish.items():
            compressed_buffer = self.codec.encode(transport_info.name, pointCloud2ToString(raw))            
            if compressed_buffer:
                # rclpy is smart and will publish the correct type even though we are giving it a serialized array
                # of bytes
                self.transport_publishers[transport].publish(compressed_buffer)
            else:
                self.get_logger().error('Error encoding message!')


if __name__ == "__main__":
    import rclpy
    import sys

    rclpy.init(args=sys.argv)

    publisher_node = None
    try:
        publisher_node = Publisher()
        rclpy.spin(publisher_node)
    except Exception as e:
        print("Error in publisher node!")
        print(e)
    finally:
        if publisher_node is not None:
            publisher_node.destroy_node()
        rclpy.shutdown()
