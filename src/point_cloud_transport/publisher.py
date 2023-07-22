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

from rclpy import Node

from .common import _TransportInfo
from .encoder import encode


class Publisher(Node):

    def __init__(self):
        self.transports = _get_topics_to_publish("point_cloud", self.get_logger())

        blacklist = set(self.get_parameter_or('disable_pub_plugins', []))

        self.publishers = {}
        self.config_servers = {}
        for transport in self.transports:
            if transport in blacklist:
                continue
            topic_to_publish = self.transports[transport]
            self.publishers[transport] = self.create_publisher(topic_to_publish.topic, topic_to_publish.data_type)

    def publish(self, raw):
        for transport, transport_info in self.transports.items():
            config = (self.config_servers[transport].config if transport in
                      self.config_servers else None)
            compressed, err = encode(raw, transport_info.name, config)
            if compressed is not None:
                self.publishers[transport].publish(compressed)
            else:
                self.get_logger().error('Error encoding message: ' + err)


if __name__ == "__main__":
    import rclpy

    try:
        publisher_node = Publisher()
        rclpy.spin(publisher_node)
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()
