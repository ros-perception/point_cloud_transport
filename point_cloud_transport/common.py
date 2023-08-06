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

"""Common definitions."""

from importlib import import_module

from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2


class TransportInfo(object):

    def __init__(self, name: str, topic: str, data_type: str):
        self.name = name
        self.topic = topic
        self.data_type = data_type


def stringToPointCloud2(buffer: str):
    cloud = deserialize_message(buffer, 'sensor_msgs/msg/PointCloud2')
    return cloud


def pointCloud2ToString(msg: PointCloud2):
    buffer = serialize_message(msg)
    return buffer


def stringToMsgType(message_type_str):
    try:
        # Dynamically import the message type
        package_name, message_type = message_type_str.replace(
            '/', '.').rsplit('.', 1)
        module = import_module(package_name)
        message_class = getattr(module, message_type)
        # Return the subscription object
        return message_class
    except Exception as e:
        print(f'Error creating subscription: {e}')
        return None
