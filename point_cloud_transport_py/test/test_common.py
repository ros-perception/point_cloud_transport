# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for the helpers in point_cloud_transport_py.common."""

from point_cloud_transport_py.common import pointCloud2ToString, stringToMsgType
from point_cloud_transport_py.common import stringToPointCloud2, TransportInfo
import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField


def test_point_cloud2_to_string_matches_rclpy(point_cloud: PointCloud2):
    """Serializing with the helper gives the same bytes as serializing with rclpy."""
    serialized: bytes = pointCloud2ToString(point_cloud)
    assert isinstance(serialized, bytes)
    assert serialized == serialize_message(point_cloud)


def test_string_to_point_cloud2_round_trip(point_cloud: PointCloud2):
    """A serialized point cloud is turned back into an equal message."""
    deserialized: PointCloud2 = stringToPointCloud2(pointCloud2ToString(point_cloud))
    assert isinstance(deserialized, PointCloud2)
    assert deserialized == point_cloud


def test_string_to_point_cloud2_keeps_the_points(point_cloud: PointCloud2):
    """The raw point data survives the round trip unchanged."""
    deserialized: PointCloud2 = stringToPointCloud2(pointCloud2ToString(point_cloud))
    assert bytes(deserialized.data) == bytes(point_cloud.data)
    assert deserialized.point_step == point_cloud.point_step
    assert deserialized.width * deserialized.height * deserialized.point_step == \
        len(deserialized.data)


def test_empty_point_cloud_round_trip():
    """A message without points round trips into an equal message."""
    empty = PointCloud2()
    assert stringToPointCloud2(pointCloud2ToString(empty)) == empty


def test_string_to_point_cloud2_rejects_garbage():
    """Bytes that are not a serialized point cloud are reported as an error."""
    with pytest.raises(Exception):
        stringToPointCloud2(b'not a serialized point cloud at all !!')


@pytest.mark.parametrize('message_type_str', [
    'sensor_msgs/msg/PointCloud2',
    'sensor_msgs.msg.PointCloud2',
])
def test_string_to_msg_type(message_type_str: str):
    """Message types are resolved from both the ROS and the Python spelling."""
    assert stringToMsgType(message_type_str) is PointCloud2


def test_string_to_msg_type_other_type():
    """Types of the same package other than the point cloud are resolved too."""
    assert stringToMsgType('sensor_msgs/msg/PointField') is PointField


def test_string_to_msg_type_unknown_package(capsys):
    """An unresolvable type gives None and a message on stdout instead of raising."""
    assert stringToMsgType('some_package_that_does_not_exist/msg/PointCloud2') is None
    assert 'some_package_that_does_not_exist' in capsys.readouterr().out


def test_string_to_msg_type_unknown_type(capsys):
    """A package without the requested type gives None and a message on stdout."""
    assert stringToMsgType('sensor_msgs/msg/NotAMessageType') is None
    assert 'NotAMessageType' in capsys.readouterr().out


@pytest.mark.parametrize('message_type_str', ['', 'PointCloud2', 'sensor_msgs/msg/'])
def test_string_to_msg_type_malformed(message_type_str, capsys):
    """Malformed type strings are reported as None rather than raising."""
    assert stringToMsgType(message_type_str) is None
    capsys.readouterr()


def test_transport_info(codec):
    """A TransportInfo can be built from the transport description of the codec."""
    topic, name, data_type = codec.getTopicToSubscribe('points', 'raw')
    transport_info = TransportInfo(name=name, topic=topic, data_type=data_type)
    assert transport_info.name == 'raw'
    assert transport_info.topic == 'points'
    assert stringToMsgType(transport_info.data_type) is PointCloud2


def test_transport_info_compares_by_value():
    """Two transport descriptions with the same content are equal."""
    first = TransportInfo(name='raw', topic='points', data_type='sensor_msgs/msg/PointCloud2')
    second = TransportInfo(name='raw', topic='points', data_type='sensor_msgs/msg/PointCloud2')
    assert first == second
    assert first != TransportInfo(name='raw', topic='other', data_type=first.data_type)
