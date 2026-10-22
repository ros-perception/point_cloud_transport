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

"""Unit tests for the point_cloud_transport::PointCloudCodec bindings."""

from point_cloud_transport_py import VectorString
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2

RAW = 'raw'
RAW_TRANSPORT = 'point_cloud_transport/raw'
POINT_CLOUD_DATA_TYPE = 'sensor_msgs/msg/PointCloud2'


def field_specs(message):
    """Return the (name, offset, datatype, count) tuple of every field of a point cloud."""
    return [(field.name, field.offset, field.datatype, field.count) for field in message.fields]


def test_raw_transport_is_loadable(transports):
    """The raw transport is part of point_cloud_transport and must always be loadable."""
    fully_qualified, names = transports
    assert RAW in names
    assert RAW_TRANSPORT in fully_qualified
    assert len(fully_qualified) == len(names)
    assert all(
        transport.endswith('/' + name) for transport, name in zip(fully_qualified, names))


def test_loadable_transports_are_reported_with_plain_strings(codec, transports):
    """The lists handed to the binding are opaque vectors of plain strings."""
    assert all(isinstance(transport, str) for transport in transports[0] + transports[1])
    loaded_transports, loaded_names = VectorString(), VectorString()
    codec.getLoadableTransports(loaded_transports, loaded_names)
    assert (list(loaded_transports), list(loaded_names)) == transports


def test_get_topic_to_subscribe_raw(codec):
    """The raw transport is served on the base topic with the plain point cloud type."""
    topic, name, data_type = codec.getTopicToSubscribe('points', RAW)
    assert (topic, name, data_type) == ('points', RAW, POINT_CLOUD_DATA_TYPE)


@pytest.mark.parametrize('transport', [RAW, RAW_TRANSPORT])
def test_get_topic_to_subscribe_accepts_both_name_forms(codec, transport):
    """Transports are looked up by short name as well as by fully qualified name."""
    assert codec.getTopicToSubscribe('points', transport) == \
        ('points', RAW, POINT_CLOUD_DATA_TYPE)


def test_get_topic_to_subscribe_keeps_the_base_topic(codec):
    """The base topic is used verbatim, the node namespace is not added by the codec."""
    for base_topic in ('points', '/points', 'sensors/points', '/sensors/lidar/points'):
        assert codec.getTopicToSubscribe(base_topic, RAW)[0] == base_topic


def test_get_topic_to_subscribe_unknown_transport(codec):
    """An unavailable transport is reported as an empty result rather than an error."""
    assert codec.getTopicToSubscribe('points', 'definitely_not_a_transport') == ('', '', '')


def test_get_topics_to_publish(codec, transports):
    """Every loadable transport gets a topic, a name and a message type."""
    topics, names, data_types = VectorString(), VectorString(), VectorString()
    result = codec.getTopicsToPublish(
        'points', VectorString(transports[0]), topics, names, data_types)
    assert result is None
    assert len(topics) == len(names) == len(data_types)
    assert RAW in names
    for topic, name, data_type in zip(topics, names, data_types):
        expected_topic = 'points' if name == RAW else f'points/{name}'
        assert topic == expected_topic
        assert data_type.count('/') == 2
        if name == RAW:
            assert data_type == POINT_CLOUD_DATA_TYPE


def test_get_topics_to_publish_reports_raw(codec, transports):
    """Looking up all transports agrees with looking up the raw transport."""
    topics, names, data_types = VectorString(), VectorString(), VectorString()
    codec.getTopicsToPublish(
        'points', VectorString(transports[0]), topics, names, data_types)
    index = list(names).index(RAW)
    assert codec.getTopicToSubscribe('points', RAW) == \
        (topics[index], names[index], data_types[index])


def test_get_topics_to_publish_is_not_restricted_by_the_request(codec):
    """The raw transport is reported even when something else was requested."""
    topics, names, data_types = VectorString(), VectorString(), VectorString()
    codec.getTopicsToPublish(
        'points', VectorString(['definitely_not_a_transport']), topics, names, data_types)
    assert RAW in names


@pytest.mark.parametrize('transport', [RAW, RAW_TRANSPORT])
def test_lossless_round_trip(codec, point_cloud, serialized_point_cloud, transport):
    """Encoding and decoding with the raw transport restores the message exactly."""
    encoded = codec.encode(transport, serialized_point_cloud)
    assert isinstance(encoded, bytes)
    assert encoded
    decoded = codec.decode(transport, encoded)
    assert deserialize_message(decoded, PointCloud2) == point_cloud


def test_encoded_raw_payload_is_a_serialized_point_cloud(
    codec, point_cloud, serialized_point_cloud,
):
    """The raw transport transfers plain point clouds, so its payload stays a PointCloud2."""
    encoded = codec.encode(RAW, serialized_point_cloud)
    assert deserialize_message(encoded, PointCloud2) == point_cloud


def test_round_trip_keeps_the_points_and_their_layout(codec, point_cloud, serialized_point_cloud):
    """The point data and the description of the fields are transferred unchanged."""
    decoded = codec.decode(RAW, codec.encode(RAW, serialized_point_cloud))
    message = deserialize_message(decoded, PointCloud2)
    assert bytes(message.data) == bytes(point_cloud.data)
    assert field_specs(message) == field_specs(point_cloud)
    assert (message.height, message.width) == (point_cloud.height, point_cloud.width)
    assert message.point_step == point_cloud.point_step
    assert message.row_step == point_cloud.row_step
    assert message.is_dense == point_cloud.is_dense


def test_empty_cloud_round_trip(codec, serialized_point_cloud):
    """A point cloud without points survives the round trip."""
    empty = PointCloud2()
    payload = serialize_message(empty)
    decoded = codec.decode(RAW, codec.encode(RAW, payload))
    assert deserialize_message(decoded, PointCloud2) == empty


def test_repeated_round_trips_are_independent(codec, point_cloud, serialized_point_cloud):
    """The codec keeps no state between calls."""
    for _ in range(3):
        decoded = codec.decode(RAW, codec.encode(RAW, serialized_point_cloud))
        assert deserialize_message(decoded, PointCloud2) == point_cloud


def test_round_trip_with_many_points(codec, make_cloud):
    """A larger cloud is transferred without truncation."""
    cloud = make_cloud(num_points=500000)
    payload = serialize_message(cloud)
    decoded = codec.decode(RAW, codec.encode(RAW, payload))
    message = deserialize_message(decoded, PointCloud2)
    assert message.width == 500000
    assert bytes(message.data) == bytes(cloud.data)


def test_encode_unknown_transport(codec, serialized_point_cloud):
    """Encoding with a transport that cannot be loaded returns empty bytes."""
    assert codec.encode('definitely_not_a_transport', serialized_point_cloud) == b''


def test_decode_unknown_transport(codec, serialized_point_cloud):
    """Decoding with a transport that cannot be loaded returns empty bytes."""
    assert codec.decode('definitely_not_a_transport', serialized_point_cloud) == b''


def test_decode_empty_payload(codec):
    """Decoding an empty payload returns empty bytes."""
    assert codec.decode(RAW, b'') == b''


def test_encode_empty_payload(codec):
    """Encoding an empty payload is reported instead of silently succeeding."""
    with pytest.raises(RuntimeError):
        codec.encode(RAW, b'')


def test_decode_truncated_payload(codec, serialized_point_cloud):
    """Decoding a payload that is cut short does not yield a point cloud."""
    assert codec.decode(RAW, serialized_point_cloud[:len(serialized_point_cloud) // 2]) == b''


def test_encode_truncated_payload(codec, serialized_point_cloud):
    """Encoding a truncated payload raises, since it cannot be deserialized."""
    with pytest.raises(RuntimeError):
        codec.encode(RAW, serialized_point_cloud[:4])


def test_decode_malformed_payload(codec):
    """Decoding bytes that are not a serialized message returns empty bytes."""
    assert codec.decode(RAW, b'not a serialized point cloud at all !!') == b''


@pytest.mark.parametrize('payload', ['a string', None, 42, PointCloud2()])
def test_encode_rejects_non_buffer_arguments(codec, payload):
    """Only objects supporting the buffer protocol are accepted."""
    with pytest.raises(TypeError):
        codec.encode(RAW, payload)


@pytest.mark.parametrize('payload', ['a string', None, PointCloud2()])
def test_decode_rejects_non_buffer_arguments(codec, payload):
    """Only objects supporting the buffer protocol are accepted."""
    with pytest.raises(TypeError):
        codec.decode(RAW, payload)


@pytest.mark.parametrize('transport', [RAW, RAW_TRANSPORT])
def test_encode_accepts_any_buffer(codec, serialized_point_cloud, transport):
    """Bytes, bytearrays and memoryviews are all accepted as input."""
    from_bytes = codec.encode(transport, serialized_point_cloud)
    assert codec.encode(transport, bytearray(serialized_point_cloud)) == from_bytes
    assert codec.encode(transport, memoryview(serialized_point_cloud)) == from_bytes
