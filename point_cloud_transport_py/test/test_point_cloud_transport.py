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

"""Unit tests for the point_cloud_transport and Publisher bindings."""

import time

from point_cloud_transport_py import PointCloudTransport, Publisher, Subscriber
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2

# Time to wait for RMW to match endpoints and deliver, in seconds.
MATCH_TIMEOUT = 5.0
# Pause between two repeated publications while waiting for a message to arrive.
PUBLISH_INTERVAL = 0.005


@pytest.fixture
def ros_context():
    context = rclpy.context.Context()
    rclpy.init(context=context)
    yield context
    rclpy.shutdown(context=context)


@pytest.fixture
def listener(ros_context):
    node = rclpy.create_node('test_listener', namespace='', context=ros_context)
    yield node
    node.destroy_node()


@pytest.fixture
def executor(ros_context, listener):
    executor = SingleThreadedExecutor(context=ros_context)
    executor.add_node(listener)
    yield executor
    executor.remove_node(listener)
    executor.shutdown()


@pytest.fixture
def transport(ros_context):
    transport = PointCloudTransport('test_transport', '')
    yield transport
    del transport


def wait_until(predicate, timeout=MATCH_TIMEOUT):
    """Wait until predicate() holds and report whether that happened in time."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def wait_until_matched(publisher):
    """Wait until the middleware connected a subscriber to the given publisher."""
    if not wait_until(lambda: publisher.get_num_subscribers() > 0):
        pytest.fail(f'no subscriber was matched on {publisher.get_topic()}')


def publish_until(publisher, payload, received, executor, count=1):
    """Publish repeatedly until at least count messages arrived, and say whether they did."""
    deadline = time.monotonic() + MATCH_TIMEOUT
    while len(received) < count and time.monotonic() < deadline:
        publisher.publish(payload)
        executor.spin_once(timeout_sec=0.01)
        time.sleep(PUBLISH_INTERVAL)
    return len(received) >= count


def advertised_topics(node):
    """Return the topics visible to the given node, mapped to their message types."""
    return dict(node.get_topic_names_and_types())


def subscribe(listener, topic, callback):
    """Subscribe the listener node to a topic with the sensor data QoS profile."""
    return listener.create_subscription(PointCloud2, topic, callback, qos_profile_sensor_data)


def test_default_publisher():
    """A default constructed publisher is invalid but safe to query."""
    publisher = Publisher()
    assert publisher.get_topic() == ''
    assert publisher.get_num_subscribers() == 0
    publisher.shutdown()
    assert publisher.get_topic() == ''
    assert publisher.get_num_subscribers() == 0


def test_default_subscriber():
    """A default constructed subscriber is invalid but safe to query."""
    subscriber = Subscriber()
    assert subscriber.get_topic() == ''
    assert subscriber.get_transport() == ''
    assert subscriber.get_num_publishers() == 0
    subscriber.shutdown()


@pytest.mark.parametrize(('base_topic', 'expected_topic'), [
    ('points', '/points'),
    ('/sensors/points', '/sensors/points'),
    ('sensors/points', '/sensors/points'),
])
def test_advertise_resolves_the_topic(transport, base_topic, expected_topic):
    """The advertised topic is fully resolved and reported back."""
    publisher = transport.advertise(base_topic, 10)
    assert publisher.get_topic() == expected_topic
    publisher.shutdown()


def test_shutdown_resets_the_publisher(transport, listener):
    """Shutting the publisher down leaves it as empty as a default constructed one."""
    base_topic = '/test_transport/shutdown'
    publisher = transport.advertise(base_topic, 10)
    if not wait_until(lambda: base_topic in advertised_topics(listener)):
        publisher.shutdown()
        pytest.fail(f'{base_topic} did not show up in the graph')
    publisher.shutdown()
    assert publisher.get_topic() == ''
    assert publisher.get_num_subscribers() == 0
    publisher.shutdown()


def test_params_file_configures_the_node(ros_context, tmp_path):
    """A parameter file is accepted."""
    params_file = tmp_path / 'params.yaml'
    params_file.write_text(
        'test_params_transport:\n'
        '  ros__parameters:\n'
        '    test_transport.params.enable_pub_plugins: [""]\n'
    )
    configured = PointCloudTransport('test_params_transport', str(params_file))
    with pytest.raises(RuntimeError):
        # creating a publisher with no plugins should raise an exception
        publisher = configured.advertise('/test_transport/params', 10)
        # in case the publisher creation did not throw
        publisher.shutdown()


def test_missing_params_file_is_reported(ros_context):
    """A parameter file that does not exist is reported while constructing the node."""
    with pytest.raises(RuntimeError):
        PointCloudTransport('test_missing_params_transport', '/does/not/exist.yaml')


def test_advertise_shows_the_topic_in_the_graph(transport, listener):
    """The advertised base topic appears in the graph with the point cloud type."""
    base_topic = '/test_transport/graph'
    publisher = transport.advertise(base_topic, 10)
    assert publisher.get_topic() == base_topic
    if not wait_until(lambda: base_topic in advertised_topics(listener)):
        publisher.shutdown()
        pytest.fail(f'{base_topic} did not show up in the graph')
    assert advertised_topics(listener)[base_topic] == ['sensor_msgs/msg/PointCloud2']
    publisher.shutdown()


def test_publish_delivers_the_point_cloud(transport, listener, executor, point_cloud):
    """A serialized point cloud published by the bindings arrives at a subscription."""
    base_topic = '/test_transport/raw_points'
    received = []
    with subscribe(listener, base_topic, received.append):
        publisher = transport.advertise(base_topic, 10)
        wait_until_matched(publisher)

        delivered = publish_until(publisher, serialize_message(point_cloud), received, executor)
        publisher.shutdown()
        if not delivered:
            pytest.fail(f'no point cloud was delivered on {base_topic}')
        assert received[0] == point_cloud


def test_publish_reaches_every_subscriber(transport, listener, executor, serialized_point_cloud):
    """Every subscriber connected to the base topic is served."""
    base_topic = '/test_transport/two_subscribers'
    received = {'first': [], 'second': []}
    with subscribe(listener, base_topic, received['first'].append):
        with subscribe(listener, base_topic, received['second'].append):
            publisher = transport.advertise(base_topic, 10)
            if not wait_until(lambda: publisher.get_num_subscribers() == 2):
                publisher.shutdown()
                pytest.fail('both subscribers were not matched on the base topic')

            deadline = time.monotonic() + MATCH_TIMEOUT
            while not all(received.values()) and time.monotonic() < deadline:
                publisher.publish(serialized_point_cloud)
                executor.spin_once(timeout_sec=0.01)
                time.sleep(PUBLISH_INTERVAL)
            publisher.shutdown()

            if not all(received.values()):
                pytest.fail(f'the subscriptions on {base_topic} were not both served')
            assert received['first'][0] == received['second'][0]
            assert publisher.get_num_subscribers() == 0


@pytest.mark.parametrize('message', [
    'not bytes',
    PointCloud2(),
    b'',
    b'not a serialized point cloud at all !!',
])
def test_publish_requires_a_serialized_message(transport, message):
    """Only serialized point clouds can be published, and errors are reported."""
    publisher = transport.advertise('/test_transport/invalid_payloads', 10)
    try:
        with pytest.raises((TypeError, RuntimeError)):
            publisher.publish(message)
    finally:
        publisher.shutdown()


def test_publish_many_point_clouds(transport, listener, executor, make_cloud):
    """The publisher keeps working over many consecutive publications."""
    base_topic = '/test_transport/many_points'
    received = []
    seen_frames = set()
    with subscribe(listener, base_topic, received.append):
        publisher = transport.advertise(base_topic, 10)
        try:
            wait_until_matched(publisher)

            clouds = [
                make_cloud(num_points=num_points, frame_id=f'frame_{num_points}')
                for num_points in (1, 8, 64)
            ]
            payloads = [serialize_message(cloud) for cloud in clouds]
            expected_frames = {cloud.header.frame_id for cloud in clouds}
            published = 0
            deadline = time.monotonic() + MATCH_TIMEOUT
            while seen_frames != expected_frames and time.monotonic() < deadline:
                publisher.publish(payloads[published % len(payloads)])
                published += 1
                executor.spin_once(timeout_sec=0.01)
                time.sleep(PUBLISH_INTERVAL)
                seen_frames = {message.header.frame_id for message in received}
        finally:
            publisher.shutdown()

        if seen_frames != expected_frames:
            pytest.fail(f'not every point cloud was delivered on {base_topic}')
        for message in (m for m in received if m.header.frame_id == 'frame_64'):
            assert message.width == 64
            assert message.row_step == message.width * message.point_step
