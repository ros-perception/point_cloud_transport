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

"""Shared helpers and fixtures for the point_cloud_transport_py tests."""

import struct

from point_cloud_transport_py import PointCloudCodec, VectorString
import pytest
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField

# Point layout of the test clouds, padded with one byte to keep the point step aligned.
POINT_FORMAT = '<ffffBBBB'
POINT_FIELDS = (
    ('x', 0),
    ('y', 4),
    ('z', 8),
    ('intensity', 12),
    ('r', 16),
    ('g', 17),
    ('b', 18),
)


def make_point_cloud(num_points=8, frame_id='base_link'):
    """Return a deterministic point cloud with x, y, z, intensity and rgb fields."""
    point_step = struct.calcsize(POINT_FORMAT)
    cloud = PointCloud2()
    cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = num_points
    cloud.fields = [
        PointField(
            name=name,
            offset=offset,
            datatype=PointField.FLOAT32 if offset < 16 else PointField.UINT8,
            count=1,
        )
        for name, offset in POINT_FIELDS
    ]
    cloud.is_dense = True
    cloud.point_step = point_step
    cloud.row_step = point_step * num_points
    cloud.data = bytearray(b''.join(
        struct.pack(
            POINT_FORMAT,
            float(i), 2.0 * i, 3.0 * i, 0.5 * i,
            i % 256, (2 * i) % 256, (3 * i) % 256,
            0,
        )
        for i in range(num_points)
    ))
    return cloud


def load_transports(codec):
    """Return the (fully qualified names, short names) of the loadable transports."""
    transports = VectorString()
    names = VectorString()
    codec.getLoadableTransports(transports, names)
    return list(transports), list(names)


@pytest.fixture
def codec():
    """Return a freshly created codec, so each test gets its own plugin loaders."""
    return PointCloudCodec()


@pytest.fixture
def make_cloud():
    """Return the point cloud factory, for tests that need clouds of their own size."""
    return make_point_cloud


@pytest.fixture
def transports(codec):
    """Return the (fully qualified names, short names) of the transports available here."""
    return load_transports(codec)


@pytest.fixture
def point_cloud(make_cloud):
    """Return a small deterministic PointCloud2 message."""
    return make_cloud()


@pytest.fixture
def serialized_point_cloud(point_cloud):
    """Return the CDR serialized form of the point cloud fixture."""
    return serialize_message(point_cloud)
