Overview
========

``point_cloud_transport_py`` exposes the C++ ``point_cloud_transport`` API to
Python via pybind11 bindings.  It allows Python nodes to publish and subscribe
to ``sensor_msgs/msg/PointCloud2`` topics using any transport plugin registered
with the C++ ``point_cloud_transport`` package.

Module Structure
----------------

.. code-block:: none

   point_cloud_transport_py/
   ├── __init__.py                 ← re-exports pybind11 classes
   ├── common.py                   ← TransportInfo + serialization helpers
   ├── _point_cloud_transport.*    ← compiled pybind11 extension
   │                                 (PointCloudTransport, Publisher, Subscriber)
   └── _codec.*                    ← compiled pybind11 extension
                                     (PointCloudCodec, VectorString)

The top-level package re-exports the classes defined in the compiled extensions:

- :class:`point_cloud_transport_py.PointCloudTransport`
- :class:`point_cloud_transport_py.Publisher`
- :class:`point_cloud_transport_py.Subscriber`
- :class:`point_cloud_transport_py.PointCloudCodec`

Plus Python-side helpers in :mod:`point_cloud_transport_py.common`:

- :class:`~point_cloud_transport_py.common.TransportInfo`
- :func:`~point_cloud_transport_py.common.stringToPointCloud2`
- :func:`~point_cloud_transport_py.common.pointCloud2ToString`
- :func:`~point_cloud_transport_py.common.stringToMsgType`

Publishing a point cloud
------------------------

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from sensor_msgs.msg import PointCloud2

   from point_cloud_transport_py import PointCloudTransport

   rclpy.init()
   node = Node('my_publisher')
   pct = PointCloudTransport(node)
   pub = pct.advertise('points', 10)

   cloud = PointCloud2()
   # ... populate cloud ...
   pub.publish(cloud)

Subscribing to a point cloud
-----------------------------

.. code-block:: python

   from point_cloud_transport_py import PointCloudTransport

   def on_cloud(msg):
       node.get_logger().info(f'Got {msg.width * msg.height} points')

   sub = pct.subscribe('points', 10, on_cloud)

Transport selection in Python follows the same rules as C++: the explicit
argument wins over the ``point_cloud_transport`` ROS parameter, which wins over
``"raw"``.  Pass the transport name via the optional ``transport_hints``
argument to :meth:`PointCloudTransport.subscribe`.

Using the codec without a node
------------------------------

:class:`~point_cloud_transport_py.PointCloudCodec` exposes the encoder/decoder
interface without requiring a running ROS node — useful for offline tools,
unit tests, and rosbag post-processing.

.. code-block:: python

   from sensor_msgs.msg import PointCloud2

   from point_cloud_transport_py import PointCloudCodec
   from point_cloud_transport_py.common import pointCloud2ToString

   codec = PointCloudCodec()
   raw = PointCloud2()
   # fill the pointcloud
   compressed_buf = codec.encode("draco", pointCloud2ToString(raw))

Serialization Helpers
---------------------

:mod:`point_cloud_transport_py.common` provides utilities that are useful when
shuttling ``PointCloud2`` messages between pybind11 and ``rclpy``:

- ``stringToPointCloud2(buffer)`` — deserialize a raw byte string into a
  ``PointCloud2`` message.
- ``pointCloud2ToString(msg)`` — serialize a ``PointCloud2`` message to bytes.
- ``stringToMsgType("pkg/msg/Foo")`` — dynamically resolve a ROS message type
  from its string identifier.

Dependencies
------------

- **point_cloud_transport** — C++ implementation (provides the plugins)
- **rclpy** / **sensor_msgs** — ROS 2 Python runtime and message types
- **rpyutils** — used for Windows DLL directory handling
- **pybind11** — C++/Python binding layer (build-time only)
