Overview
========

The `point_cloud_transport <https://github.com/ros-perception/point_cloud_transport>`_ an be used to
publish and subscribe to `PointCloud2 <http://docs.ros.org/latest/api/sensor_msgs/html/msg/PointCloud2.html>`_ messages. At this level of usage, it is similar to using ROS 2 Publishers and Subscribers. Using `point_cloud_transport <https://github.com/ros-perception/point_cloud_transport>`_
instead of the ROS 2 primitives, however, gives the user much greater flexibility in how point clouds are communicated between nodes.

Usage
=====

`point_cloud_transport <https://github.com/ros-perception/point_cloud_transport>`_ can be used to publish
and subscribe to `PointCloud2 <http://docs.ros.org/latest/api/sensor_msgs/html/msg/PointCloud2.html>`_
messages. At this level of usage, it is similar to using ROS 2 Publishers and Subscribers.
Using `point_cloud_transport <https://github.com/ros-perception/point_cloud_transport>`_ instead of the
ROS 2 primitives, however, gives the user much greater flexibility in how point clouds are communicated
between nodes.

For complete examples of publishing and subscribing to point clouds using
`point_cloud_transport <https://github.com/ros-perception/point_cloud_transport>`_\ ,
see `Tutorial <https://github.com/ros-perception/point_cloud_transport_tutorial>`_.

Communicating PointCloud2 messages using `point_cloud_transport <https://github.com/ros-perception/point_cloud_transport>`_\ :

.. code-block:: cpp

   #include <rclcpp/rclcpp.hpp>
   #include <point_cloud_transport/point_cloud_transport.hpp>

   void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
   {
     // ... process the message
   }

   auto node = std::make_shared<rclcpp::Node>();
   point_cloud_transport::PointCloudTransport pct(node);
   point_cloud_transport::Subscriber sub = pct.subscribe("in_point_cloud_base_topic", 1, Callback);
   point_cloud_transport::Publisher pub = pct.advertise("out_point_cloud_base_topic", 1);

Alternatively, you can use point_cloud_transport outside of ROS2.

.. code-block:: cpp

   #include <sensor_msgs/msg/point_cloud2.hpp>

   #include <point_cloud_transport/point_cloud_codec.hpp>

   point_cloud_transport::PointCloudCodec codec;

   sensor_msgs::msg::PointCloud2 msg;
   // ... do some cool pointcloud generation stuff ...

   // untyped version (outputs an rclcpp::SerializedMessage)
   rclcpp::SerializedMessage serialized_msg;
   bool success = codec.encode("draco", msg, serialized_msg);

   // OR

   // typed version (outputs whatever message your selected transport returns,
   // for draco that is a point_cloud_interfaces::msg::CompressedPointCloud2)
   point_cloud_interfaces::msg::CompressedPointCloud2 compressed_msg;
   bool success = codec.encode("draco", msg, compressed_msg);

Known transports
----------------

* `draco_point_cloud_transport <https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/draco_point_cloud_transport>`_\ : Lossy compression via Google
* `zlib_point_cloud_transport <https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/zlib_point_cloud_transport>`_\ : Lossless compression via Zlib compression.
* `zstd_point_cloud_transport <https://github.com/ros-perception/point_cloud_transport_plugins/tree/rolling/zstd_point_cloud_transport>`_\ : Lossless compression via Zstd compression.
* Did you write one? Don't hesitate and send a pull request adding it to this list!

Support
=======

For questions/comments please email the maintainer mentioned in ``package.xml``.

If you have found an error in this package, please file an issue at: `https://github.com/ros-perception/point_cloud_transport/issues <https://github.com/ros-perception/point_cloud_transport/issues>`_

Patches are encouraged, and may be submitted by forking this project and
submitting a pull request through GitHub. Any help is further development of the project is much appreciated.


.. toctree::
   :maxdepth: 2

   self

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
