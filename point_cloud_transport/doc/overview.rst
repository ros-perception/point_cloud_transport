Overview
========

``point_cloud_transport`` provides a ROS 2 framework for publishing and subscribing to
``sensor_msgs/msg/PointCloud2`` messages using interchangeable transport plugins.  It
mirrors the design of ``image_transport`` but targets 3-D point cloud data.

Module Structure
----------------

.. code-block:: none

   point_cloud_transport/
   ├── include/point_cloud_transport/
   │   ├── point_cloud_transport.hpp   ← PointCloudTransport + free functions
   │   ├── publisher.hpp               ← Publisher handle
   │   ├── subscriber.hpp              ← Subscriber handle
   │   ├── publisher_plugin.hpp        ← Base class for publisher plugins
   │   ├── subscriber_plugin.hpp       ← Base class for subscriber plugins
   │   ├── simple_publisher_plugin.hpp ← Template base for simple pub plugins
   │   ├── simple_subscriber_plugin.hpp← Template base for simple sub plugins
   │   ├── point_cloud_codec.hpp       ← Node-free encode/decode interface
   │   ├── transport_hints.hpp         ← Transport selection helpers
   │   ├── subscriber_filter.hpp       ← message_filters integration
   │   └── ...
   └── src/

Transport System
----------------

When a :cpp:class:`~point_cloud_transport::Publisher` is advertised on a *base topic*
(e.g. ``/lidar/points``), ``point_cloud_transport`` automatically advertises one
sub-topic per available transport plugin:

.. code-block:: none

   /lidar/points           ← raw (uncompressed) PointCloud2
   /lidar/points/draco     ← Draco-compressed (if plugin loaded)
   /lidar/points/zlib      ← zlib-compressed (if plugin loaded)

A :cpp:class:`~point_cloud_transport::Subscriber` subscribes to the transport-specific
sub-topic that matches the requested transport, but delivers standard
``sensor_msgs/msg/PointCloud2`` messages to the application callback.

Creating Publishers and Subscribers
------------------------------------

The preferred API uses ``rclcpp::node_interfaces::NodeInterfaces``:

.. code-block:: cpp

   #include <point_cloud_transport/point_cloud_transport.hpp>

   auto node = std::make_shared<rclcpp::Node>("my_node");
   point_cloud_transport::PointCloudTransport pct(node);

   // Publish
   auto pub = pct.advertise("points", rclcpp::SensorDataQoS());

   // Subscribe (transport selected via ROS parameter or TransportHints)
   auto sub = pct.subscribe(
     "points", rclcpp::SensorDataQoS(),
     [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
       // process msg
     });

Free-function equivalents exist for cases where a
:cpp:class:`~point_cloud_transport::PointCloudTransport` object is inconvenient:

.. code-block:: cpp

   auto pub = point_cloud_transport::create_publisher(node, "points", rclcpp::SensorDataQoS());
   auto sub = point_cloud_transport::create_subscription(
     node, "points", callback, "raw", rclcpp::SensorDataQoS());

Transport Selection
-------------------

The active transport is chosen at subscription time.  Priority order:

1. Explicit ``transport`` argument to ``subscribe()`` / ``create_subscription()``.
2. ROS parameter ``<node_name>.point_cloud_transport`` (set via ``TransportHints``).
3. Default: ``"raw"`` (uncompressed).

.. code-block:: cpp

   // Force Draco transport
   point_cloud_transport::TransportHints hints(node, "draco");
   auto sub = pct.subscribe("points", qos, callback, {}, &hints);

Plugin Development
------------------

Transport plugins are pluginlib plugins that derive from
:cpp:class:`~point_cloud_transport::PublisherPlugin` and
:cpp:class:`~point_cloud_transport::SubscriberPlugin`.

For the common case of a single transport-specific message type, derive from the
simpler template bases:

.. code-block:: cpp

   #include <point_cloud_transport/simple_publisher_plugin.hpp>

   class MyPublisher
     : public point_cloud_transport::SimplePublisherPlugin<my_msgs::msg::Compressed>
   {
   public:
     std::string getTransportName() const override { return "my_transport"; }

   protected:
     EncodeResult encodeTyped(
       const sensor_msgs::msg::PointCloud2 & raw,
       my_msgs::msg::Compressed & compressed) override
     {
       // compress raw → compressed
       return true;
     }
   };

Register the plugin with pluginlib and declare it in ``default_plugins.xml``.

Node-free Codec
---------------

:cpp:class:`~point_cloud_transport::PointCloudCodec` provides encode/decode without
a running ROS node — useful for offline tools and tests:

.. code-block:: cpp

   #include <point_cloud_transport/point_cloud_codec.hpp>

   point_cloud_transport::PointCloudCodec codec;
   auto encoder = codec.getEncoderByName("draco");
   rclcpp::SerializedMessage serialized;
   encoder->encode(raw_cloud, serialized);

message_filters Integration
-----------------------------

:cpp:class:`~point_cloud_transport::SubscriberFilter` wraps a
:cpp:class:`~point_cloud_transport::Subscriber` for use in
``message_filters`` synchronisation chains:

.. code-block:: cpp

   #include <point_cloud_transport/subscriber_filter.hpp>
   #include <message_filters/synchronizer.h>

   point_cloud_transport::SubscriberFilter sub_a(pct, "points_a", qos);
   point_cloud_transport::SubscriberFilter sub_b(pct, "points_b", qos);
   message_filters::Synchronizer<MySyncPolicy> sync(policy, sub_a, sub_b);

Dependencies
------------

- **rclcpp** — node interfaces and QoS
- **sensor_msgs** — ``PointCloud2`` message type
- **pluginlib** — plugin loading
- **message_filters** — time-synchronised subscription chains
- **rcpputils** — asserts and type utilities
