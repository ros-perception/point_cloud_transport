Overview
========

``point_cloud_transport`` provides a ROS 2 framework for publishing and subscribing to
``sensor_msgs/msg/PointCloud2`` messages using interchangeable transport plugins.  It
mirrors the design of ``image_transport`` but targets ``PointCloud2`` data.

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
(e.g. ``/lidar/points``), ``point_cloud_transport`` automatically advertises
sub-topics for all available transport plugins:

.. code-block:: none

   /lidar/points           ← raw (uncompressed) PointCloud2
   /lidar/points/draco     ← Draco-compressed (if plugin available)
   /lidar/points/zlib      ← zlib-compressed (if plugin available)

No data conversion or compression is done by any publisher plugin until a subscriber actually requests the data.

A :cpp:class:`~point_cloud_transport::Subscriber` subscribes to the transport-specific
sub-topic that matches the requested transport, but delivers standard
``sensor_msgs/msg/PointCloud2`` messages to the application callback.

Simple and General Transports
-----------------------------
A general point cloud transport can use any number of
topics and send any kind of data over them to manage
the transmission of encoded messages. It is not even
required that a single ``PointCloud2`` message has to
result into publishing a single message (e.g. video codecs
might generate no output packet for a frame under certain
circumstances). The API of ``point_cloud_transport`` is designed
to accommodate any arrangement.
However, most transports satisfy the conditions of simple
transport:
- There is exactly one topic used by the transport.
- Each raw message results in exactly one encoded message.
Simple transports are much easier to implement using the template
classes :cpp:class:`~point_cloud_transport::SimpleSubscriberPlugin`
and :cpp:class:`~point_cloud_transport::SimplePublisherPlugin` .

Creating Publishers and Subscribers
------------------------------------

.. code-block:: cpp

   #include <point_cloud_transport/point_cloud_transport.hpp>

   auto node = std::make_shared<rclcpp::Node>("my_node");
   point_cloud_transport::PointCloudTransport pct(*node);

   // Publish
   auto pub = pct.advertise("points", rclcpp::SystemDefaultsQoS());

   // Subscribe (transport selected via ROS parameter or TransportHints)
   auto sub = pct.subscribe(
     "points", rclcpp::SensorDataQoS(),
     [](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
       // process msg
     });

Free-function equivalents exist for cases where a
:cpp:class:`~point_cloud_transport::PointCloudTransport` object is inconvenient:

.. code-block:: cpp

   auto pub = point_cloud_transport::create_publisher(*node, "points", rclcpp::SystemDefaultsQoS());
   auto sub = point_cloud_transport::create_subscription(
     *node, "points", callback, "raw", rclcpp::SensorDataQoS());

Transport Selection
-------------------

The active transport is chosen at subscription time.  Priority order:

1. Explicit ``transport`` argument to ``create_subscription()``.
2. ROS parameter ``point_cloud_transport`` (name can be changed in ``TransportHints``).
3. Default: ``"raw"`` (can be changed in ``TransportHints``).

.. code-block:: cpp

   // Force Draco transport
   point_cloud_transport::TransportHints hints(*node, "draco");
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
     // Datatype of the transport-specific message, as "package/msg/Message".
     std::string getDataType() const override { return "my_msgs/msg/Compressed"; }

     // Declare any runtime parameters for this transport (none here).
     void declareParameters(const std::string & /*base_topic*/) override {}

     // Encode the raw cloud into the transport-specific message and return it.
     TypedEncodeResult encodeTyped(
       const sensor_msgs::msg::PointCloud2 & raw) const override
     {
       my_msgs::msg::Compressed compressed;
       // ... compress raw into compressed ...
       return compressed;
     }
   };

The transport name (``"my_transport"``) is taken from the ``<transport_name>``
element of the plugin manifest, so ``getTransportName()`` does not need to be
overridden.  Register the plugin with pluginlib via a ``plugins.xml`` file:

.. code-block:: xml
   :caption: plugins.xml

   <library path="my_transport_plugin">
     <transport_name>my_transport</transport_name>
     <message_type>my_msgs/msg/Compressed</message_type>
     <class name="point_cloud_transport/my_transport_pub"
            type="my_transport::MyPublisher"
            base_class_type="point_cloud_transport::PublisherPlugin">
       <description>My point cloud transport (publisher).</description>
     </class>
   </library>

See :doc:`plugin_api` for the full plugin-authoring guide.

Node-free Codec
---------------

:cpp:class:`~point_cloud_transport::PointCloudCodec` provides encode/decode without
a running ROS node — useful for offline tools and tests:

.. code-block:: cpp

   #include <point_cloud_transport/point_cloud_codec.hpp>

   point_cloud_transport::PointCloudCodec codec;

   // Encode a raw cloud into a transport-specific serialized message.
   rclcpp::SerializedMessage serialized;
   if (!codec.encode("draco", raw_cloud, serialized)) {
     std::cerr << "Encoding the pointcloud failed" << std::endl;
     return false;
   }

   // Decode it back into a PointCloud2.
   sensor_msgs::msg::PointCloud2 decoded;
   if (!codec.decode("draco", serialized, decoded)) {
     std::cerr << "Decoding the pointcloud failed" << std::endl;
     return false;
   }

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
