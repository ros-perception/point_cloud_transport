User API
========

This page documents the classes most users of ``point_cloud_transport`` will interact
with directly: :cpp:class:`~point_cloud_transport::PointCloudTransport`,
:cpp:class:`~point_cloud_transport::Publisher`,
:cpp:class:`~point_cloud_transport::Subscriber`, and
:cpp:class:`~point_cloud_transport::TransportHints`.

For plugin authors, see :doc:`Writing a Transport Plugin <plugin_api>`.  For the full auto-generated Doxygen
listing, see :ref:`exhale_class_classpoint__cloud__transport_1_1PointCloudTransport`
and follow cross-links from there.

PointCloudTransport
-------------------

:cpp:class:`point_cloud_transport::PointCloudTransport` is the main entry point.
It owns the pluginlib loaders and exposes ``advertise()`` / ``subscribe()``
methods analogous to ``rclcpp::Node``.

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1PointCloudTransport`.

Construction
~~~~~~~~~~~~

Construct a :cpp:class:`~point_cloud_transport::PointCloudTransport` from either
a ``rclcpp::Node`` (deprecated) or a ``NodeInterfaces`` bundle.  The
``NodeInterfaces`` form is preferred because it works with lifecycle nodes and
component composition.

.. code-block:: cpp

   #include <point_cloud_transport/point_cloud_transport.hpp>
   #include <rclcpp/rclcpp.hpp>

   auto node = std::make_shared<rclcpp::Node>("publisher_node");
   point_cloud_transport::PointCloudTransport pct(*node);

Advertising a topic
~~~~~~~~~~~~~~~~~~~

``advertise()`` creates a :cpp:class:`~point_cloud_transport::Publisher` that
internally advertises one ROS publisher per available transport:

.. code-block:: cpp

   auto pub = pct.advertise("points", rclcpp::SystemDefaultsQoS());

   sensor_msgs::msg::PointCloud2 cloud;
   // ... populate cloud ...
   pub.publish(cloud);

Subscribing to a topic
~~~~~~~~~~~~~~~~~~~~~~

``subscribe()`` accepts a callback on ``sensor_msgs::msg::PointCloud2`` and
returns a :cpp:class:`~point_cloud_transport::Subscriber`.  The active transport
is chosen from (in order): the ``TransportHints`` argument, the
``point_cloud_transport`` node parameter, or ``"raw"``.

.. code-block:: cpp

   void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
     RCLCPP_INFO(rclcpp::get_logger("sub"), "Got %u points", msg->width * msg->height);
   }

   auto sub = pct.subscribe("points", rclcpp::SensorDataQoS(), callback);

Member-function callbacks are supported via additional overloads:

.. code-block:: cpp

   class Consumer : public rclcpp::Node {
   public:
     Consumer() : Node("consumer") {
       transport_ = std::make_unique<point_cloud_transport::PointCloudTransport>(
         *this);
       sub_ = transport_->subscribe(
         "points", rclcpp::SensorDataQoS(),
         &Consumer::onCloud, this);
     }
   private:
     void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) { /* ... */ }
     std::unique_ptr<point_cloud_transport::PointCloudTransport> transport_;
     point_cloud_transport::Subscriber sub_;
   };

Publisher
---------

:cpp:class:`point_cloud_transport::Publisher` is a handle to one or more
transport-specific publishers.  It is returned by
:cpp:func:`~point_cloud_transport::PointCloudTransport::advertise` and by the
:cpp:func:`point_cloud_transport::create_publisher` free function.

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1Publisher`.

Common methods:

- ``publish(msg)`` — encode and publish on all active transports.
- ``getNumSubscribers()`` — total subscribers across all transports.
- ``getTopic()`` — the base topic.
- ``getPublishers()`` — underlying ``rclcpp::PublisherBase`` handles keyed by transport name.
- ``shutdown()`` — close all transports.

Subscriber
----------

:cpp:class:`point_cloud_transport::Subscriber` hides transport details behind a
standard ``PointCloud2`` callback.

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1Subscriber`.

Common methods:

- ``getTopic()`` — the base topic (not the transport-specific sub-topic).
- ``getTransport()`` — the name of the transport in use.
- ``getNumPublishers()`` — publishers on the transport-specific sub-topic.
- ``getSubscription()`` — the underlying ``rclcpp::SubscriptionBase``.
- ``shutdown()`` — unsubscribe.

TransportHints
--------------

:cpp:class:`point_cloud_transport::TransportHints` selects which transport a
:cpp:class:`~point_cloud_transport::Subscriber` should use.  It can look up the
default from a ROS parameter, allowing users to change transports at launch time
without recompiling.

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1TransportHints`.

.. code-block:: cpp

   // Select transport explicitly
   point_cloud_transport::TransportHints hints(*node, "draco");
   auto sub = pct.subscribe("points", qos, callback, {}, &hints);

   // Or rely on the 'point_cloud_transport' parameter:
   //   ros2 run my_pkg my_node --ros-args -p point_cloud_transport:=zlib

Free Functions
--------------

For callers that do not want to keep a
:cpp:class:`~point_cloud_transport::PointCloudTransport` object alive,
free-function equivalents exist:

- :cpp:func:`point_cloud_transport::create_publisher`
- :cpp:func:`point_cloud_transport::create_subscription`

.. code-block:: cpp

   auto pub = point_cloud_transport::create_publisher(
     *node, "points", rclcpp::SystemDefaultsQoS());

   auto sub = point_cloud_transport::create_subscription(
     *node, "points", callback, "raw",
     rclcpp::SensorDataQoS());
