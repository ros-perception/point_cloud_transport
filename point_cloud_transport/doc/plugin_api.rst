Writing a Transport Plugin
==========================

``point_cloud_transport`` is a plugin framework.  New transports are implemented
as pluginlib plugins that derive from
:cpp:class:`~point_cloud_transport::PublisherPlugin` and
:cpp:class:`~point_cloud_transport::SubscriberPlugin` (or the template bases
:cpp:class:`~point_cloud_transport::SimplePublisherPlugin` /
:cpp:class:`~point_cloud_transport::SimpleSubscriberPlugin`).

This page is for transport authors.  End users of existing transports should see
:doc:`user_api`.

Base Classes
------------

PublisherPlugin
~~~~~~~~~~~~~~~

:cpp:class:`point_cloud_transport::PublisherPlugin` is the abstract base for all
publisher plugins.  It defines the minimal contract for advertising a transport,
encoding ``sensor_msgs/PointCloud2`` into a transport-specific message, and
publishing.

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1PublisherPlugin`.

Required overrides (among others):

- ``getTransportName()`` — short identifier (e.g. ``"draco"``).  The default
  implementation reads the ``type`` attribute of the pluginlib manifest.
- ``getMessageType()`` — fully-qualified name of the transport-specific message.
- ``advertiseImpl()`` — advertise one ROS publisher per output topic.
- ``encode()`` — convert a raw ``PointCloud2`` into the transport-specific type.
- ``publish()`` — send the encoded message on the wire.
- ``shutdown()`` — release advertised publishers.

SubscriberPlugin
~~~~~~~~~~~~~~~~

:cpp:class:`point_cloud_transport::SubscriberPlugin` is the abstract base for all
subscriber plugins.  It subscribes to the transport-specific topic, decodes
incoming messages into ``sensor_msgs/PointCloud2``, and forwards them to the
user callback.

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1SubscriberPlugin`.

Required overrides (among others):

- ``getTransportName()``
- ``getMessageType()``
- ``subscribeImpl()`` — create the underlying ROS subscription.
- ``decode()`` — convert the transport-specific message back into ``PointCloud2``.
- ``getTopic()`` / ``getNumPublishers()`` / ``shutdown()``.

Simple Template Bases
---------------------

For the common case of "single transport-specific message type in, ``PointCloud2``
out" (or vice versa), derive from the template bases rather than the abstract
classes.  They implement most of the boilerplate and only ask for the
encode/decode step and parameter declarations.

SimplePublisherPlugin<M>
~~~~~~~~~~~~~~~~~~~~~~~~

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1SimplePublisherPlugin`.

Subclasses must implement:

- ``encodeTyped(const sensor_msgs::msg::PointCloud2 &, M &)`` — the conversion.
- ``declareParameters(const std::string & base_topic)`` — declare ROS
  parameters on the node so users can tune the transport at runtime.

.. code-block:: cpp

   #include <point_cloud_transport/simple_publisher_plugin.hpp>
   #include <my_msgs/msg/compressed.hpp>

   class MyPublisher
     : public point_cloud_transport::SimplePublisherPlugin<my_msgs::msg::Compressed>
   {
   public:
     std::string getTransportName() const override { return "my_transport"; }

   protected:
     void declareParameters(const std::string & base_topic) override
     {
       declareParam<int>(base_topic, "level", 5);
     }

     EncodeResult encodeTyped(
       const sensor_msgs::msg::PointCloud2 & raw,
       my_msgs::msg::Compressed & compressed) override
     {
       // ... compress raw into compressed ...
       return true;
     }
   };

SimpleSubscriberPlugin<M>
~~~~~~~~~~~~~~~~~~~~~~~~~

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1SimpleSubscriberPlugin`.

Subclasses must implement:

- ``decodeTyped(const M &, sensor_msgs::msg::PointCloud2 &)`` — the conversion.
- ``declareParameters()`` — declare ROS parameters on the node.

.. code-block:: cpp

   class MySubscriber
     : public point_cloud_transport::SimpleSubscriberPlugin<my_msgs::msg::Compressed>
   {
   public:
     std::string getTransportName() const override { return "my_transport"; }

   protected:
     void declareParameters() override {}

     DecodeResult decodeTyped(
       const my_msgs::msg::Compressed & compressed,
       sensor_msgs::msg::PointCloud2 & raw) override
     {
       // ... decompress compressed into raw ...
       return true;
     }
   };

Registering a Plugin
--------------------

Plugins are registered with pluginlib as usual:

.. code-block:: xml
   :caption: my_pkg_plugins.xml

   <library path="my_pkg_plugin">
     <class
         name="point_cloud_transport/my_transport_pub"
         type="my_pkg::MyPublisher"
         base_class_type="point_cloud_transport::PublisherPlugin">
       <description>My point cloud transport (publisher).</description>
     </class>
     <class
         name="point_cloud_transport/my_transport_sub"
         type="my_pkg::MySubscriber"
         base_class_type="point_cloud_transport::SubscriberPlugin">
       <description>My point cloud transport (subscriber).</description>
     </class>
   </library>

And in ``CMakeLists.txt``:

.. code-block:: cmake

   pluginlib_export_plugin_description_file(point_cloud_transport my_pkg_plugins.xml)

Built-in Plugins
----------------

- :cpp:class:`point_cloud_transport::RawPublisher` /
  :cpp:class:`point_cloud_transport::RawSubscriber` — the built-in ``"raw"``
  transport.  Carries uncompressed ``PointCloud2`` messages.
  Full API pages:
  :ref:`exhale_class_classpoint__cloud__transport_1_1RawPublisher`,
  :ref:`exhale_class_classpoint__cloud__transport_1_1RawSubscriber`.

Node-free Codec
---------------

:cpp:class:`point_cloud_transport::PointCloudCodec` exposes encode/decode for all
loaded plugins without requiring a running ROS node.  It is useful for offline
tools, unit tests, and language bindings that do not want to wrap a full
:cpp:class:`~point_cloud_transport::PointCloudTransport`.

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1PointCloudCodec`.

.. code-block:: cpp

   point_cloud_transport::PointCloudCodec codec;
   for (const auto & name : codec.getLoadableTransports()) {
     std::cout << "Available transport: " << name << "\n";
   }

   auto encoder = codec.getEncoderByName("draco");
   rclcpp::SerializedMessage out;
   encoder->encode(raw_cloud, out);
