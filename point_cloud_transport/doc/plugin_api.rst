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

- ``getDataType()`` — datatype of the transport-specific message, as text in the
  form ``package/msg/Message``.
- ``advertiseImpl()`` — advertise the transport-specific topic(s) for the
  transport.
- ``encode()`` — convert a raw ``PointCloud2`` into the transport-specific type.
- ``publish()`` — send the encoded message on the wire.
- ``declareParameters()`` — declare any runtime parameters for the transport.
- ``shutdown()`` — release advertised publishers.

``getTransportName()`` (short identifier, e.g. ``"draco"``) and
``getMessageType()`` are *not* required overrides: their default implementations
read the ``<transport_name>`` and ``<message_type>`` elements of the pluginlib
manifest XML.  Override them only if you need a different value at runtime.

SubscriberPlugin
~~~~~~~~~~~~~~~~

:cpp:class:`point_cloud_transport::SubscriberPlugin` is the abstract base for all
subscriber plugins.  It subscribes to the transport-specific topic, decodes
incoming messages into ``sensor_msgs/PointCloud2``, and forwards them to the
user callback.

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1SubscriberPlugin`.

Required overrides (among others):

- ``getDataType()`` — datatype of the transport-specific message, as text in the
  form ``package/msg/Message``.
- ``subscribeImpl()`` — subscribe to the transport-specific topic(s).
- ``decode()`` — convert the transport-specific message back into ``PointCloud2``.
- ``declareParameters()`` — declare any runtime parameters for the transport.
- ``getTopic()`` / ``getNumPublishers()`` / ``shutdown()``.

As with the publisher base, ``getTransportName()`` and ``getMessageType()`` are
*not* required overrides — they default to the ``<transport_name>`` and
``<message_type>`` elements of the pluginlib manifest XML.

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

- ``encodeTyped(const sensor_msgs::msg::PointCloud2 &)`` — the conversion; returns
  the transport-specific message (``TypedEncodeResult``) directly.
- ``getDataType()`` — datatype of the transport-specific message.  This is still
  required today; a future release intends to derive it from the manifest and
  drop the override.
- ``declareParameters(const std::string & base_topic)`` — declare ROS
  parameters on the node so users can tune the transport at runtime.

.. code-block:: cpp

   #include <point_cloud_transport/simple_publisher_plugin.hpp>
   #include <my_msgs/msg/compressed.hpp>

   class MyPublisher
     : public point_cloud_transport::SimplePublisherPlugin<my_msgs::msg::Compressed>
   {
   public:
     std::string getDataType() const override { return this->getMessageType(); }

     void declareParameters(const std::string & base_topic) override
     {
       declareParam<int>(base_topic, "level", 5);
     }

     TypedEncodeResult encodeTyped(
       const sensor_msgs::msg::PointCloud2 & raw) const override
     {
       my_msgs::msg::Compressed compressed;
       // ... compress raw into compressed ...
       return compressed;
     }
   };

The transport name is taken from the ``<transport_name>`` element of the plugin
manifest, so ``getTransportName()`` does not need to be overridden.

SimpleSubscriberPlugin<M>
~~~~~~~~~~~~~~~~~~~~~~~~~

Full API page:
:ref:`exhale_class_classpoint__cloud__transport_1_1SimpleSubscriberPlugin`.

Subclasses must implement:

- ``decodeTyped(const M &)`` — the conversion; returns the decoded
  ``PointCloud2`` (``DecodeResult``) directly.
- ``getDataType()`` — datatype of the transport-specific message.  As on the
  publisher side this is still required today, with a pending issue to derive it
  from the manifest instead.
- ``declareParameters()`` — declare ROS parameters on the node.

.. code-block:: cpp

   class MySubscriber
     : public point_cloud_transport::SimpleSubscriberPlugin<my_msgs::msg::Compressed>
   {
   public:
     std::string getDataType() const override { return "my_msgs/msg/Compressed"; }

     void declareParameters() override {}

     DecodeResult decodeTyped(
       const my_msgs::msg::Compressed & compressed) const override
     {
       auto raw = std::make_shared<sensor_msgs::msg::PointCloud2>();
       // ... decompress compressed into *raw ...
       return raw;
     }
   };

Registering a Plugin
--------------------

Plugins are registered with pluginlib as usual.  The ``<transport_name>`` and
``<message_type>`` elements feed the default ``getTransportName()`` /
``getMessageType()`` implementations.  By convention, the publisher plugin's
lookup name ends with ``_pub`` and the subscriber's with ``_sub``:

.. code-block:: xml
   :caption: my_pkg_plugins.xml

   <library path="my_pkg_plugin">
     <transport_name>my_transport</transport_name>
     <message_type>my_msgs/msg/Compressed</message_type>
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

The plugin package must depend on ``point_cloud_transport`` (and on the package
that provides the transport-specific message) in its ``package.xml``:

.. code-block:: xml
   :caption: package.xml

   <depend>point_cloud_transport</depend>
   <depend>pluginlib</depend>
   <depend>sensor_msgs</depend>
   <depend>my_msgs</depend>

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

   // List the transports that can be loaded.
   std::vector<std::string> transports, names;
   codec.getLoadableTransports(transports, names);
   for (const auto & name : names) {
     std::cout << "Available transport: " << name << "\n";
   }

   // Encode a raw cloud into a transport-specific serialized message...
   rclcpp::SerializedMessage serialized;
   if (!codec.encode("draco", raw_cloud, serialized)) {
     std::cerr << "Encoding the pointcloud failed" << std::endl;
     return false;
   }

   // ...and decode it back into a PointCloud2.
   sensor_msgs::msg::PointCloud2 decoded;
   if (!codec.decode("draco", serialized, decoded)) {
     std::cerr << "Decoding the pointcloud failed" << std::endl;
     return false;
   }
