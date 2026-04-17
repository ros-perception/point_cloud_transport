API Reference
=============

The ``point_cloud_transport`` API is split between end-users and plugin authors:

- :doc:`user_api` — classes and functions for applications that publish or
  subscribe to point clouds.
- :doc:`Writing a Transport Plugin <plugin_api>` — base classes and
  registration hooks for authoring new transport plugins.

Public API Classes
------------------

End-user classes:

- :cpp:class:`point_cloud_transport::PointCloudTransport` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1PointCloudTransport`
- :cpp:class:`point_cloud_transport::PointCloudTransportLoader` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1PointCloudTransportLoader`
- :cpp:class:`point_cloud_transport::Publisher` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1Publisher`
- :cpp:class:`point_cloud_transport::Subscriber` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1Subscriber`
- :cpp:class:`point_cloud_transport::TransportHints` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1TransportHints`
- :cpp:class:`point_cloud_transport::SubscriberFilter` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1SubscriberFilter`

Plugin-author classes:

- :cpp:class:`point_cloud_transport::PublisherPlugin` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1PublisherPlugin`
- :cpp:class:`point_cloud_transport::SubscriberPlugin` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1SubscriberPlugin`
- :cpp:class:`point_cloud_transport::SimplePublisherPlugin` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1SimplePublisherPlugin`
- :cpp:class:`point_cloud_transport::SimpleSubscriberPlugin` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1SimpleSubscriberPlugin`
- :cpp:class:`point_cloud_transport::SingleSubscriberPublisher` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1SingleSubscriberPublisher`
- :cpp:class:`point_cloud_transport::RawPublisher` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1RawPublisher`
- :cpp:class:`point_cloud_transport::RawSubscriber` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1RawSubscriber`
- :cpp:class:`point_cloud_transport::PointCloudCodec` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1PointCloudCodec`

Exceptions:

- :cpp:class:`point_cloud_transport::Exception` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1Exception`
- :cpp:class:`point_cloud_transport::TransportLoadException` —
  :ref:`exhale_class_classpoint__cloud__transport_1_1TransportLoadException`
