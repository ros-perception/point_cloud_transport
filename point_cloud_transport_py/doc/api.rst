API Reference
=============

Compiled Classes
----------------

The following classes are provided by the pybind11 bindings to
``point_cloud_transport``.  Their docstrings live in the compiled C++ module and
mirror the C++ API — see the
`point_cloud_transport <https://docs.ros.org/en/rolling/p/point_cloud_transport/>`__
documentation for a full description.

PointCloudTransport
~~~~~~~~~~~~~~~~~~~

Main entry point.  Owns the plugin loaders and exposes ``advertise()`` and
``subscribe()`` methods that return :class:`Publisher` and :class:`Subscriber`
handles.

.. code-block:: python

   from point_cloud_transport_py import PointCloudTransport
   pct = PointCloudTransport(node)
   pub = pct.advertise('points', 10)
   sub = pct.subscribe('points', 10, callback)

Publisher
~~~~~~~~~

Handle returned by :meth:`PointCloudTransport.advertise`.  Use
``publish(msg)`` to send a ``sensor_msgs/PointCloud2`` on every configured
transport; use ``shutdown()`` to release the underlying advertisements.

Subscriber
~~~~~~~~~~

Handle returned by :meth:`PointCloudTransport.subscribe`.  Calls the user
callback with a ``sensor_msgs/PointCloud2`` each time a message is received.
Use ``shutdown()`` to unsubscribe.

PointCloudCodec
~~~~~~~~~~~~~~~

Node-free encode/decode facade.  Provides ``getLoadableTransports()``,
``getEncoderByName()``, ``getDecoderByName()``, ``encode()``, and ``decode()``
methods that operate on serialized messages.

common module
-------------

The :mod:`point_cloud_transport_py.common` module provides serialization helpers
and the :class:`~point_cloud_transport_py.common.TransportInfo` dataclass:

- :class:`point_cloud_transport_py.common.TransportInfo`
- :func:`point_cloud_transport_py.common.stringToPointCloud2`
- :func:`point_cloud_transport_py.common.pointCloud2ToString`
- :func:`point_cloud_transport_py.common.stringToMsgType`

Full module documentation: :doc:`/point_cloud_transport_py.common`.

Package overview: :doc:`/point_cloud_transport_py`.
