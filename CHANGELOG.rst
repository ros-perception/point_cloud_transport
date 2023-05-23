^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package point_cloud_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2023-05-23)
------------------
* Fixed bad_expected_access bug in simple_subscriber_plugin.
* Contributors: Martin Pecka

1.0.8 (2023-05-15)
------------------
* Upstreamed NumberAllocator to cras_py_common.
* Contributors: Martin Pecka

1.0.7 (2023-05-15)
------------------
* Upstreamed Python support functions to cras_py_common v 2.2.1.
* Contributors: Martin Pecka

1.0.6 (2023-05-15)
------------------
* Split Python API into several files.
* Improved Python API to allow the pub/sub-like usage.
* Contributors: Martin Pecka

1.0.5 (2023-05-12)
------------------
* Fixed bug in republish. Plugin blacklist was acting exactly the opposite way.
* Contributors: Martin Pecka

1.0.4 (2023-05-12)
------------------
* Turned republish into a nodelet (but kept its node version).
* Contributors: Martin Pecka

1.0.3 (2023-05-11)
------------------
* Added Python API for encode()/decode().
* Contributors: Martin Pecka

1.0.2 (2023-05-11)
------------------
* Added possibility to report log messages.
* Contributors: Martin Pecka

1.0.1 (2023-05-11)
------------------
* Reworked the idea to include direct encoders/decoders and not only publishers/subscribers.
* Initial cleanup before releasing. No substantial changes made.
* Forked from https://github.com/paplhjak/point_cloud_transport
* Contributors: Jakub Paplham, Martin Pecka, Tomas Petricek
