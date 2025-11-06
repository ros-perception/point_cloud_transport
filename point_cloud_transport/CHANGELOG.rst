^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package point_cloud_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.20 (2025-11-06)
-------------------
* Fix duplicate component registration for Republisher (`#142 <https://github.com/ros-perception/point_cloud_transport/issues/142>`_) (`#145 <https://github.com/ros-perception/point_cloud_transport/issues/145>`_)
* Removed outdated comment (`#138 <https://github.com/ros-perception/point_cloud_transport/issues/138>`_) (`#141 <https://github.com/ros-perception/point_cloud_transport/issues/141>`_)
* Contributors: mergify[bot]

1.0.19 (2025-10-15)
-------------------
* Use standard unsigned int in place of uint for Windows compatibility (backport `#134 <https://github.com/ros-perception/point_cloud_transport/issues/134>`_) (`#137 <https://github.com/ros-perception/point_cloud_transport/issues/137>`_)
* Contributors: mergify[bot]

1.0.18 (2024-05-16)
-------------------
* [rolling] Get user specified parameters at startup (`#80 <https://github.com/ros-perception/point_cloud_transport/issues/80>`_) (`#83 <https://github.com/ros-perception/point_cloud_transport/issues/83>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 90c603a1e8fb56c3203ff6870e4f2205c37e59b4)
  Co-authored-by: john-maidbot <78750993+john-maidbot@users.noreply.github.com>
* Rename the republish_node to pc_republish_node. (`#75 <https://github.com/ros-perception/point_cloud_transport/issues/75>`_) (`#77 <https://github.com/ros-perception/point_cloud_transport/issues/77>`_)
  The major reason for this is that image_transport already
  has a republish_node, and when we are building for distribution
  we can't have two files named /opt/ros/rolling/lib/librepublish_node.so
  Rename this one to libpc_republish_node, which should remove
  the conflict.
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>
* Contributors: Alejandro Hernández Cordero, mergify[bot]

1.0.17 (2023-03-26)
-------------------
* Cleanup republisher (`#58 <https://github.com/ros-perception/point_cloud_transport/issues/58>`_) (`#70 <https://github.com/ros-perception/point_cloud_transport/issues/70>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Improve Windows support (`#50 <https://github.com/ros-perception/point_cloud_transport/issues/50>`_) (`#61 <https://github.com/ros-perception/point_cloud_transport/issues/61>`_)
* Fixed MacOS M1 build (`#57 <https://github.com/ros-perception/point_cloud_transport/issues/57>`_) (`#59 <https://github.com/ros-perception/point_cloud_transport/issues/59>`_)
* Contributors: Alejandro Hernández Cordero, john-maidbot

1.0.16 (2023-02-19)
-------------------
* Change tl_expected for rcpputils (`#48 <https://github.com/ros-perception/point_cloud_transport/issues/48>`_) (`#54 <https://github.com/ros-perception/point_cloud_transport/issues/54>`_)
* Clean CMake (`#49 <https://github.com/ros-perception/point_cloud_transport/issues/49>`_) (`#53 <https://github.com/ros-perception/point_cloud_transport/issues/53>`_)
* Contributors: Alejandro Hernández Cordero

1.0.15 (2023-12-12)
-------------------
* Fixed draco subscriber parameter names (`#43 <https://github.com/ros-perception/point_cloud_transport/issues/43>`_) (`#44 <https://github.com/ros-perception/point_cloud_transport/issues/44>`_)
  (cherry picked from commit 48cd0ced3dcf12d13bf648a903d691355480b18b)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

1.0.14 (2023-10-05)
-------------------
* Fix param name (`#39 <https://github.com/ros-perception/point_cloud_transport/issues/39>`_) (`#40 <https://github.com/ros-perception/point_cloud_transport/issues/40>`_)
  (cherry picked from commit 7acc9458dbfd75dcb4e8e2c984fd16e5d5d5aac8)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Fixed param name (`#36 <https://github.com/ros-perception/point_cloud_transport/issues/36>`_) (`#37 <https://github.com/ros-perception/point_cloud_transport/issues/37>`_)
  (cherry picked from commit 851434a59ef2de7bccb1a46e27882c0480534289)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

1.0.13 (2023-09-20)
-------------------
* feat: replace third party expected with ros package (`#32 <https://github.com/ros-perception/point_cloud_transport/issues/32>`_) (`#33 <https://github.com/ros-perception/point_cloud_transport/issues/33>`_)
  (cherry picked from commit d13b7a2feb63c82cbd619a99a7eed7c95f9ac558)
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
* Contributors: mergify[bot], Daisuke Nishimatsu

1.0.12 (2023-09-18)
-------------------
* ROS2 port
* Contributors: Alejandro Hernández Cordero, John D'Angelo

1.0.11 (2023-06-16)
-------------------
* Fixed bad_expected_access error when pointcloud encoding fails.
* Added known transports to readme.
* Update README.md
* Contributors: Martin Pecka

1.0.10 (2023-05-24)
-------------------
* Fixed wrong dependency version.
* Contributors: Martin Pecka

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
