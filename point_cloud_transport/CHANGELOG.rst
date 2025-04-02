^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package point_cloud_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.4 (2025-04-02)
------------------
* fix: add `rclcpp::shutdown` (`#110 <https://github.com/ros-perception/point_cloud_transport/issues/110>`_) (`#111 <https://github.com/ros-perception/point_cloud_transport/issues/111>`_)
  (cherry picked from commit 40fec8f3e30b2ed68f6f5ad4915b098cece4c68a)
  Co-authored-by: Yuyuan Yuan <az6980522@gmail.com>
* Contributors: mergify[bot]

4.0.3 (2024-12-18)
------------------

4.0.2 (2024-06-27)
------------------
* Stop using ament_target_dependencies. (`#86 <https://github.com/ros-perception/point_cloud_transport/issues/86>`_) (`#87 <https://github.com/ros-perception/point_cloud_transport/issues/87>`_)
  Co-authored-by: Chris Lalancette <clalancette@gmail.com>

4.0.1 (2024-05-24)
------------------
* [rolling] Get user specified parameters at startup (`#80 <https://github.com/ros-perception/point_cloud_transport/issues/80>`_) (`#82 <https://github.com/ros-perception/point_cloud_transport/issues/82>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 90c603a1e8fb56c3203ff6870e4f2205c37e59b4)
  Co-authored-by: john-maidbot <78750993+john-maidbot@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

4.0.0 (2024-03-29)
------------------
* Rename the republish_node to pc_republish_node. (`#75 <https://github.com/ros-perception/point_cloud_transport/issues/75>`_)
* Fixed flake8 errors (`#72 <https://github.com/ros-perception/point_cloud_transport/issues/72>`_)
* Added documentation (`#69 <https://github.com/ros-perception/point_cloud_transport/issues/69>`_)
* Contributors: Alejandro Hernández Cordero, Chris Lalancette

3.0.5 (2024-03-15)
-------------------
* Fixed SubscriberFilter linking issue on windows (`#64 <https://github.com/ros-perception/point_cloud_transport/issues/64>`_)
* Contributors: Alejandro Hernández Cordero

3.0.4 (2023-02-19)
-------------------
* Cleanup republisher (`#58 <https://github.com/ros-perception/point_cloud_transport/issues/58>`_)
* Fixed clang issues (`#55 <https://github.com/ros-perception/point_cloud_transport/issues/55>`_)
* Fixed MacOS M1 build (`#57 <https://github.com/ros-perception/point_cloud_transport/issues/57>`_)
* Contributors: Alejandro Hernández Cordero

3.0.3 (2023-02-13)
-------------------
* Improve Windows support (`#50 <https://github.com/ros-perception/point_cloud_transport//issues/50>`_)
* Change tl_expected for rcpputils (`#48 <https://github.com/ros-perception/point_cloud_transport//issues/48>`_)
* Clean CMake (`#49 <https://github.com/ros-perception/point_cloud_transport//issues/49>`_)
* Contributors: Alejandro Hernández Cordero

3.0.2 (2023-12-12)
-------------------
* Fixed draco subscriber parameter names (`#43 <https://github.com/ros-perception/point_cloud_transport/issues/43>`_)
* Contributors: Alejandro Hernández Cordero

3.0.1 (2023-10-05)
-------------------
* Fix param name (`#39 <https://github.com/ros-perception/point_cloud_transport/issues/39>`_)
* Fixed param name (`#36 <https://github.com/ros-perception/point_cloud_transport/issues/36>`_)
* Contributors: Alejandro Hernández Cordero

3.0.0 (2023-09-20)
-------------------
* feat: replace third party expected with ros package (`#32 <https://github.com/ros-perception/point_cloud_transport/issues/32>`_)
* fix: modify wrong install for header (`#30 <https://github.com/ros-perception/point_cloud_transport/issues/30>`_)
* Contributors: Daisuke Nishimatsu

2.0.0 (2023-09-18)
-------------------
* Removed warning (`#28 <https://github.com/ros-perception/point_cloud_transport/issues/28>`_)
* Added point_cloud_transport_py (`#26 <https://github.com/ros-perception/point_cloud_transport/issues/26>`_)
* Bug fixes from porting tutorials (`#18 <https://github.com/ros-perception/point_cloud_transport/issues/18>`_)
* Use whitelist instead of blacklist (`#13 <https://github.com/ros-perception/point_cloud_transport/issues/13>`_)
* Add ThirdParty folder to support building offline without FetchContent (`#12 <https://github.com/ros-perception/point_cloud_transport/issues/12>`_)
* Fix pointcloud-codec and python bindings (`#3 <https://github.com/ros-perception/point_cloud_transport/issues/3>`_)
* Contributors: Alejandro Hernández Cordero, john-maidbot
