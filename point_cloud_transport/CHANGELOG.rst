^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package point_cloud_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

5.3.0 (2025-07-25)
------------------
* Update subscriber filter (`#126 <https://github.com/ros-perception/point_cloud_transport/issues/126>`_)
* Simplify NodeInterface API mehotd call (`#129 <https://github.com/ros-perception/point_cloud_transport/issues/129>`_)
* Fixed QOS override tests (`#128 <https://github.com/ros-perception/point_cloud_transport/issues/128>`_)
* Contributors: Alejandro Hernández Cordero, ElSayed ElSheikh

5.2.2 (2025-06-27)
------------------
* Deprecated rmw_qos_profile_t (`#125 <https://github.com/ros-perception/point_cloud_transport/issues/125>`_)
* Contributors: Alejandro Hernández Cordero

5.2.1 (2025-06-04)
------------------
* Feat/Add LifecycleNode Support (`#109 <https://github.com/ros-perception/point_cloud_transport/issues/109>`_)
* Contributors: ElSayed ElSheikh

5.2.0 (2025-04-28)
------------------

5.1.2 (2025-04-28)
------------------
* Add `rclcpp::shutdown` (`#110 <https://github.com/ros-perception/point_cloud_transport/issues/110>`_)
* Contributors: Yuyuan Yuan

5.1.1 (2024-11-25)
------------------

5.1.0 (2024-11-20)
------------------

5.0.4 (2024-10-03)
------------------

5.0.3 (2024-07-19)
------------------
* Updated deprecated message filter headers (`#94 <https://github.com/ros-perception/point_cloud_transport/issues/94>`_)
* Contributors: Alejandro Hernández Cordero

5.0.2 (2024-06-17)
------------------

5.0.1 (2024-06-14)
------------------
* Removed warning (`#89 <https://github.com/ros-perception/point_cloud_transport/issues/89>`_)
* republisher: qos override pub and sub (`#88 <https://github.com/ros-perception/point_cloud_transport/issues/88>`_)
* Stop using ament_target_dependencies. (`#86 <https://github.com/ros-perception/point_cloud_transport/issues/86>`_)
  We are slowly moving away from its use, so stop using it
  here.  While we are in here, notice some things that makes
  this easier:
  1. pluginlib is absolutely a public dependency of this package.
  Because of that, we can just rely on the PUBLIC export of it,
  and we don't need to link it into every test.  But that also means
  we don't need some of the forward-declarations that were in
  loader_fwds.hpp, as we can just get those through the header file.
  2. republish.hpp doesn't really need to exist at all.  That's
  because it is only a header file, but the implementation is in
  an executable.  Thus, no downstream could ever use it.  So just
  remove the file and put the declaration straight into the cpp file.
* Contributors: Alejandro Hernández Cordero, Chris Lalancette

5.0.0 (2024-05-13)
------------------
* [rolling] Get user specified parameters at startup (`#80 <https://github.com/ros-perception/point_cloud_transport//issues/80>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: john-maidbot

4.1.0 (2024-04-30)
------------------

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
