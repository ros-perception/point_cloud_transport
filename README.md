# \<POINT CLOUD TRANSPORT>
 **v0.1.**

_**Contents**_

  * [Description](#description)
  * [Getting Started](#getting-started)
    * [Installation](#installation)
      * [Prerequisites](#prerequisites)
      * [Setting up ROS workspace](#setting-up-ros-workspace)
      * [Building project](#building-project)
    * [Dependencies](#dependencies)
  * [Usage](#usage)
  * [Additional Information](#additional-information)
    * [Authors](#authors)
    * [License](#license)
    * [Acknowledgments](#acknowledgments)
  
    


Description
===========

[<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport) is a [ROS](https://www.ros.org/) package for subscribing to and publishing [PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html) messages. It provides support for transporting point clouds in low-bandwidth environment using [Draco](https://github.com/google/draco) compression library.

[<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport) is released as C++ source code.

Getting Started
===========

## Installation

### Prerequisites
- The Robot Operating System ( [ROS](https://www.ros.org/) ) \
http://wiki.ros.org/melodic/Installation

- Catkin Command Line Tools ( *optional* ) \
https://catkin-tools.readthedocs.io/en/latest/installing.html



### Setting up ROS workspace
Commands for creating workspace directory and cloning all necessary repositories:
~~~~~ bash
$ mkdir -p point_cloud_transport_ws/src
$ cd point_cloud_transport_ws/src
$ git clone https://github.com/paplhjak/draco.git
$ git clone https://github.com/paplhjak/point_cloud_transport.git
$ git clone https://github.com/paplhjak/draco_point_cloud_transport.git
$ git clone https://github.com/paplhjak/point_cloud_transport_plugins.git
~~~~~
### Building project
Commands for setting up catkin workspace and building the project:
~~~~~ bash
$ cd ..
$ catkin init
$ catkin config --extend /opt/ros/melodic
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
$ catkin build
~~~~~
To build the project without [Catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) use the following command in the root of the workspace:
~~~~~ bash
$ catkin_make_isolated
~~~~~

## Dependencies

* [ROS](https://www.ros.org/) - Framework
* [Draco](https://github.com/google/draco) - Compression library
* [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) - Command line tools for working with the catkin meta-buildsystem and catkin workspaces

Usage
======
[<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport) can be used to publish and subscribe to [PointCloud2](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html) messages. At this level of usage, it is similar to using ROS Publishers and Subscribers. Using [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport) instead of the ROS primitives, however, gives the user much greater flexibility in how point clouds are communicated between nodes.

For complete examples of publishing and subscribing to point clouds using [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport) , see [Tutorial](https://github.com/paplhjak/point_cloud_transport_tutorial). 

## C++
Communicating PointCloud2 messages using base [ROS](https://www.ros.org/)
publishers and subscribers:
```cpp
#include <ros/ros.h>

void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // ... process the message
}

ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe("in_point_cloud_topic", 1, Callback);
ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("out_point_cloud_topic", 1);
```

Communicating PointCloud2 messages using [<point_cloud_transport>](https://github.com/paplhjak/point_cloud_transport):
```cpp
#include <ros/ros.h>
#include <point_cloud_transport/point_cloud_transport.h>

void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // ... process the message
}

ros::NodeHandle nh;
point_cloud_transport::PointCloudTransport pct(nh);
point_cloud_transport::Subscriber sub = pct.subscribe("in_point_cloud_base_topic", 1, Callback);
point_cloud_transport::Publisher pub = pct.advertise("out_point_cloud_base_topic", 1);
```


Additional Information
======

## Authors

* **Jakub Paplhám** - *Initial work* - [paplhjak](https://github.com/paplhjak)
* **tpet** - *Supervisor* - [tpet](https://github.com/tpet)

See also the list of [contributors](https://github.com/users/paplhjak/projects/1/contributors).

## License

This project is licensed under the BSD License - see the [LICENSE.md](https://github.com/paplhjak/point_cloud_transport/blob/master/LICENSE) file for details.

## Acknowledgments

* [<image_transport>](http://wiki.ros.org/image_transport) - Provided template of plugin interface
* [Draco](https://github.com/google/draco) - Provided compression functionality

Support
=======

For questions/comments please email <paplhjak@fel.cvut.cz>

If you have found an error in this package, please file an issue at: \
<https://github.com/paplhjak/point_cloud_transport/issues>

Patches are encouraged, and may be submitted by forking this project and
submitting a pull request through GitHub.

