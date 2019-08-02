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
