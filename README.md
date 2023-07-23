# point_cloud_transport

## Description

[point_cloud_transport](https://github.com/john-maidbot/point_cloud_transport) is a [ROS 2](https://www.ros.org/) package for subscribing to and publishing [PointCloud2](http://docs.ros.org/latest/api/sensor_msgs/html/msg/PointCloud2.html) messages via different transport layers.
E.g. it can provide support for transporting point clouds in low-bandwidth environment using [Draco](https://github.com/google/draco) compression library.

[point_cloud_transport](https://github.com/john-maidbot/point_cloud_transport) is **NOT yet** released as C++ source code and binary packages via ROS buildfarm.

## Usage

[point_cloud_transport](https://github.com/john-maidbot/point_cloud_transport) can be used to publish and subscribe to [PointCloud2](http://docs.ros.org/latest/api/sensor_msgs/html/msg/PointCloud2.html) messages. At this level of usage, it is similar to using ROS 2 Publishers and Subscribers. Using [point_cloud_transport](https://github.com/john-maidbot/point_cloud_transport) instead of the ROS 2 primitives, however, gives the user much greater flexibility in how point clouds are communicated between nodes.

For complete examples of publishing and subscribing to point clouds using [point_cloud_transport](https://github.com/john-maidbot/point_cloud_transport), see [Tutorial](https://github.com/ahcorde/point_cloud_transport_tutorial).

### C++
Communicating PointCloud2 messages using [point_cloud_transport](https://github.com/john-maidbot/point_cloud_transport):
```cpp
#include <rclcpp/rclcpp.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>

void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
  // ... process the message
}

auto node = std::make_shared<rclcpp::Node>();
point_cloud_transport::PointCloudTransport pct(node);
point_cloud_transport::Subscriber sub = pct.subscribe("in_point_cloud_base_topic", 1, Callback);
point_cloud_transport::Publisher pub = pct.advertise("out_point_cloud_base_topic", 1);
```

### Republish rclcpp component

TODO


### Python bindings


### Python




### Blacklist point cloud transport

This allows you to specify plugins you do not want to load (a.k.a. blacklist them).

```bash
ros2 run point_cloud_transport_tutorial my_publisher <rosbag2 file> --ros-args -p /pct/point_cloud/disable_pub_plugins:=["point_cloud_transport/raw"]
```

## Known transports

- [draco_point_cloud_transport](https://wiki.ros.org/draco_point_cloud_transport): Lossy compression via Google
- [zlib_point_cloud_transport](???): Lossless compression via Zlib compression.
- Did you write one? Don't hesitate and send a pull request adding it to this list!

## Authors

### ROS 1 Version:

* **Jakub Paplhám** - *Initial work* - [paplhjak](https://github.com/paplhjak)
* **Ing. Tomáš Petříček, Ph.D.** - *Supervisor* - [tpet](https://github.com/tpet)
* **Martin Pecka** - *Maintainer* - [peci1](https://github.com/peci1)

### ROS 2 Version:

 * **John D'Angelo** - *Port to ROS 2 and Maintainer* - [john-maidbot](https://github.com/john-maidbot)
 * **Alejandro Hernández** - *Port to ROS 2 and Maintainer* - [ahcorde](https://github.com/ahcorde)

## License

This project is licensed under the BSD License - see the [LICENSE](https://github.com/john-maidbot/point_cloud_transport/blob/master/LICENSE) file for details.

## Acknowledgments

* [image_transport](http://wiki.ros.org/image_transport) - Provided template of plugin interface
* [Draco](https://github.com/google/draco) - Provided compression functionality

Support
=======

For questions/comments please email the maintainer mentioned in `package.xml`.

If you have found an error in this package, please file an issue at: [https://github.com/john-maidbot/point_cloud_transport/issues]()

Patches are encouraged, and may be submitted by forking this project and
submitting a pull request through GitHub. Any help is further development of the project is much appreciated.
