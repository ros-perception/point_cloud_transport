# point_cloud_transport

[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

ROS2 Distro | Build Status | Package build |
:---------: | :----: | :----------: |
Rolling |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rdev__point_cloud_transport__ubuntu_noble_amd64)](https://build.ros2.org/job/Rdev__point_cloud_transport__ubuntu_noble_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__point_cloud_transport__ubuntu_noble_amd64__binary/) |
Kilted |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kdev__point_cloud_transport__ubuntu_noble_amd64)](https://build.ros2.org/job/Kdev__point_cloud_transport__ubuntu_noble_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__point_cloud_transport__ubuntu_noble_amd64__binary/) |
Jazzy |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jdev__point_cloud_transport__ubuntu_noble_amd64)](https://build.ros2.org/job/Jdev__point_cloud_transport__ubuntu_noble_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__point_cloud_transport__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__point_cloud_transport__ubuntu_noble_amd64__binary/) |
Humble |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__point_cloud_transport__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__point_cloud_transport__ubuntu_jammy_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__point_cloud_transport__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__point_cloud_transport__ubuntu_jammy_amd64__binary/) |

## Description

[point_cloud_transport](https://github.com/ros-perception/point_cloud_transport) is a [ROS 2](https://www.ros.org/) package for subscribing to and publishing [PointCloud2](http://docs.ros.org/latest/api/sensor_msgs/html/msg/PointCloud2.html) messages via different transport layers.
E.g. it can provide support for transporting point clouds in low-bandwidth environment using [Draco](https://github.com/google/draco) compression library.

## Usage

[point_cloud_transport](https://github.com/ros-perception/point_cloud_transport) can be used to publish and subscribe to [PointCloud2](http://docs.ros.org/latest/api/sensor_msgs/html/msg/PointCloud2.html) messages. At this level of usage, it is similar to using ROS 2 Publishers and Subscribers. Using [point_cloud_transport](https://github.com/ros-perception/point_cloud_transport) instead of the ROS 2 primitives, however, gives the user much greater flexibility in how point clouds are communicated between nodes.

For complete examples of publishing and subscribing to point clouds using [point_cloud_transport](https://github.com/ros-perception/point_cloud_transport), see [Tutorial](https://github.com/ros-perception/point_cloud_transport_tutorial).

### C++
Communicating PointCloud2 messages using [point_cloud_transport](https://github.com/ros-perception/point_cloud_transport):
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

Alternatively, you can use point_cloud_transport outside of ROS2.

```cpp
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_codec.hpp>

point_cloud_transport::PointCloudCodec codec;

sensor_msgs::msg::PointCloud2 msg;
// ... do some cool pointcloud generation stuff ...

// untyped version (outputs an rclcpp::SerializedMessage)
rclcpp::SerializedMessage serialized_msg;
bool success = codec.encode("draco", msg, serialized_msg);

// OR

// typed version (outputs whatever message your selected transport returns,
// for draco that is a point_cloud_interfaces::msg::CompressedPointCloud2)
point_cloud_interfaces::msg::CompressedPointCloud2 compressed_msg;
bool success = codec.encode("draco", msg, compressed_msg);
```

### Republish rclcpp component

We provide a process to republish any topic speaking in a given transport to a different topic speaking a different transport.
e.g. have it subscribe to a topic you recorded encoded using draco and publish it as a raw, decoded message

```bash
ros2 run point_cloud_transport republish --ros-args -p in_transport:=raw -p out_transport:=draco --remap in:=input_topic_name --remap out:=ouput_topic_name
```

### Python

The functionality of `point_cloud_transport` is also exposed to python via `pybind11` and `rclpy` serialization.

Please see [publisher.py](https://github.com/ros-perception/point_cloud_transport_tutorial/blob/jazzy/scripts/publisher.py) and [subscriber.py](https://github.com/ros-perception/point_cloud_transport_tutorial/blob/jazzy/scripts/subscriber_old_school.py) for example usage.

### Whitelist point cloud transport

This allows you to specify plugins you do want to load (a.k.a. whitelist them).

```bash
ros2 run point_cloud_transport_tutorial my_publisher --ros-args -p pct.point_cloud.enable_pub_plugins:=["point_cloud_transport/zlib"]
```

## Known transports

- [draco_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/jazzy/draco_point_cloud_transport): Lossy compression via Google
- [zlib_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/jazzy/zlib_point_cloud_transport): Lossless compression via Zlib compression.
- [zstd_point_cloud_transport](https://github.com/ros-perception/point_cloud_transport_plugins/tree/jazzy/zstd_point_cloud_transport): Lossless compression via Zstd compression.
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

This project is licensed under the BSD License - see the [LICENSE](https://github.com/ros-perception/point_cloud_transport/blob/master/LICENSE) file for details.

## Acknowledgments

* [image_transport](https://github.com/ros-perception/image_common) - Provided template of plugin interface
* [Draco](https://github.com/google/draco) - Provided compression functionality

Support
=======

For questions/comments please email the maintainer mentioned in `package.xml`.

If you have found an error in this package, please file an issue at: [https://github.com/ros-perception/point_cloud_transport/issues]()

Patches are encouraged, and may be submitted by forking this project and
submitting a pull request through GitHub. Any help is further development of the project is much appreciated.
