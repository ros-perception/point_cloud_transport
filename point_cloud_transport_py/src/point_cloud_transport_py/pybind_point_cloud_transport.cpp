// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <thread>

#include <point_cloud_transport/subscriber.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include "./pybind11.hpp"

namespace point_cloud_transport_python
{
// Bindings for the PointCloudCodec class
PYBIND11_MODULE(_point_cloud_transport, m)
{
  m.doc() = "Python wrapper of the point_cloud_transport API";

  pybind11::class_<point_cloud_transport::Publisher>(m, "Publisher")
  .def(pybind11::init())
  .def("get_topic",
      &point_cloud_transport::Publisher::getTopic,
      "Returns the base point cloud topic.")
  .def("get_num_subscribers",
      &point_cloud_transport::Publisher::getNumSubscribers,
      "Returns the number of subscribers this publisher is connected to.")
  .def("shutdown", &point_cloud_transport::Publisher::shutdown,
      "Unsubscribe the callback associated with this Publisher.")
  .def("publish",[] (point_cloud_transport::Publisher &publisher,
                     std::string &buffer) {
       sensor_msgs::msg::PointCloud2 pc2;
       rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

       rcl_serialized_message_t raw_serialized_msg = rmw_get_zero_initialized_serialized_message();
       // Allocate memory for the serialized message
       raw_serialized_msg.buffer_capacity = buffer.size();
       raw_serialized_msg.buffer_length = buffer.size();
       raw_serialized_msg.buffer = reinterpret_cast<uint8_t *>(const_cast<char *>(buffer.c_str()));
       raw_serialized_msg.allocator = rcl_get_default_allocator();
       if (!raw_serialized_msg.buffer) {
         throw std::runtime_error("Failed to allocate memory for serialized message.");
       }

       // Copy the string data into the serialized message buffer
       rclcpp::SerializedMessage extracted_serialized_msg(raw_serialized_msg);
       serialization.deserialize_message(&extracted_serialized_msg, &pc2);

       publisher.publish(pc2);
       },
       "Publish a point cloud on the topics associated with this Publisher.");

  pybind11::class_<point_cloud_transport::PointCloudTransport>(m, "PointCloudTransport")
  .def(pybind11::init([](const std::string& node_name, const std::string& launch_params_filepath) {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
    rclcpp::NodeOptions node_options;

    if (!launch_params_filepath.empty())
    {
      node_options.allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
        .arguments({ "--ros-args", "--params-file", launch_params_filepath });
    }
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(node_name, "", node_options);

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    auto spin_node = [node, executor]() {
      executor->add_node(node);
      executor->spin();
    };
    std::thread execution_thread(spin_node);
    execution_thread.detach();

    return point_cloud_transport::PointCloudTransport(node);
  }))
  .def("advertise",
       pybind11::overload_cast<const std::string &, uint32_t>(
         &point_cloud_transport::PointCloudTransport::advertise));
  // TODO(ahcorde): Revisit this method
  // .def("subscribe",
  //      [](point_cloud_transport::PointCloudTransport & pct,
  //         const std::string & base_topic,
  //         uint32_t queue_size,
  //         const std::function<void(const char *)> & callback)
  //    {
  //      return pct.subscribe_lol(
  //        base_topic, queue_size,
  //        [callback](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  //        {
  //          std::cerr << "1" << '\n';
  //          rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  //          rclcpp::SerializedMessage serialized_msg;
  //
  //          serialization.serialize_message(msg.get(), &serialized_msg);
  //
  //          std::string buffer;
  //          buffer.resize(serialized_msg.get_rcl_serialized_message().buffer_capacity);
  //          std::cerr << "buffer_capacity: " << serialized_msg.get_rcl_serialized_message().buffer_capacity << '\n';
  //          memcpy(buffer.data(), serialized_msg.get_rcl_serialized_message().buffer, serialized_msg.get_rcl_serialized_message().buffer_capacity);
  //
  //          std::cerr << "2" << '\n';
  //          callback(buffer.c_str());
  //          std::cerr << "3" << '\n';
  //        });
  //    });
  // ;

  pybind11::class_<point_cloud_transport::Subscriber>(m, "Subscriber")
  .def(pybind11::init())
  .def("get_topic",
       &point_cloud_transport::Subscriber::getTopic,
       "Returns the base point cloud topic.")
  .def("get_num_publishers",
       &point_cloud_transport::Subscriber::getNumPublishers,
       "Returns the number of publishers this subscriber is connected to.")
  .def("get_transport",
       &point_cloud_transport::Subscriber::getTransport,
       "Returns the name of the transport being used.")
  .def("shutdown", &point_cloud_transport::Subscriber::shutdown,
       "Unsubscribe the callback associated with this Subscriber.");

}
}  // namespace point_cloud_transport_python
