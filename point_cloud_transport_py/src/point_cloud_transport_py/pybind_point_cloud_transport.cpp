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

#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <point_cloud_transport/subscriber.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>

#include "./pybind11.hpp"
#include "./borrowed_serialized_message.hpp"

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
  .def("publish", [](point_cloud_transport::Publisher & publisher,
    const pybind11::buffer & buffer) {
      // Deserialize straight from the Python memory, without intermediate copies.
      sensor_msgs::msg::PointCloud2 pc2;
      {
        point_cloud_transport_python::BorrowedSerializedMessage borrowed(buffer);
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
        serialization.deserialize_message(&borrowed.get(), &pc2);
      }
      publisher.publish(pc2);
       },
       "Publish a serialized sensor_msgs/PointCloud2 (bytes) on the topics associated with this "
       "Publisher.");

  pybind11::class_<point_cloud_transport::PointCloudTransport>(m, "PointCloudTransport")
  .def(pybind11::init([](const std::string & node_name,
    const std::string & launch_params_filepath) {
      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      rclcpp::NodeOptions node_options;

      if (!launch_params_filepath.empty()) {
        node_options.allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
        .arguments({"--ros-args", "--params-file", launch_params_filepath});
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

      return point_cloud_transport::PointCloudTransport(*node);
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
  //          std::cerr << "buffer_capacity: "
  //                    << serialized_msg.get_rcl_serialized_message().buffer_capacity
  //                    << '\n';
  //          memcpy(
  //            buffer.data(),
  //            serialized_msg.get_rcl_serialized_message().buffer,
  //            serialized_msg.get_rcl_serialized_message().buffer_capacity);
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
