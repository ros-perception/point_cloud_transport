// Copyright (c) 2023, John D'Angelo
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/types.h>
#include <rcl/allocator.h>
#include <rclcpp/serialization.hpp>

#include <point_cloud_transport/point_cloud_codec.hpp>

namespace py = pybind11;
using namespace point_cloud_transport;  // NOLINT

// Utilities for handling the transfer of information to/from python
namespace point_cloud_transport
{
void stringToSerializedMsg(const std::string & buffer, rclcpp::SerializedMessage & serial_msg)
{
  rcl_serialized_message_t raw_serialized_msg = rmw_get_zero_initialized_serialized_message();

  // Allocate memory for the serialized message
  raw_serialized_msg.buffer_capacity = buffer.size();
  raw_serialized_msg.buffer_length = buffer.size();
  raw_serialized_msg.buffer = static_cast<uint8_t *>(malloc(raw_serialized_msg.buffer_capacity));

  if (!raw_serialized_msg.buffer) {
    // Handle memory allocation error
    // For simplicity, we just throw an exception here
    throw std::runtime_error("Failed to allocate memory for serialized message.");
  }

  // Copy the string data into the serialized message buffer
  memcpy(raw_serialized_msg.buffer, buffer.c_str(), raw_serialized_msg.buffer_length);

  serial_msg = rclcpp::SerializedMessage(raw_serialized_msg);
}

void serializedMsgToString(const rclcpp::SerializedMessage & serial_msg, std::string & buffer)
{
  buffer = std::string(
    reinterpret_cast<const char *>(serial_msg.get_rcl_serialized_message().buffer),
    serial_msg.get_rcl_serialized_message().buffer_length);
}

void stringToPointCloud2(const std::string & buffer, sensor_msgs::msg::PointCloud2 & cloud)
{
  // serialize the pointcloud2 msg and use stringToSerialziedMsg
  auto serialized_msg_ptr = std::make_shared<rclcpp::SerializedMessage>();
  stringToSerializedMsg(buffer, *serialized_msg_ptr);

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> deserializer;
  deserializer.deserialize_message(serialized_msg_ptr.get(), &(cloud));
}

void pointCloud2ToString(const sensor_msgs::msg::PointCloud2 & cloud, std::string & buffer)
{
  // serialize the pointcloud2 msg and use stringToSerialziedMsg
  auto serialized_msg_ptr = std::make_shared<rclcpp::SerializedMessage>();
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
  serializer.serialize_message(&(cloud), serialized_msg_ptr.get());
  serializedMsgToString(*serialized_msg_ptr, buffer);
}
}  // namespace point_cloud_transport

// Bindings for STL vector of strings
PYBIND11_MAKE_OPAQUE(std::vector<std::string>);

// Bindings for the PointCloudCodec class
PYBIND11_MODULE(_codec, m)
{
  py::bind_vector<std::vector<std::string>>(m, "VectorString");

  py::class_<PointCloudCodec>(m, "PointCloudCodec")
  .def(pybind11::init())
  .def("getLoadableTransports", &PointCloudCodec::getLoadableTransports)
  .def("getTopicsToPublish", &PointCloudCodec::getTopicsToPublish)
  .def(
    "getTopicToSubscribe",
    [](PointCloudCodec & codec, const std::string & baseTopic, const std::string & transport)
    {
      std::string topic;
      std::string name;
      std::string dataType;
      codec.getTopicToSubscribe(baseTopic, transport, topic, name, dataType);
      return std::make_tuple(topic, name, dataType);
    })
  .def(
    "encode",
    [](PointCloudCodec & codec, const std::string & transport_name,
    const std::string & pointcloud_buffer)
    {
      sensor_msgs::msg::PointCloud2 msg;
      stringToPointCloud2(pointcloud_buffer, msg);

      rclcpp::SerializedMessage serialized_msg;
      bool success = codec.encode(transport_name, msg, serialized_msg);

      std::string serialized_buffer;
      if (success) {
        serializedMsgToString(serialized_msg, serialized_buffer);
      }
      return serialized_buffer;
    }
  )
  .def(
    "decode",
    [](PointCloudCodec & codec, const std::string & transport_name,
    const std::string & serialized_buffer)
    {
      rclcpp::SerializedMessage serialized_msg;
      stringToSerializedMsg(serialized_buffer, serialized_msg);

      sensor_msgs::msg::PointCloud2 msg;
      bool success = codec.decode(transport_name, serialized_msg, msg);

      std::string buffer;
      if (success) {
        pointCloud2ToString(msg, buffer);
      }
      return buffer;
    }
  );
}
