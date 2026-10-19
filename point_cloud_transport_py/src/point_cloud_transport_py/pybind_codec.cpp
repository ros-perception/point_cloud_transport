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

#include <string>
#include <tuple>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <point_cloud_transport/point_cloud_codec.hpp>

#include "borrowed_serialized_message.hpp"

namespace py = pybind11;
using namespace point_cloud_transport;  // NOLINT
using point_cloud_transport_python::BorrowedSerializedMessage;
using point_cloud_transport_python::serialized_message_to_bytes;

// Bindings for STL vector of strings
PYBIND11_MAKE_OPAQUE(std::vector<std::string>)

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
    const py::buffer & pointcloud_buffer)
    {
      // Deserialize straight from the Python memory, without intermediate copies.
      sensor_msgs::msg::PointCloud2 msg;
      {
        BorrowedSerializedMessage borrowed(pointcloud_buffer);
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> deserializer;
        deserializer.deserialize_message(&borrowed.get(), &msg);
      }

      rclcpp::SerializedMessage serialized_msg;
      if (!codec.encode(transport_name, msg, serialized_msg)) {
        return py::bytes();
      }
      return serialized_message_to_bytes(serialized_msg);
    },
    "Encode a serialized sensor_msgs/PointCloud2 (bytes) with the given transport. "
    "Returns the serialized transport message, or empty bytes on failure."
  )
  .def(
    "decode",
    [](PointCloudCodec & codec, const std::string & transport_name,
    const py::buffer & serialized_buffer)
    {
      sensor_msgs::msg::PointCloud2 msg;
      {
        BorrowedSerializedMessage borrowed(serialized_buffer);
        if (!codec.decode(transport_name, borrowed.get(), msg)) {
          return py::bytes();
        }
      }

      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
      serializer.serialize_message(&msg, &serialized_msg);
      return serialized_message_to_bytes(serialized_msg);
    },
    "Decode a serialized transport message (bytes) with the given transport. "
    "Returns the serialized sensor_msgs/PointCloud2, or empty bytes on failure."
  );
}
