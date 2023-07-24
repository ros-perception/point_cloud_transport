// Copyright (c) 2023 John D'Angelo
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
//    * Neither the name of the Willow Garage nor the names of its
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

#include <functional>
#include <string>
#include <memory>

#include "gtest/gtest.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "point_cloud_transport/point_cloud_codec.hpp"

class TestCodec : public ::testing::Test
{
public:
  // dont do this in real code
  point_cloud_transport::PointCloudCodec codec;
};

// utility function for verifying the raw encode/decode process works
// (doesnt really make sense to use this with a lossy compression algo though, so its only
// applicable to test raw or zlib)
bool arePointCloudsEqual(
  const sensor_msgs::msg::PointCloud2 & msg1,
  const sensor_msgs::msg::PointCloud2 & msg2)
{
  if (msg1.width != msg2.width || msg1.height != msg2.height) {
    return false;
  }

  // Compare fields
  if (msg1.fields.size() != msg2.fields.size()) {
    return false;
  }

  for (size_t i = 0; i < msg1.fields.size(); ++i) {
    if (msg1.fields[i].name != msg2.fields[i].name ||
      msg1.fields[i].offset != msg2.fields[i].offset ||
      msg1.fields[i].datatype != msg2.fields[i].datatype ||
      msg1.fields[i].count != msg2.fields[i].count)
    {
      return false;
    }
  }

  // Compare data
  for (const std::string field_name : {"r", "g", "b"}) {
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter1(msg1, field_name);
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter2(msg2, field_name);

    for (; iter1 != iter1.end() && iter2 != iter2.end(); ++iter1, ++iter2) {
      if (*iter1 != *iter2) {
        return false;
      }
    }
  }

  for (const std::string field_name : {"x", "y", "z"}) {
    sensor_msgs::PointCloud2ConstIterator<float> iter1(msg1, field_name);
    sensor_msgs::PointCloud2ConstIterator<float> iter2(msg2, field_name);

    for (; iter1 != iter1.end() && iter2 != iter2.end(); ++iter1, ++iter2) {
      if (*iter1 != *iter2) {
        return false;
      }
    }
  }

  return true;
}

TEST_F(TestCodec, listTransports) {
  std::vector<std::string> transports;
  std::vector<std::string> names;
  codec.getLoadableTransports(transports, names);
  for (size_t i = 0; i < transports.size(); ++i) {
    std::cout << "Transport: " << transports[i] << std::endl;
    std::cout << "Name: " << names[i] << std::endl;
  }
  EXPECT_TRUE(true);
}

TEST_F(TestCodec, listSubscribedTopics) {
  std::vector<std::string> transports;
  std::vector<std::string> names;
  codec.getLoadableTransports(transports, names);
  const std::string base_topic = "point_cloud";
  for (const auto & transport : transports) {
    std::string topic, name, dataType;
    codec.getTopicToSubscribe(base_topic, transport, topic, name, dataType);
    std::cout << "Transport: " << transport << std::endl;
    std::cout << "Topic: " << topic << std::endl;
    std::cout << "Name: " << name << std::endl;
    std::cout << "DataType: " << dataType << std::endl;
  }
  EXPECT_TRUE(true);
}

TEST_F(TestCodec, listPublishedTopisc) {
  std::vector<std::string> transports;
  std::vector<std::string> topics;
  std::vector<std::string> names;
  std::vector<std::string> dataTypes;

  const std::string base_topic = "point_cloud";
  codec.getTopicsToPublish(base_topic, transports, topics, names, dataTypes);
  for (size_t i = 0; i < transports.size(); ++i) {
    std::cout << "Transport: " << transports[i] << std::endl;
    std::cout << "Topic: " << topics[i] << std::endl;
    std::cout << "Name: " << names[i] << std::endl;
    std::cout << "DataType: " << dataTypes[i] << std::endl;
  }

  EXPECT_TRUE(true);
}

TEST_F(TestCodec, encodeMesssage) {
  auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  rclcpp::SerializedMessage serialized_msg;

  // populate the pointcloud with some random data

  // Set the PointCloud2 header
  msg->header.frame_id = "base_link";  // Replace with the appropriate frame ID

  // Set the PointCloud2 dimensions and fields
  msg->height = 1;
  msg->width = 4;
  msg->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  // Fill the PointCloud2 data
  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*msg, "b");

  // Sample data points
  float points[] = {1.0, 2.0, 3.0, 4.0};
  uint8_t colors[] = {255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0};

  for (int i = 0; i < 4; ++i) {
    *iter_x = points[i];
    *iter_y = points[i] * 2;
    *iter_z = points[i] * 3;

    *iter_r = colors[i * 3];
    *iter_g = colors[i * 3 + 1];
    *iter_b = colors[i * 3 + 2];

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }

  // encode that data
  std::cout << "Encoding message" << std::endl;
  bool success = codec.encode("raw", *msg, serialized_msg);
  EXPECT_TRUE(success);

  std::cout << "Decoding message" << std::endl;
  auto new_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  success = codec.decode("raw", serialized_msg, *new_msg);
  EXPECT_TRUE(success);

  // check that the data is the same
  std::cout << "Check for equality" << std::endl;
  success = arePointCloudsEqual(*msg, *new_msg);
  EXPECT_TRUE(success);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
