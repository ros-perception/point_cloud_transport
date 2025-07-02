// Copyright (c) 2023 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>

#include "point_cloud_transport/point_cloud_transport.hpp"

class TestPublisher : public ::testing::Test
{
protected:
  void SetUp()
  {
    node_ = rclcpp::Node::make_shared("test_publisher");
  }

  rclcpp::Node::SharedPtr node_;
};

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
TEST_F(TestPublisher, publisher)
{
  auto pub = point_cloud_transport::create_publisher(node_, "point_cloud");
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("point_cloud"), 1u);
  pub.shutdown();
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("point_cloud"), 0u);
  // coverage tests: invalid publisher should fail but not crash
  pub.publish(sensor_msgs::msg::PointCloud2());
  pub.publish(sensor_msgs::msg::PointCloud2::ConstSharedPtr());
}

TEST_F(TestPublisher, point_cloud_transport_publisher)
{
  point_cloud_transport::PointCloudTransport it(node_);
  auto pub = it.advertise("point_cloud", rmw_qos_profile_sensor_data);
}
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif

TEST_F(TestPublisher, publisher_ni_api)
{
  auto pub = point_cloud_transport::create_publisher(
    *node_,
    "point_cloud",
    rclcpp::SystemDefaultsQoS());
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("point_cloud"), 1u);
  pub.shutdown();
  EXPECT_EQ(node_->get_node_graph_interface()->count_publishers("point_cloud"), 0u);
  // coverage tests: invalid publisher should fail but not crash
  pub.publish(sensor_msgs::msg::PointCloud2());
  pub.publish(sensor_msgs::msg::PointCloud2::ConstSharedPtr());
}

TEST_F(TestPublisher, point_cloud_transport_publisher_ni_api)
{
  point_cloud_transport::PointCloudTransport it(*node_);
  auto pub = it.advertise("point_cloud", rclcpp::SystemDefaultsQoS());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
