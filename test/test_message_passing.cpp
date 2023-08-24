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

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "utils.hpp"

using namespace std::chrono_literals;

int total_pointclouds_received = 0;

class MessagePassingTesting : public ::testing::Test
{
public:
  sensor_msgs::msg::PointCloud2::UniquePtr generate_random_cloudpoint()
  {
    auto pointcloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    return pointcloud;
  }

protected:
  void SetUp()
  {
    node_ = rclcpp::Node::make_shared("test_message_passing");
    total_pointclouds_received = 0;
  }

  rclcpp::Node::SharedPtr node_;
};

void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  (void) msg;
  total_pointclouds_received++;
}

TEST_F(MessagePassingTesting, one_message_passing)
{
  const size_t max_retries = 3;
  const size_t max_loops = 200;
  const std::chrono::milliseconds sleep_per_loop = std::chrono::milliseconds(10);

  rclcpp::executors::SingleThreadedExecutor executor;

  auto pub = point_cloud_transport::create_publisher(node_, "pointcloud");
  auto sub =
    point_cloud_transport::create_subscription(
    node_, "pointcloud", pointcloudCallback,
    "raw");

  test_rclcpp::wait_for_subscriber(node_, sub.getTopic());

  ASSERT_EQ(0, total_pointclouds_received);
  ASSERT_EQ(1u, pub.getNumSubscribers());
  ASSERT_EQ(1u, sub.getNumPublishers());

  executor.spin_node_some(node_);
  ASSERT_EQ(0, total_pointclouds_received);

  size_t retry = 0;
  while (retry < max_retries && total_pointclouds_received == 0) {
    // generate random pointcloud and publish it
    pub.publish(generate_random_cloudpoint());

    executor.spin_node_some(node_);
    size_t loop = 0;
    while ((total_pointclouds_received != 1) && (loop++ < max_loops)) {
      std::this_thread::sleep_for(sleep_per_loop);
      executor.spin_node_some(node_);
    }
  }

  ASSERT_EQ(1, total_pointclouds_received);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
