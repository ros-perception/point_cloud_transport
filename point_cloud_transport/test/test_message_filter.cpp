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

#include <gtest/gtest.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "point_cloud_transport/point_cloud_transport.hpp"
#include "point_cloud_transport/subscriber_filter.hpp"

class TestSubscriber : public ::testing::Test
{
protected:
  void SetUp()
  {
    node_ = rclcpp::Node::make_shared("test_subscriber");
  }

  rclcpp::Node::SharedPtr node_;
};

void callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg1,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg2)
{
  (void) msg1;
  (void) msg2;
}

TEST_F(TestSubscriber, create_and_release_filter)
{
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>
    ApproximateTimePolicy;

  point_cloud_transport::SubscriberFilter pcl_sub1(node_, "pointcloud1", "raw");
  point_cloud_transport::SubscriberFilter pcl_sub2(node_, "pointcloud2", "raw");

  auto sync = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(
    ApproximateTimePolicy(
      10), pcl_sub1, pcl_sub2);
  sync->registerCallback(std::bind(callback, std::placeholders::_1, std::placeholders::_2));

  pcl_sub1.unsubscribe();
  pcl_sub2.unsubscribe();
  sync.reset();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
