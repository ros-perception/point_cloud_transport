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

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/utilities.hpp>

#include "point_cloud_transport/transport_hints.hpp"

TEST(TransportHints, default_transport_is_raw) {
  auto node = rclcpp::Node::make_shared("test_transport_hints");
  point_cloud_transport::TransportHints hints(*node);
  EXPECT_EQ("raw", hints.getTransport());
}

TEST(TransportHints, custom_default_without_override) {
  auto node = rclcpp::Node::make_shared("test_transport_hints");
  point_cloud_transport::TransportHints hints(*node, "draco");
  EXPECT_EQ("draco", hints.getTransport());
}

TEST(TransportHints, parameter_override_wins) {
  auto node = rclcpp::Node::make_shared(
    "test_transport_hints", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("point_cloud_transport", "zlib")}));
  point_cloud_transport::TransportHints hints(*node, "draco");
  EXPECT_EQ("zlib", hints.getTransport());
}

TEST(TransportHints, custom_parameter_name) {
  auto node = rclcpp::Node::make_shared(
    "test_transport_hints", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("custom_transport", "zlib"),
        rclcpp::Parameter("point_cloud_transport", "raw")}));
  point_cloud_transport::TransportHints hints(*node, "draco", "custom_transport");
  EXPECT_EQ("zlib", hints.getTransport());
}

TEST(TransportHints, empty_override_uses_default) {
  auto node = rclcpp::Node::make_shared(
    "test_transport_hints", rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("point_cloud_transport", "")}));
  point_cloud_transport::TransportHints hints(*node, "draco");
  EXPECT_EQ("draco", hints.getTransport());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
