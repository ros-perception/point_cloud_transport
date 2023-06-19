// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak .. 2009, Willow Garage, Inc.

/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
 * Copyright (c) 2009, Willow Garage, Inc.
 *
 *        All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *        modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *       AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *       IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *       DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *       FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *       DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *       SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *       OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *       OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber.hpp>
#include <point_cloud_transport/exception.hpp>

// TODO: Fix this
int main(int argc, char **argv)
{
  auto vargv = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (vargv.size() < 2)
  {
    printf(
        "Usage: %s in_transport in:=<in_base_topic> [out_transport] out:=<out_base_topic>\n",
        argv[0]);
    return 0;
  }

  auto node = rclcpp::Node::make_shared("point_cloud_republisher");

  std::string in_topic = rclcpp::expand_topic_or_service_name(
      "in",
      node->get_name(), node->get_namespace());
  std::string out_topic = rclcpp::expand_topic_or_service_name(
      "out",
      node->get_name(), node->get_namespace());

  std::string in_transport = vargv[1];

  point_cloud_transport::PointCloudTransport pct(node);

  if (vargv.size() < 3)
  {
    // Use all available transports for output
    auto pub = std::make_shared<point_cloud_transport::Publisher>(pct.advertise(out_topic, rmw_qos_profile_default));

    // Use Publisher::publish as the subscriber callback
    typedef void (point_cloud_transport::Publisher::*PublishMemFn)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&) const;
    PublishMemFn pub_mem_fn = &point_cloud_transport::Publisher::publish;

    const point_cloud_transport::TransportHints hint(in_transport);
    auto sub = pct.subscribe(
        in_topic, static_cast<uint32_t>(1),
        pub_mem_fn, pub, &hint);
    // spin the node
    rclcpp::spin(node);
  }
  else
  {
    // Use one specific transport for output
    std::string out_transport = vargv[2];

    // Load transport plugin
    typedef point_cloud_transport::PublisherPlugin Plugin;
    auto loader = pct.getPublisherLoader();
    std::string lookup_name = Plugin::getLookupName(out_transport);

    auto instance = loader->createUniqueInstance(lookup_name);
    // DO NOT use instance after this line
    std::shared_ptr<Plugin> pub = std::move(instance);
    pub->advertise(node.get(), out_topic);

    // Use PublisherPlugin::publish as the subscriber callback
    typedef void (point_cloud_transport::PublisherPlugin::*PublishMemFn)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&) const;
    PublishMemFn pub_mem_fn = &point_cloud_transport::PublisherPlugin::publish;

    const point_cloud_transport::TransportHints hint(in_transport);
    auto sub = pct.subscribe(
        in_topic, static_cast<uint32_t>(1),
        pub_mem_fn, pub, &hint);
    // spin the node
    rclcpp::spin(node);
  }

  return 0;
}
