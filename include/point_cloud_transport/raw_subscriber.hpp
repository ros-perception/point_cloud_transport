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

#pragma once

#include <string>
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/simple_subscriber_plugin.hpp>

namespace point_cloud_transport
{

/**
 * The default SubscriberPlugin.
 *
 * RawSubscriber is a simple wrapper for ros::Subscriber which listens for PointCloud2 messages
 * and passes them through to the callback.
 */
class RawSubscriber : public point_cloud_transport::SimpleSubscriberPlugin<sensor_msgs::msg::PointCloud2>
{
public:
  virtual ~RawSubscriber() {}


  SubscriberPlugin::DecodeResult decodeTyped(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& compressed) const
  {
    return compressed;
  }

  SubscriberPlugin::DecodeResult decodeTyped(const sensor_msgs::msg::PointCloud2& compressed) const
  {
    auto compressedPtr = std::make_shared<const sensor_msgs::msg::PointCloud2>(compressed);
    return this->decodeTyped(compressedPtr);
  }

  std::string getTransportName() const override;

protected:
  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message, const Callback& user_cb) override;

  std::string getTopicToSubscribe(const std::string& base_topic) const override;

  using SubscriberPlugin::subscribeImpl;

  void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options) override
  {
    this->subscribeImplWithOptions(node, base_topic, callback, custom_qos, options);
  }
};

}
