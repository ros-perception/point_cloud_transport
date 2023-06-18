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

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pluginlib/class_loader.hpp"

#include <point_cloud_transport/loader_fwds.hpp>
#include <point_cloud_transport/transport_hints.hpp>

namespace point_cloud_transport {

/**
 * Manages a subscription callback on a specific topic that can be interpreted
 * as a PointCloud2 topic.
 *
 * Subscriber is the client-side counterpart to Publisher. By loading the
 * appropriate plugin, it can subscribe to a base point cloud topic using any available
 * transport. The complexity of what transport is actually used is hidden from the user,
 * who sees only a normal PointCloud2 callback.
 *
 * A Subscriber should always be created through a call to PointCloudTransport::subscribe(),
 * or copied from one that was.
 * Once all copies of a specific Subscriber go out of scope, the subscription callback
 * associated with that handle will stop being called. Once all Subscriber for a given
 * topic go out of scope the topic will be unsubscribed.
 */
class Subscriber
{
public:
  typedef std::function<void (const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)> Callback;

  Subscriber() = default;

  Subscriber(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    SubLoaderPtr loader,
    const std::string & transport,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  /**
   * Returns the base point cloud topic.
   *
   * The Subscriber may actually be subscribed to some transport-specific topic that
   * differs from the base topic.
   */
  std::string getTopic() const;

  /**
   * Returns the number of publishers this subscriber is connected to.
   */
  uint32_t getNumPublishers() const;

  /**
   * Returns the name of the transport being used.
   */
  std::string getTransport() const;

  /**
   * Unsubscribe the callback associated with this Subscriber.
   */
  void shutdown();

  operator void*() const;

  bool operator<(const point_cloud_transport::Subscriber& rhs) const
  {
    return impl_ < rhs.impl_;
  }

  bool operator!=(const point_cloud_transport::Subscriber& rhs) const
  {
    return impl_ != rhs.impl_;
  }

  bool operator==(const point_cloud_transport::Subscriber& rhs) const
  {
    return impl_ == rhs.impl_;
  }

private:

  struct Impl;
  std::shared_ptr<Impl> impl_;
  friend class PointCloudTransport;
};

}
