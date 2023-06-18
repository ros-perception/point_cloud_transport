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
#include <string>

#include "rclcpp/macros.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace point_cloud_transport
{

//! Allows publication of a point cloud to a single subscriber. Only available inside subscriber connection callbacks.
class SingleSubscriberPublisher
{
public:
  typedef std::function<uint32_t()> GetNumSubscribersFn;
  typedef std::function<void(const sensor_msgs::msg::PointCloud2&)> PublishFn;

  SingleSubscriberPublisher(const SingleSubscriberPublisher &) = delete;
  SingleSubscriberPublisher & operator=(const SingleSubscriberPublisher &) = delete;

  SingleSubscriberPublisher(const std::string& caller_id, const std::string& topic,
                            const GetNumSubscribersFn& num_subscribers_fn,
                            const PublishFn& publish_fn);

  std::string getSubscriberName() const;

  std::string getTopic() const;

  uint32_t getNumSubscribers() const;

  void publish(const sensor_msgs::msg::PointCloud2& message) const;
  void publish(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message) const;

private:
  std::string caller_id_;
  std::string topic_;
  GetNumSubscribersFn num_subscribers_fn_;
  PublishFn publish_fn_;

  friend class Publisher;  // to get publish_fn_ directly
};

typedef std::function<void(const SingleSubscriberPublisher&)> SubscriberStatusCallback;

} // namespace point_cloud_transport
