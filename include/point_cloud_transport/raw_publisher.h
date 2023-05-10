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

#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/simple_publisher_plugin.h>

namespace point_cloud_transport
{

//! RawPublisher is a simple wrapper for ros::Publisher,
//! publishing unaltered PointCloud2 messages on the base topic.
class RawPublisher : public point_cloud_transport::SimplePublisherPlugin<sensor_msgs::PointCloud2>
{
public:
  std::string getTransportName() const override
  {
    return "raw";
  }

  // Override the default implementation because publishing the message pointer allows
  // the no-copy intraprocess optimization.
  void publish(const sensor_msgs::PointCloud2ConstPtr& message) const override
  {
    getPublisher().publish(message);
  }

  // Override the default implementation to not copy data to a sensor_msgs::PointCloud2 first
  void publish(const sensor_msgs::PointCloud2& message, const uint8_t* data) const override;

protected:
  void publish(const sensor_msgs::PointCloud2& message, const PublishFn& publish_fn) const override
  {
    publish_fn(message);
  }

  std::string getTopicToAdvertise(const std::string& base_topic) const override
  {
    return base_topic;
  }
};

}
