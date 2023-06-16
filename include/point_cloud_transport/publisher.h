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

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/loader_fwds.h>
#include <point_cloud_transport/single_subscriber_publisher.h>

namespace point_cloud_transport
{

class Publisher
{
public:
  //! Constructor
  Publisher();

  //! get total number of subscribers to all advertised topics.
  uint32_t getNumSubscribers() const;

  //! get base topic of this Publisher
  std::string getTopic() const;

  //! Publish a point cloud on the topics associated with this Publisher.
  void publish(const sensor_msgs::PointCloud2& message) const;

  //! Publish a point cloud on the topics associated with this Publisher.
  void publish(const sensor_msgs::PointCloud2ConstPtr& message) const;

  //! Shutdown the advertisements associated with this Publisher.
  void shutdown();

  operator void*() const;

  bool operator<(const point_cloud_transport::Publisher& rhs) const
  {
    return impl_ < rhs.impl_;
  }

  bool operator!=(const point_cloud_transport::Publisher& rhs) const
  {
    return impl_ != rhs.impl_;
  }

  bool operator==(const point_cloud_transport::Publisher& rhs) const
  {
    return impl_ == rhs.impl_;
  }

private:
  Publisher(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
            const point_cloud_transport::SubscriberStatusCallback& connect_cb,
            const point_cloud_transport::SubscriberStatusCallback& disconnect_cb,
            const ros::VoidPtr& tracked_object, bool latch,
            const point_cloud_transport::PubLoaderPtr& loader);

  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;

  static void weakSubscriberCb(const ImplWPtr& impl_wptr,
                               const point_cloud_transport::SingleSubscriberPublisher& plugin_pub,
                               const point_cloud_transport::SubscriberStatusCallback& user_cb);

  point_cloud_transport::SubscriberStatusCallback rebindCB(
      const point_cloud_transport::SubscriberStatusCallback& user_cb);

  friend class PointCloudTransport;
};

}