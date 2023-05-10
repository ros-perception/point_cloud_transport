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
#include <vector>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/publisher.h>
#include <point_cloud_transport/single_subscriber_publisher.h>
#include <point_cloud_transport/subscriber.h>
#include <point_cloud_transport/transport_hints.h>


namespace point_cloud_transport {

/**
* Advertise and subscribe to PointCloud2 topics.
*
* PointCloudTransport is analogous to ros::NodeHandle in that it contains advertise() and
* subscribe() functions for creating advertisements and subscriptions of PointCloud2 topics.
*/

class PointCloudTransport
{
public:
  //! Constructor
  explicit PointCloudTransport(const ros::NodeHandle& nh);

  //! Advertise a PointCloud2 topic, simple version.
  point_cloud_transport::Publisher advertise(const std::string& base_topic, uint32_t queue_size, bool latch = false);

  //! Advertise an PointCloud2 topic with subscriber status callbacks.
  point_cloud_transport::Publisher advertise(const std::string& base_topic, uint32_t queue_size,
                                             const point_cloud_transport::SubscriberStatusCallback& connect_cb,
                                             const point_cloud_transport::SubscriberStatusCallback& disconnect_cb = {},
                                             const ros::VoidPtr& tracked_object = {}, bool latch = false);

  //! Subscribe to a point cloud topic, version for arbitrary boost::function object.
  point_cloud_transport::Subscriber subscribe(
      const std::string& base_topic, uint32_t queue_size,
      const boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>& callback,
      const ros::VoidPtr& tracked_object = {},
      const point_cloud_transport::TransportHints& transport_hints = {});

  //! Subscribe to a point cloud topic, version for bare function.
  point_cloud_transport::Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                                              void(* fp)(const sensor_msgs::PointCloud2ConstPtr&),
                                              const point_cloud_transport::TransportHints& transport_hints = {})
  {
    return subscribe(base_topic, queue_size,
                     boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>(fp),
                     ros::VoidPtr(), transport_hints);
  }

  //! Subscribe to a point cloud topic, version for class member function with bare pointer.
  template<class T>
  point_cloud_transport::Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                                              void(T::*fp)(const sensor_msgs::PointCloud2ConstPtr&), T* obj,
                                              const point_cloud_transport::TransportHints& transport_hints = {})
  {
    return subscribe(base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
  }

  //! Subscribe to a point cloud topic, version for class member function with shared_ptr.
  template<class T>
  point_cloud_transport::Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                                              void(T::*fp)(const sensor_msgs::PointCloud2ConstPtr&),
                                              const boost::shared_ptr<T>& obj,
                                              const point_cloud_transport::TransportHints& transport_hints = {})
  {
    return subscribe(base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
  }

  //! Returns the names of all transports declared in the system. Declared
  //! transports are not necessarily built or loadable.
  std::vector<std::string> getDeclaredTransports() const;

  //! Returns the names of all transports that are loadable in the system.
  std::vector<std::string> getLoadableTransports() const;

private:
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;
};

}
