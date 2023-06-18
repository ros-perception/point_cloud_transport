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
#include <unordered_map>
#include <vector>

#include "rclcpp/node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/single_subscriber_publisher.hpp>
#include <point_cloud_transport/subscriber.hpp>
#include <point_cloud_transport/transport_hints.hpp>


namespace point_cloud_transport {

/**
* Advertise and subscribe to PointCloud2 topics.
*
* PointCloudTransport is analogous to ros::NodeHandle in that it contains advertise() and
* subscribe() functions for creating advertisements and subscriptions of PointCloud2 topics.
*/

class PointCloudTransportLoader
{
public:
  //! Constructor
  PointCloudTransportLoader();

  //! Destructor
  virtual ~PointCloudTransportLoader();

  //! Returns the names of all transports declared in the system. Declared
  //! transports are not necessarily built or loadable.
  std::vector<std::string> getDeclaredTransports() const;

  //! Returns the names of all transports that are loadable in the system (keys are lookup names, values are names).
  std::unordered_map<std::string, std::string> getLoadableTransports() const;

  //! The loader that can load publisher plugins.
  PubLoaderPtr getPublisherLoader() const;

  //! The loader that can load subscriber plugins.
  SubLoaderPtr getSubscriberLoader() const;

protected:
  point_cloud_transport::PubLoaderPtr pub_loader_;
  point_cloud_transport::SubLoaderPtr sub_loader_;
};


/**
* Advertise and subscribe to PointCloud2 topics.
*
* PointCloudTransport is analogous to ros::NodeHandle in that it contains advertise() and
* subscribe() functions for creating advertisements and subscriptions of PointCloud2 topics.
*/

class PointCloudTransport : public PointCloudTransportLoader
{
  using VoidPtr = std::shared_ptr<void>;

public:
  //! Constructor
  explicit PointCloudTransport(rclcpp::Node::SharedPtr node);

  ~PointCloudTransport() override = default;

  std::string getTransportOrDefault(const TransportHints * transport_hints)
  {
    std::string ret;
    if (nullptr == transport_hints) {
      TransportHints th(node_.get());
      ret = th.getTransport();
    } else {
      ret = transport_hints->getTransport();
    }
    return ret;
  }

  //! Advertise a PointCloud2 topic, simple version.
  Publisher advertise(
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos)
  {
    return Publisher(node_.get(), base_topic, pub_loader_, custom_qos);
  }

  //! Advertise an PointCloud2 topic with subscriber status callbacks.
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // point_cloud_transport::Publisher advertise(const std::string& base_topic, uint32_t queue_size,
  //                                            const point_cloud_transport::SubscriberStatusCallback& connect_cb,
  //                                            const point_cloud_transport::SubscriberStatusCallback& disconnect_cb = {},
  //                                            const ros::VoidPtr& tracked_object = {}, bool latch = false);

  //! Subscribe to a point cloud topic, version for arbitrary std::function object.
  point_cloud_transport::Subscriber subscribe(
      const std::string &base_topic, uint32_t queue_size,
      const std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)> &callback,
      const VoidPtr &tracked_object = {},
      const point_cloud_transport::TransportHints *transport_hints = nullptr)
  {
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = queue_size;
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions();
    return Subscriber(node_.get(), base_topic, callback, sub_loader_, getTransportOrDefault(transport_hints), custom_qos, options);
  }

  //! Subscribe to a point cloud topic, version for bare function.
  point_cloud_transport::Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                                              void(* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&),
                                              const point_cloud_transport::TransportHints* transport_hints = nullptr)
  {
    return subscribe(base_topic, queue_size,
                     std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&)>(fp),
                     VoidPtr(), transport_hints);
  }

  //! Subscribe to a point cloud topic, version for class member function with bare pointer.
  template<class T>
  point_cloud_transport::Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                                              void(T::*fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&), T* obj,
                                              const point_cloud_transport::TransportHints* transport_hints = nullptr,
                                              bool allow_concurrent_callbacks = false)
  {
    return subscribe(base_topic, queue_size, std::bind(fp, obj.get(), std::placeholders::_1), VoidPtr(), transport_hints);
  }

  //! Subscribe to a point cloud topic, version for class member function with shared_ptr.
  template<class T>
  point_cloud_transport::Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                                              void(T::*fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr&),
                                              const std::shared_ptr<T>& obj,
                                              const point_cloud_transport::TransportHints* transport_hints = nullptr)
  {
    return subscribe(base_topic, queue_size, std::bind(fp, obj.get(), std::placeholders::_1), obj, transport_hints);
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}

// TODO: May need these?
// extern "C" void pointCloudTransportGetLoadableTransports(
//     cras::allocator_t transportAllocator, cras::allocator_t nameAllocator);

// extern "C" void pointCloudTransportGetTopicsToPublish(
//     const char* baseTopic, cras::allocator_t transportAllocator, cras::allocator_t nameAllocator,
//     cras::allocator_t topicAllocator, cras::allocator_t dataTypeAllocator, cras::allocator_t configTypeAllocator);

// extern "C" void pointCloudTransportGetTopicToSubscribe(
//     const char* baseTopic, const char* transport, cras::allocator_t nameAllocator, cras::allocator_t topicAllocator,
//     cras::allocator_t dataTypeAllocator, cras::allocator_t configTypeAllocator);
