/*
 * Copyright (c) 2023, Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POINT_CLOUD_TRANSPORT__PUBLISHER_HPP_
#define POINT_CLOUD_TRANSPORT__PUBLISHER_HPP_

#include <string>
#include <memory>

#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pluginlib/class_loader.hpp"

#include <point_cloud_transport/loader_fwds.hpp>
#include <point_cloud_transport/single_subscriber_publisher.hpp>

namespace point_cloud_transport
{

class Publisher
{
public:
  //! Constructor
  Publisher() = default;

  Publisher(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    PubLoaderPtr loader,
    rmw_qos_profile_t custom_qos,
    rclcpp::PublisherOptions options = rclcpp::PublisherOptions());

  //! get total number of subscribers to all advertised topics.
  uint32_t getNumSubscribers() const;

  //! get base topic of this Publisher
  std::string getTopic() const;

  //! Publish a point cloud on the topics associated with this Publisher.
  void publish(const sensor_msgs::msg::PointCloud2 & message) const;

  //! Publish a point cloud on the topics associated with this Publisher.
  void publish(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message) const;

  //! Shutdown the advertisements associated with this Publisher.
  void shutdown();

  operator void *() const;

  bool operator<(const point_cloud_transport::Publisher & rhs) const
  {
    return impl_ < rhs.impl_;
  }

  bool operator!=(const point_cloud_transport::Publisher & rhs) const
  {
    return impl_ != rhs.impl_;
  }

  bool operator==(const point_cloud_transport::Publisher & rhs) const
  {
    return impl_ == rhs.impl_;
  }

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  friend class PointCloudTransport;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__PUBLISHER_HPP_
