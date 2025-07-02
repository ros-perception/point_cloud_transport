// Copyright (c) 2023, Czech Technical University in Prague
// Copyright (c) 2019, paplhjak
// Copyright (c) 2009, Willow Garage, Inc.
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
//    * Neither the name of the copyright holder nor the names of its
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
//

#ifndef POINT_CLOUD_TRANSPORT__PUBLISHER_HPP_
#define POINT_CLOUD_TRANSPORT__PUBLISHER_HPP_

#include <memory>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_interfaces.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/loader_fwds.hpp>
#include <point_cloud_transport/single_subscriber_publisher.hpp>

#include "point_cloud_transport/visibility_control.hpp"

namespace point_cloud_transport
{

class Publisher
{
public:
  //! Constructor
  POINT_CLOUD_TRANSPORT_PUBLIC
  Publisher() = default;

  [[deprecated("Use Publisher(rclcpp::node_interfaces...) instead")]]
  POINT_CLOUD_TRANSPORT_PUBLIC
  Publisher(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    PubLoaderPtr loader,
    rmw_qos_profile_t custom_qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions())
  : Publisher(
      *node,
      base_topic,
      loader,
      rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos),
      options)
  {}

  [[deprecated("Use Publisher(rclcpp::node_interfaces, ..., rclcpp::QoS custom_qos,...) instead")]]
  POINT_CLOUD_TRANSPORT_PUBLIC
  Publisher(
    std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
    const std::string & base_topic,
    PubLoaderPtr loader,
    rmw_qos_profile_t custom_qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions());

  POINT_CLOUD_TRANSPORT_PUBLIC
  Publisher(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    PubLoaderPtr loader,
    rclcpp::QoS custom_qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions());

  //! get total number of subscribers to all advertised topics.
  POINT_CLOUD_TRANSPORT_PUBLIC
  uint32_t getNumSubscribers() const;

  //! get base topic of this Publisher
  POINT_CLOUD_TRANSPORT_PUBLIC
  std::string getTopic() const;

  //! Publish a point cloud on the topics associated with this Publisher.
  POINT_CLOUD_TRANSPORT_PUBLIC
  void publish(const sensor_msgs::msg::PointCloud2 & message) const;

  //! Publish a point cloud on the topics associated with this Publisher.
  POINT_CLOUD_TRANSPORT_PUBLIC
  void publish(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message) const;

  //! Shutdown the advertisements associated with this Publisher.
  POINT_CLOUD_TRANSPORT_PUBLIC
  void shutdown();

  POINT_CLOUD_TRANSPORT_PUBLIC
  operator void *() const;

  POINT_CLOUD_TRANSPORT_PUBLIC
  bool operator<(const point_cloud_transport::Publisher & rhs) const
  {
    return impl_ < rhs.impl_;
  }

  POINT_CLOUD_TRANSPORT_PUBLIC
  bool operator!=(const point_cloud_transport::Publisher & rhs) const
  {
    return impl_ != rhs.impl_;
  }

  POINT_CLOUD_TRANSPORT_PUBLIC
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
