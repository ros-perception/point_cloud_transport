// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "point_cloud_transport/subscriber_filter.hpp"

#include <functional>
#include <memory>
#include <string>

namespace point_cloud_transport
{
SubscriberFilter::SubscriberFilter(
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
  const std::string & base_topic,
  const std::string & transport,
  rclcpp::QoS custom_qos,
  rclcpp::SubscriptionOptions options)
{
  subscribe(node_interfaces, base_topic, transport, custom_qos, options);
}

SubscriberFilter::SubscriberFilter()
{
}

SubscriberFilter::~SubscriberFilter()
{
  unsubscribe();
}

void SubscriberFilter::subscribe(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & base_topic,
  const std::string & transport,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  subscribe(*node, base_topic, transport,
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos), options);
}

void SubscriberFilter::subscribe(
  std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
  const std::string & base_topic,
  const std::string & transport,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
{
  subscribe(*node_interfaces, base_topic, transport,
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos), options);
}

void SubscriberFilter::subscribe(
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
  const std::string & base_topic,
  const std::string & transport,
  rclcpp::QoS custom_qos,
  rclcpp::SubscriptionOptions options)
{
  unsubscribe();
  sub_ = point_cloud_transport::create_subscription(
    node_interfaces, base_topic,
    std::bind(&SubscriberFilter::cb, this, std::placeholders::_1),
    transport, custom_qos, options);
  node_interfaces_ = node_interfaces;
  topic_ = base_topic;
  transport_ = transport;
  qos_ = custom_qos;
  options_ = options;
}

void SubscriberFilter::subscribe()
{
  unsubscribe();
  if (!topic_.empty()) {
    sub_ = point_cloud_transport::create_subscription(
      node_interfaces_, topic_,
      std::bind(&SubscriberFilter::cb, this, std::placeholders::_1),
      transport_, qos_, options_);
  } else {
    RCLCPP_ERROR(
      node_interfaces_.get_node_logging_interface()->get_logger(),
      "Cannot re-subscribe: the subscriber filter must be initialized first."
    );
  }
}

void SubscriberFilter::unsubscribe()
{
  sub_.shutdown();
}

std::string SubscriberFilter::getTopic() const
{
  return this->topic_;
}

uint32_t SubscriberFilter::getNumPublishers() const
{
  return sub_.getNumPublishers();
}

std::string SubscriberFilter::getTransport() const
{
  return this->transport_;
}

const Subscriber & SubscriberFilter::getSubscriber() const
{
  return sub_;
}
}  // namespace point_cloud_transport
