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

#ifndef POINT_CLOUD_TRANSPORT__SUBSCRIBER_FILTER_HPP_
#define POINT_CLOUD_TRANSPORT__SUBSCRIBER_FILTER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/transport_hints.hpp>
#include "point_cloud_transport/visibility_control.hpp"
#include "point_cloud_transport/exception.hpp"

namespace point_cloud_transport
{

///
/// PointCloud2 subscription filter.
///
/// This class wraps Subscriber as a "filter" compatible with the message_filters
/// package. It acts as a highest-level filter, simply passing messages from a point cloud
/// transport subscription through to the filters which have connected to it.
///
/// When this object is destroyed it will unsubscribe from the ROS subscription.
///
/// SubscriberFilter has no input connection.
///
/// The output connection for the SubscriberFilter object is the same signature as for rclcpp
/// subscription callbacks.
///
class SubscriberFilter
  : public message_filters::SubscriberBase,
  public message_filters::SimpleFilter<sensor_msgs::msg::PointCloud2>
{
public:
  ///
  /// \brief Constructor
  /// \param node The rclcpp node to use to subscribe.
  /// \param base_topic The topic to subscribe to.
  /// \param queue_size The subscription queue size
  /// \param transport The transport hint to pass along
  ///
  [[deprecated("Use SubscriberFilter(rclcpp::node_interfaces...) instead")]]
  POINT_CLOUD_TRANSPORT_PUBLIC
  SubscriberFilter(
    std::shared_ptr<rclcpp::Node> node, const std::string & base_topic,
    const std::string & transport)
  : SubscriberFilter(
      *node,
      base_topic, transport)
  {
  }

  POINT_CLOUD_TRANSPORT_PUBLIC
  SubscriberFilter(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    const std::string & transport,
    rclcpp::QoS custom_qos = rclcpp::SystemDefaultsQoS(),
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  //! Empty constructor, use subscribe() to subscribe to a topic
  POINT_CLOUD_TRANSPORT_PUBLIC
  SubscriberFilter();

  POINT_CLOUD_TRANSPORT_PUBLIC
  ~SubscriberFilter();
  ///
  /// \brief Subscribe to a topic. If this Subscriber is already subscribed to a topic,
  /// this function will first unsubscribe.
  /// \param node The rclcpp Node to use to subscribe.
  /// \param base_topic The topic to subscribe to.
  /// \param transport The transport hint to pass along
  /// \param custom_qos Custom quality of service
  /// \param options Subscriber options
  ///
  [[deprecated("Use subscribe(rclcpp::node_interfaces...) instead")]]
  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    const std::string & transport,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  [[deprecated("Use subscribe(rclcpp::node_interfaces..., rclcpp::QoS, ...) instead")]]
  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe(
    std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
    const std::string & base_topic,
    const std::string & transport,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    const std::string & transport,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  //! Re-subscribe to a topic.
  // Only works if this subscriber has previously been subscribed to a topic.
  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe() override;

  //! Force immediate unsubscription of this subscriber from its topic
  POINT_CLOUD_TRANSPORT_PUBLIC
  void unsubscribe() override;

  POINT_CLOUD_TRANSPORT_PUBLIC
  std::string getTopic() const;

  //! Returns the number of publishers this subscriber is connected to.
  POINT_CLOUD_TRANSPORT_PUBLIC
  uint32_t getNumPublishers() const;

  //! Returns the name of the transport being used.
  POINT_CLOUD_TRANSPORT_PUBLIC
  std::string getTransport() const;

  //! Returns the internal point_cloud_transport::Subscriber object.
  POINT_CLOUD_TRANSPORT_PUBLIC
  const Subscriber & getSubscriber() const;

private:
  //! Must override parent message_filters::SubscriberBase method
  // where RequiredInterfaces are just <NodeParametersInterface, NodeTopicsInterface>
  void subscribe(
    rclcpp::node_interfaces::NodeInterfaces<
      NodeParametersInterface,
      NodeTopicsInterface>,
    const std::string &,
    const rclcpp::QoS &) override
  {
    throw point_cloud_transport::Exception("Not implemented");
  }

  //! Must override parent message_filters::SubscriberBase method
  // where RequiredInterfaces are just <NodeParametersInterface, NodeTopicsInterface>
  void subscribe(
    rclcpp::node_interfaces::NodeInterfaces<
      NodeParametersInterface,
      NodeTopicsInterface>,
    const std::string &,
    const rclcpp::QoS &,
    rclcpp::SubscriptionOptions) override
  {
    throw point_cloud_transport::Exception("Not implemented");
  }

  void cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m)
  {
    signalMessage(m);
  }

  Subscriber sub_;
  rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces_;

  std::string topic_;
  std::string transport_;
  rclcpp::QoS qos_ = rclcpp::SystemDefaultsQoS();
  rclcpp::SubscriptionOptions options_;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__SUBSCRIBER_FILTER_HPP_
