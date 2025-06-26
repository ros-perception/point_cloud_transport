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

#ifndef POINT_CLOUD_TRANSPORT__FULL_SUBSCRIBER_FILTER_HPP_
#define POINT_CLOUD_TRANSPORT__FULL_SUBSCRIBER_FILTER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/subscriber_filter.hpp>
#include <point_cloud_transport/transport_hints.hpp>
#include "point_cloud_transport/visibility_control.hpp"

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
/// FullSubscriberFilter has no input connection.
///
/// The output connection for the FullSubscriberFilter object is the same signature as for rclcpp
/// subscription callbacks.
///
class FullSubscriberFilter
  : public message_filters::SubscriberBase,
  public SubscriberFilter
{
public:
  POINT_CLOUD_TRANSPORT_PUBLIC
  FullSubscriberFilter(
    std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
    const std::string & base_topic,
    const std::string & transport,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  template<typename NodeT = rclcpp::Node::SharedPtr>
  POINT_CLOUD_TRANSPORT_PUBLIC
  FullSubscriberFilter(
    NodeT node,
    const std::string & base_topic,
    const std::string & transport,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    auto node_interfaces = std::make_shared<rclcpp::node_interfaces::NodeInterfaces<
          rclcpp::node_interfaces::NodeBaseInterface,
          rclcpp::node_interfaces::NodeParametersInterface,
          rclcpp::node_interfaces::NodeTopicsInterface,
          rclcpp::node_interfaces::NodeLoggingInterface>>(*node);
    subscribe(node_interfaces, base_topic, transport, custom_qos, options);
  }

  //! Empty constructor, use subscribe() to subscribe to a topic
  POINT_CLOUD_TRANSPORT_PUBLIC
  FullSubscriberFilter() {}

  //! Empty constructor, parent class already calls unsubscribe()
  POINT_CLOUD_TRANSPORT_PUBLIC
  ~FullSubscriberFilter() {}

  ///
  /// \brief Subscribe to a topic. If this Subscriber is already subscribed to a topic,
  /// this function will first unsubscribe.
  /// \param node_interfaces the ROS node interfaces required for core node functionality, including
  ///    NodeBaseInterface, NodeParametersInterface, NodeTopicsInterface, and NodeLoggingInterface.
  /// \param base_topic The topic to subscribe to.
  /// \param transport The transport hint to pass along
  /// \param custom_qos Custom quality of service
  /// \param options Subscriber options
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe(
    std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
    const std::string & base_topic,
    const std::string & transport,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions()) override;

  template<typename NodeT = rclcpp::Node::SharedPtr>
  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe(
    NodeT node,
    const std::string & base_topic,
    const std::string & transport,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    auto node_interfaces = std::make_shared<rclcpp::node_interfaces::NodeInterfaces<
          rclcpp::node_interfaces::NodeBaseInterface,
          rclcpp::node_interfaces::NodeParametersInterface,
          rclcpp::node_interfaces::NodeTopicsInterface,
          rclcpp::node_interfaces::NodeLoggingInterface>>(*node);
    subscribe(node_interfaces, base_topic, transport, custom_qos, options);
  }

  void subscribe(
    RequiredInterfaces /*node_interfaces*/, const std::string & /*topic*/,
    const rclcpp::QoS & /*qos*/) override
  {}
  void subscribe(
    RequiredInterfaces  /*node_interfaces*/,
    const std::string & /*topic*/,
    const rclcpp::QoS & /*qos*/,
    rclcpp::SubscriptionOptions /*options*/) override
  {}

  //! Re-subscribe to a topic.
  // Only works if this subscriber has previously been subscribed to a topic.
  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe() override;

  //! Force immediate unsubscription of this subscriber from its topic
  POINT_CLOUD_TRANSPORT_PUBLIC
  void unsubscribe() override;

private:
  std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces_;

  std::string topic_;
  std::string transport_;
  rclcpp::QoS qos_ = rclcpp::SystemDefaultsQoS();
  rclcpp::SubscriptionOptions options_;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__FULL_SUBSCRIBER_FILTER_HPP_
