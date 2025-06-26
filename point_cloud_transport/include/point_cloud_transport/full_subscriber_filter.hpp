#ifndef POINT_CLOUD_TRANSPORT__FULL_SUBSCRIBER_FILTER_HPP_
#define POINT_CLOUD_TRANSPORT__FULL_SUBSCRIBER_FILTER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
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
  public message_filters::SimpleFilter<sensor_msgs::msg::PointCloud2>
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
    const rclcpp::QoS & custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  template<typename NodeT = rclcpp::Node::SharedPtr>
  POINT_CLOUD_TRANSPORT_PUBLIC
  FullSubscriberFilter(
    NodeT node,
    const std::string & base_topic,
    const std::string & transport,
    const rclcpp::QoS & custom_qos,
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
  FullSubscriberFilter();

  POINT_CLOUD_TRANSPORT_PUBLIC
  ~FullSubscriberFilter();

  ///
  /// \brief Subscribe to a topic. If this Subscriber is already subscribed to a topic,
  /// this function will first unsubscribe.
  /// \param node The rclcpp Node to use to subscribe.
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
    const rclcpp::QoS & custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  template<typename NodeT = rclcpp::Node::SharedPtr>
  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe(
    NodeT node,
    const std::string & base_topic,
    const std::string & transport,
    const rclcpp::QoS & custom_qos,
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

  //! Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed to a topic.
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
  void cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m)
  {
    this->signalMessage(m);
  }

  Subscriber sub_;
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
