#include "point_cloud_transport/full_subscriber_filter.hpp"

#include <memory>
#include <string>

namespace point_cloud_transport
{
FullSubscriberFilter::FullSubscriberFilter(
  std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
  const std::string & base_topic,
  const std::string & transport,
  const rclcpp::QoS & custom_qos,
  rclcpp::SubscriptionOptions options)
{
  subscribe(node_interfaces, base_topic, transport, custom_qos, options);
}

FullSubscriberFilter::FullSubscriberFilter()
{
}

FullSubscriberFilter::~FullSubscriberFilter()
{
  unsubscribe();
}

void FullSubscriberFilter::subscribe(
  std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeTopicsInterface,
    rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
  const std::string & base_topic,
  const std::string & transport,
  const rclcpp::QoS & custom_qos,
  rclcpp::SubscriptionOptions options)
{
  unsubscribe();
  sub_ = point_cloud_transport::create_subscription(
    node_interfaces, base_topic,
    std::bind(&FullSubscriberFilter::cb, this, std::placeholders::_1),
    transport, custom_qos, options);
  
  this->node_interfaces_ = node_interfaces;
  this->topic_ = base_topic;
  this->transport_ = transport;
  this->qos_ = custom_qos;
  this->options_ = options;
}

void FullSubscriberFilter::subscribe()
{
  if (!topic_.empty()) {
    subscribe(node_interfaces_, topic_, transport_, qos_, options_);
  }
}

void FullSubscriberFilter::unsubscribe()
{
  sub_.shutdown();
}

std::string FullSubscriberFilter::getTopic() const
{
  return this->topic_;
}

uint32_t FullSubscriberFilter::getNumPublishers() const
{
  return sub_.getNumPublishers();
}

std::string FullSubscriberFilter::getTransport() const
{
  return this->transport_;
}

const Subscriber & FullSubscriberFilter::getSubscriber() const
{
  return sub_;
}
}  // namespace point_cloud_transport
