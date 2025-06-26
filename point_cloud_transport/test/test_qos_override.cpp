// Copyright (c) 2023 Open Source Robotics Foundation, Inc.
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
//    * Neither the name of the Willow Garage nor the names of its
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

#include <gtest/gtest.h>

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "point_cloud_transport/point_cloud_transport.hpp"

class TestQosOverride : public ::testing::Test
{
  using NodeInterfacesPtr = std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::NodeBaseInterface,
        rclcpp::node_interfaces::NodeParametersInterface,
        rclcpp::node_interfaces::NodeTopicsInterface,
        rclcpp::node_interfaces::NodeLoggingInterface
      >>;

private:
  inline NodeInterfacesPtr get_node_interfaces(const rclcpp::Node::SharedPtr & node)
  {
    return std::make_shared<rclcpp::node_interfaces::NodeInterfaces<
               rclcpp::node_interfaces::NodeBaseInterface,
               rclcpp::node_interfaces::NodeParametersInterface,
               rclcpp::node_interfaces::NodeTopicsInterface,
               rclcpp::node_interfaces::NodeLoggingInterface
             >>(
      node->get_node_base_interface(),
      node->get_node_parameters_interface(),
      node->get_node_topics_interface(),
      node->get_node_logging_interface()
             );
  }

protected:
  void SetUp()
  {
    pub_node_ = rclcpp::Node::make_shared("test_publisher");
    qos_override_pub_node_ = rclcpp::Node::make_shared(
      "test_qos_override_publisher", rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter(
        "qos_overrides./pointcloud.publisher.reliability", "best_effort"),
    }));
    sub_node_ = rclcpp::Node::make_shared("test_subscriber");
    qos_override_sub_node_ = rclcpp::Node::make_shared(
      "test_qos_override_subscriber", rclcpp::NodeOptions().parameter_overrides(
    {
      rclcpp::Parameter(
        "qos_overrides./pointcloud.subscription.reliability", "best_effort"),
    }));

    pub_node_ni_ = get_node_interfaces(pub_node_);
    qos_override_pub_node_ni_ = get_node_interfaces(qos_override_pub_node_);
    sub_node_ni_ = get_node_interfaces(sub_node_);
    qos_override_sub_node_ni_ = get_node_interfaces(qos_override_sub_node_);
  }

  rclcpp::Node::SharedPtr pub_node_;
  rclcpp::Node::SharedPtr qos_override_pub_node_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::Node::SharedPtr qos_override_sub_node_;

  NodeInterfacesPtr pub_node_ni_;
  NodeInterfacesPtr qos_override_pub_node_ni_;
  NodeInterfacesPtr sub_node_ni_;
  NodeInterfacesPtr qos_override_sub_node_ni_;
};

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
TEST_F(TestQosOverride, qos_override_publisher_without_options) {
  auto pub = point_cloud_transport::create_publisher(
    pub_node_, "pointcloud",
    rmw_qos_profile_default);
  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("pointcloud");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();

  pub = point_cloud_transport::create_publisher(
    qos_override_pub_node_, "pointcloud", rmw_qos_profile_default);

  endpoint_info_vec = qos_override_pub_node_->get_publishers_info_by_topic("pointcloud");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();
}

TEST_F(TestQosOverride, qos_override_publisher_with_options) {
  rclcpp::PublisherOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions(
  {
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  });

  auto pub = point_cloud_transport::create_publisher(
    pub_node_, "pointcloud", rmw_qos_profile_default, options);
  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("pointcloud");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();

  pub = point_cloud_transport::create_publisher(
    qos_override_pub_node_, "pointcloud", rmw_qos_profile_default, options);

  endpoint_info_vec = qos_override_pub_node_->get_publishers_info_by_topic("pointcloud");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
  pub.shutdown();
}

TEST_F(TestQosOverride, qos_override_subscriber_without_options) {
  std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  auto sub = point_cloud_transport::create_subscription(
    sub_node_, "pointcloud", fcn, "raw", rmw_qos_profile_default);
  auto endpoint_info_vec = sub_node_->get_subscriptions_info_by_topic("pointcloud");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  sub.shutdown();

  sub = point_cloud_transport::create_subscription(
    qos_override_sub_node_, "pointcloud", fcn, "raw", rmw_qos_profile_default);

  endpoint_info_vec = qos_override_sub_node_->get_subscriptions_info_by_topic("pointcloud");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::Reliable);
}

TEST_F(TestQosOverride, qos_override_subscriber_with_options) {
  std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  rclcpp::SubscriptionOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions(
  {
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  });

  auto sub = point_cloud_transport::create_subscription(
    sub_node_, "pointcloud", fcn, "raw", rmw_qos_profile_default, options);
  auto endpoint_info_vec = sub_node_->get_subscriptions_info_by_topic("pointcloud");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  sub.shutdown();

  sub = point_cloud_transport::create_subscription(
    qos_override_sub_node_, "pointcloud", fcn, "raw", rmw_qos_profile_default, options);

  endpoint_info_vec = qos_override_sub_node_->get_subscriptions_info_by_topic("pointcloud");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
}
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif

TEST_F(TestQosOverride, qos_override_publisher_without_options_ni_api) {
  auto pub = point_cloud_transport::create_publisher(
    pub_node_ni_, "pointcloud",
    rclcpp::SystemDefaultsQoS());
  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("pointcloud");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();

  pub = point_cloud_transport::create_publisher(
    qos_override_pub_node_ni_, "pointcloud", rclcpp::SystemDefaultsQoS());

  endpoint_info_vec = qos_override_pub_node_->get_publishers_info_by_topic("pointcloud");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();
}

TEST_F(TestQosOverride, qos_override_publisher_with_options_ni_api) {
  rclcpp::PublisherOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions(
  {
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  });

  auto pub = point_cloud_transport::create_publisher(
    pub_node_ni_, "pointcloud", rclcpp::SystemDefaultsQoS(), options);
  auto endpoint_info_vec = pub_node_->get_publishers_info_by_topic("pointcloud");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
  pub.shutdown();

  pub = point_cloud_transport::create_publisher(
    qos_override_pub_node_ni_, "pointcloud", rclcpp::SystemDefaultsQoS(), options);

  endpoint_info_vec = qos_override_pub_node_->get_publishers_info_by_topic("pointcloud");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
  pub.shutdown();
}

TEST_F(TestQosOverride, qos_override_subscriber_without_options_ni_api) {
  std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  auto sub = point_cloud_transport::create_subscription(
    sub_node_ni_, "pointcloud", fcn, "raw", rclcpp::SystemDefaultsQoS());
  auto endpoint_info_vec = sub_node_->get_subscriptions_info_by_topic("pointcloud");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
  sub.shutdown();

  sub = point_cloud_transport::create_subscription(
    qos_override_sub_node_ni_, "pointcloud", fcn, "raw", rclcpp::SystemDefaultsQoS());

  endpoint_info_vec = qos_override_sub_node_->get_subscriptions_info_by_topic("pointcloud");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
}

TEST_F(TestQosOverride, qos_override_subscriber_with_options_ni_api) {
  std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)> fcn =
    [](const auto & msg) {(void)msg;};

  rclcpp::SubscriptionOptions options;
  options.qos_overriding_options = rclcpp::QosOverridingOptions(
  {
    rclcpp::QosPolicyKind::Depth,
    rclcpp::QosPolicyKind::Durability,
    rclcpp::QosPolicyKind::History,
    rclcpp::QosPolicyKind::Reliability,
  });

  auto sub = point_cloud_transport::create_subscription(
    sub_node_ni_, "pointcloud", fcn, "raw", rclcpp::SystemDefaultsQoS(), options);
  auto endpoint_info_vec = sub_node_->get_subscriptions_info_by_topic("pointcloud");
  EXPECT_EQ(endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
  sub.shutdown();

  sub = point_cloud_transport::create_subscription(
    qos_override_sub_node_ni_, "pointcloud", fcn, "raw", rclcpp::SystemDefaultsQoS(), options);

  endpoint_info_vec = qos_override_sub_node_->get_subscriptions_info_by_topic("pointcloud");
  EXPECT_EQ(
    endpoint_info_vec[0].qos_profile().reliability(),
    rclcpp::ReliabilityPolicy::BestEffort);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
