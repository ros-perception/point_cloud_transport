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

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/exception.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/single_subscriber_publisher.hpp>

namespace point_cloud_transport
{

struct Publisher::Impl
{
  explicit Impl(std::shared_ptr<rclcpp::Node> node)
  : logger_(node->get_logger()),
    unadvertised_(false)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  uint32_t getNumSubscribers() const
  {
    uint32_t count = 0;
    for (const auto & pub : publishers_) {
      count += pub->getNumSubscribers();
    }
    return count;
  }

  std::string getTopic() const
  {
    return base_topic_;
  }

  bool isValid() const
  {
    return !unadvertised_;
  }

  void shutdown()
  {
    if (!unadvertised_) {
      unadvertised_ = true;
      for (auto & pub : publishers_) {
        pub->shutdown();
      }
      publishers_.clear();
    }
  }

  rclcpp::Logger logger_;
  std::string base_topic_;
  PubLoaderPtr loader_;
  std::vector<std::shared_ptr<PublisherPlugin>> publishers_;
  bool unadvertised_;
};

Publisher::Publisher(
  std::shared_ptr<rclcpp::Node> node, const std::string & base_topic,
  PubLoaderPtr loader, rmw_qos_profile_t custom_qos,
  const rclcpp::PublisherOptions & options)
: impl_(std::make_shared<Impl>(node))
{
  // Resolve the name explicitly because otherwise the compressed topics don't remap
  // properly (#3652).
  std::string point_cloud_topic = rclcpp::expand_topic_or_service_name(
    base_topic,
    node->get_name(), node->get_namespace());
  impl_->base_topic_ = point_cloud_topic;
  impl_->loader_ = loader;

  auto ns_len = node->get_effective_namespace().length();
  std::string param_base_name = point_cloud_topic.substr(ns_len);
  std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');
  if (param_base_name.front() == '.') {
    param_base_name = param_base_name.substr(1);
  }
  std::vector<std::string> whitelist_vec;

  std::vector<std::string> all_transport_names;
  for (const auto & lookup_name : loader->getDeclaredClasses()) {
    all_transport_names.emplace_back(erase_last_copy(lookup_name, "_pub"));
  }

  try {
    whitelist_vec = node->declare_parameter<std::vector<std::string>>(
      param_base_name + ".enable_pub_plugins", all_transport_names);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    RCLCPP_DEBUG_STREAM(
      node->get_logger(), param_base_name << ".enable_pub_plugins" << " was previously declared"
    );
    whitelist_vec =
      node->get_parameter(
      param_base_name +
      ".enable_pub_plugins").get_value<std::vector<std::string>>();
  }

  std::set<std::string> whitelist;
  for (size_t i = 0; i < whitelist_vec.size(); ++i) {
    whitelist.insert(whitelist_vec[i]);
  }

  for (const auto & transport_name : whitelist) {
    const auto & lookup_name = transport_name + "_pub";
    try {
      auto pub = loader->createUniqueInstance(lookup_name);
      pub->advertise(node, point_cloud_topic, custom_qos, options);
      impl_->publishers_.push_back(std::move(pub));
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(
        impl_->logger_, "Failed to load plugin %s, error string: %s",
        lookup_name.c_str(), e.what());
    }
  }

  if (impl_->publishers_.empty()) {
    throw point_cloud_transport::Exception(
            "No plugins found! Does `rospack plugins --attrib=plugin "
            "point_cloud_transport` find any packages?");
  }
}

uint32_t Publisher::getNumSubscribers() const
{
  if (impl_ && impl_->isValid()) {
    return impl_->getNumSubscribers();
  }
  return 0;
}

std::string Publisher::getTopic() const
{
  if (impl_) {
    return impl_->getTopic();
  }
  return {};
}

void Publisher::publish(const sensor_msgs::msg::PointCloud2 & message) const
{
  if (!impl_ || !impl_->isValid()) {
    // TODO(ros2) Switch to RCUTILS_ASSERT when ros2/rcutils#112 is merged
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("point_cloud_transport");
    RCLCPP_FATAL(logger, "Call to publish() on an invalid point_cloud_transport::Publisher");
    return;
  }

  for (const auto & pub : impl_->publishers_) {
    if (pub->getNumSubscribers() > 0) {
      pub->publish(message);
    }
  }
}

void Publisher::publish(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message) const
{
  if (!impl_ || !impl_->isValid()) {
    // TODO(ros2) Switch to RCUTILS_ASSERT when ros2/rcutils#112 is merged
    auto logger = impl_ ? impl_->logger_ : rclcpp::get_logger("point_cloud_transport");
    RCLCPP_FATAL(logger, "Call to publish() on an invalid point_cloud_transport::Publisher");
    return;
  }

  for (const auto & pub : impl_->publishers_) {
    if (pub->getNumSubscribers() > 0) {
      pub->publishPtr(message);
    }
  }
}

void Publisher::shutdown()
{
  if (impl_) {
    impl_->shutdown();
    impl_.reset();
  }
}

Publisher::operator void *() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void *>(1) : reinterpret_cast<void *>(0);
}

}  // namespace point_cloud_transport
