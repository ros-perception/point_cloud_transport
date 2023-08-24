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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <pluginlib/exceptions.hpp>

#include "rclcpp/expand_topic_or_service_name.hpp"
#include "rclcpp/logging.hpp"

#include <point_cloud_transport/exception.hpp>
#include <point_cloud_transport/subscriber.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

namespace point_cloud_transport
{

struct Subscriber::Impl
{
  Impl(std::shared_ptr<rclcpp::Node> node, SubLoaderPtr loader)
  : logger_(node->get_logger()),
    loader_(loader),
    unsubscribed_(false)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  bool isValid() const
  {
    return !unsubscribed_;
  }

  void shutdown()
  {
    if (!unsubscribed_) {
      unsubscribed_ = true;
      if (subscriber_) {
        subscriber_->shutdown();
      }
    }
  }

  rclcpp::Logger logger_;
  std::string lookup_name_;
  point_cloud_transport::SubLoaderPtr loader_;
  std::shared_ptr<SubscriberPlugin> subscriber_;
  bool unsubscribed_;
};

Subscriber::Subscriber(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & base_topic,
  const Callback & callback,
  SubLoaderPtr loader,
  const std::string & transport,
  rmw_qos_profile_t custom_qos,
  rclcpp::SubscriptionOptions options)
: impl_(std::make_shared<Impl>(node, loader))
{
  // Load the plugin for the chosen transport.
  std::string lookup_name = SubscriberPlugin::getLookupName(transport);
  try {
    impl_->subscriber_ = loader->createSharedInstance(lookup_name);
  } catch (pluginlib::PluginlibException & e) {
    throw point_cloud_transport::TransportLoadException(transport, e.what());
  }
  // Hang on to the class loader so our shared library doesn't disappear from under us.
  impl_->loader_ = loader;

  // Try to catch if user passed in a transport-specific topic as base_topic.
  std::string clean_topic = base_topic;
  size_t found = clean_topic.rfind('/');
  if (found != std::string::npos) {
    std::string transport = clean_topic.substr(found + 1);
    std::string plugin_name = SubscriberPlugin::getLookupName(transport);
    std::vector<std::string> plugins = loader->getDeclaredClasses();
    if (std::find(plugins.begin(), plugins.end(), plugin_name) != plugins.end()) {
      std::string real_base_topic = clean_topic.substr(0, found);

      RCLCPP_WARN(
        impl_->logger_,
        "[point_cloud_transport] It looks like you are trying to subscribe directly to a "
        "transport-specific point_cloud topic '%s', in which case you will likely get a  "
        "connection error. Try subscribing to the base topic '%s' instead with parameter "
        "~point_cloud_transport set to '%s' (on the command line, _point_cloud_transport:=%s). ",
        clean_topic.c_str(), real_base_topic.c_str(), transport.c_str(), transport.c_str());
    }
  }

  // Tell plugin to subscribe.
  impl_->subscriber_->subscribe(node, base_topic, callback, custom_qos, options);
  RCLCPP_INFO(impl_->logger_, "Subscribing to: %s\n", impl_->subscriber_->getTopic().c_str());
}

std::string Subscriber::getTopic() const
{
  if (impl_) {
    return impl_->subscriber_->getTopic();
  }
  return {};
}

uint32_t Subscriber::getNumPublishers() const
{
  if (impl_) {
    return impl_->subscriber_->getNumPublishers();
  }
  return 0;
}

std::string Subscriber::getTransport() const
{
  if (impl_) {
    return impl_->subscriber_->getTransportName();
  }
  return {};
}

void Subscriber::shutdown()
{
  if (impl_) {
    impl_->shutdown();
  }
}

Subscriber::operator void *() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void *>(1) : reinterpret_cast<void *>(0);
}

}  // namespace point_cloud_transport
