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
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "point_cloud_transport/exception.hpp"
#include "point_cloud_transport/point_cloud_transport.hpp"
#include "point_cloud_transport/publisher.hpp"
#include "point_cloud_transport/publisher_plugin.hpp"
#include "point_cloud_transport/republish.hpp"
#include "point_cloud_transport/subscriber.hpp"
#include "point_cloud_transport/qos.hpp"

using namespace std::chrono_literals;

namespace point_cloud_transport
{
Republisher::Republisher(const rclcpp::NodeOptions & options)
: Node("point_cloud_republisher", options)
{
  // Initialize Republishercomponent after construction
  // shared_from_this can't be used in the constructor
  this->timer_ = create_wall_timer(
    1ms, [this]() {
      if (initialized_) {
        timer_->cancel();
      } else {
        this->initialize();
        initialized_ = true;
      }
    });
}

void Republisher::initialize()
{
  std::string in_topic = rclcpp::expand_topic_or_service_name(
    "in",
    this->get_name(), this->get_namespace());

  std::string out_topic = rclcpp::expand_topic_or_service_name(
    "out",
    this->get_name(), this->get_namespace());

  std::string in_transport = "raw";
  this->declare_parameter<std::string>("in_transport", in_transport);
  if (!this->get_parameter(
      "in_transport", in_transport))
  {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "The 'in_transport' parameter was not defined." << in_transport);
  } else {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "The 'in_transport' parameter is set to: " << in_transport);
  }

  std::string out_transport = "";
  this->declare_parameter<std::string>("out_transport", out_transport);
  if (!this->get_parameter(
      "out_transport", out_transport))
  {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "The parameter 'out_transport' was not defined." << out_transport);
  } else {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "The 'out_transport' parameter is set to: " << out_transport);
  }

   std::string publisher_qos_profile_name = "DEFAULT";
   this->declare_parameter<std::string>("publisher_qos_profile", publisher_qos_profile_name);
   if(this->get_parameter("publisher_qos_profile", publisher_qos_profile_name))
   {
       RCLCPP_INFO_STREAM(this->get_logger(), "found param publisher_qos_profile");
   }
   RCLCPP_INFO_STREAM(
       this->get_logger(),
       "The 'publisher_qos_profile' parameter is set to: " << publisher_qos_profile_name);

   rmw_qos_profile_t publisher_qos_profile = rmw_qos_profile_default;
   bool foundProfile = detectQoSProfileFromString(publisher_qos_profile_name,
                                                  publisher_qos_profile);

   if(!foundProfile)
   {
       RCLCPP_WARN_STREAM(this->get_logger(),
                          "unkown QoS profile, will use SYSTEM_DEFAULT instead."
                          << "Please check the 'publisher_qos_profile' param "
                          << "(valid values are 'PARAMS_EVENTS', 'PARAMS', "
                          << "'SENSOR_DATA', 'SERVICES_DEFAULT' and 'SYSTEM_DEFAULT'.");
       publisher_qos_profile = rmw_qos_profile_default;
   }


  pct = std::make_shared<point_cloud_transport::PointCloudTransport>(this->shared_from_this());

  if (out_transport.empty()) {
    // Use all available transports for output
    this->simple_pub =
      std::make_shared<point_cloud_transport::Publisher>(
      pct->advertise(
        out_topic,
        publisher_qos_profile));

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "out topic1: " << this->simple_pub->getTopic());

    // Use Publisher::publish as the subscriber callback
    typedef void (point_cloud_transport::Publisher::* PublishMemFn)(
      const sensor_msgs::msg::
      PointCloud2::ConstSharedPtr &) const;
    PublishMemFn pub_mem_fn = &point_cloud_transport::Publisher::publish;

    const point_cloud_transport::TransportHints hint(in_transport);
    this->sub = pct->subscribe(
      in_topic, static_cast<uint32_t>(1),
      pub_mem_fn, this->simple_pub, &hint);
  } else {
    // Load transport plugin
    typedef point_cloud_transport::PublisherPlugin Plugin;
    auto loader = pct->getPublisherLoader();
    std::string lookup_name = Plugin::getLookupName(out_transport);
    RCLCPP_INFO(this->get_logger(), "Loading %s publisher", lookup_name.c_str());

    auto instance = loader->createUniqueInstance(lookup_name);
    // DO NOT use instance after this line
    this->pub = std::move(instance);
    pub->advertise(this->shared_from_this(), out_topic, publisher_qos_profile);

    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "out topic2: " << this->pub->getTopic());

    // Use PublisherPlugin::publish as the subscriber callback
    typedef void (point_cloud_transport::PublisherPlugin::* PublishMemFn)(
      const sensor_msgs::msg::
      PointCloud2::ConstSharedPtr &) const;
    PublishMemFn pub_mem_fn = &point_cloud_transport::PublisherPlugin::publish;

    RCLCPP_INFO(this->get_logger(), "Loading %s subscriber", in_topic.c_str());

    const point_cloud_transport::TransportHints hint(in_transport);
    this->sub = pct->subscribe(
      in_topic, static_cast<uint32_t>(1),
      pub_mem_fn, pub, &hint);
  }
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "in topic: " << this->sub.getTopic());
}
}  // namespace point_cloud_transport

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_transport::Republisher)
