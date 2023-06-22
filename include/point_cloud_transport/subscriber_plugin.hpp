/*
 * Copyright (c) 2023, Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POINT_CLOUD_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_
#define POINT_CLOUD_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_

#include <list>
#include <memory>
#include <string>
#include <optional>

#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/expected.hpp>
#include <point_cloud_transport/transport_hints.hpp>

namespace point_cloud_transport
{

/**
 * Base class for plugins to Subscriber.
 */
class SubscriberPlugin
{
public:
  //! \brief Result of cloud decoding. Either a `sensor_msgs::msg::PointCloud2`
  //! holding the raw message, empty value or error message.
  typedef cras::expected<std::optional<sensor_msgs::msg::PointCloud2::ConstSharedPtr>,
      std::string> DecodeResult;

  SubscriberPlugin() = default;
  SubscriberPlugin(const SubscriberPlugin &) = delete;
  SubscriberPlugin & operator=(const SubscriberPlugin &) = delete;

  virtual ~SubscriberPlugin() = default;

  typedef std::function<void (const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)> Callback;

  /**
   * Get a string identifier for the transport provided by
   * this plugin.
   */
  virtual std::string getTransportName() const = 0;


  /**
   * \brief Subscribe to an pointcloud topic, version for arbitrary std::function object.
   */
  void subscribe(
    rclcpp::Node * node, const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    return subscribeImpl(node, base_topic, callback, custom_qos, options);
  }

  /**
   * \brief Subscribe to an pointcloud topic, version for bare function.
   */
  void subscribe(
    rclcpp::Node * node, const std::string & base_topic,
    void (* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &),
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    return subscribe(
      node, base_topic,
      std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)>(fp),
      custom_qos, options);
  }

  /**
   * \brief Subscribe to an pointcloud topic, version for class member function with bare pointer.
   */
  template<class T>
  void subscribe(
    rclcpp::Node * node, const std::string & base_topic,
    void (T::* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &), T * obj,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    return subscribe(
      node, base_topic,
      std::bind(fp, obj, std::placeholders::_1), custom_qos, options);
  }

  /**
   * Get the transport-specific communication topic.
   */
  virtual std::string getTopic() const = 0;

  /**
   * Returns the number of publishers this subscriber is connected to.
   */
  virtual uint32_t getNumPublishers() const = 0;

  /**
   * Unsubscribe the callback associated with this SubscriberPlugin.
   */
  virtual void shutdown() = 0;

  /**
   * Return the lookup name of the SubscriberPlugin associated with a specific
   * transport identifier.
   */
  static std::string getLookupName(const std::string & transport_type)
  {
    return "point_cloud_transport/" + transport_type + "_sub";
  }

protected:
  /**
   * Subscribe to a point cloud transport topic. Must be implemented by the subclass.
   */
  virtual void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default) = 0;

  virtual void subscribeImpl(
    rclcpp::Node * node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options)
  {
    (void) options;
    RCLCPP_ERROR(
      node->get_logger(),
      "SubscriberPlugin::subscribeImpl with five arguments has not been overridden");
    this->subscribeImpl(node, base_topic, callback, custom_qos);
  }
};

class SingleTopicSubscriberPlugin : public SubscriberPlugin
{
public:
  /**
   * Return the communication topic name for a given base topic.
   *
   * Defaults to \<base topic\>/\<transport name\>.
   */
  virtual std::string getTopicToSubscribe(const std::string & base_topic) const = 0;

  /**
   * Return the datatype of the dynamic reconfigure (as text in the form `package/Config`).
   *
   * Return empty string if no reconfiguration is supported.
   */
  virtual std::string getConfigDataType() const = 0;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_