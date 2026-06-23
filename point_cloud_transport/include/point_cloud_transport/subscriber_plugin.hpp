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

#ifndef POINT_CLOUD_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_
#define POINT_CLOUD_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <optional>

#include <rclcpp/macros.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/transport_hints.hpp>
#include <point_cloud_transport/visibility_control.hpp>

namespace point_cloud_transport
{

///
/// Base class for plugins to Subscriber.
///
class SubscriberPlugin
{
public:
  /// \brief Result of cloud decoding. Either a `sensor_msgs::msg::PointCloud2`
  /// holding the raw message, empty value or error message.
  typedef tl::expected<std::optional<sensor_msgs::msg::PointCloud2::ConstSharedPtr>,
      std::string> DecodeResult;

  SubscriberPlugin() = default;
  SubscriberPlugin(const SubscriberPlugin &) = delete;
  SubscriberPlugin & operator=(const SubscriberPlugin &) = delete;

  virtual ~SubscriberPlugin() = default;

  typedef std::function<void (const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)> Callback;

  /**
   * \brief Get a string identifier for the transport provided by this plugin.
   *
   * The default implementation auto-discovers the name from the pluginlib
   * manifest XML by matching the demangled C++ type name of \c *this against
   * the \c type attribute of each ``<class>`` element.  The result is cached
   * after the first call.
   *
   * Plugins that override getTransportName() continue to work unchanged.
   * Returning a different value than what is in the manifest is considered problematic.
   */
  POINT_CLOUD_TRANSPORT_PUBLIC
  virtual std::string getTransportName() const;

  /**
   * \brief Get the primary message type used by this plugin.
   *
   * Returns the value of the ``<message_type>`` element from the plugin
   * manifest XML.  The result is cached after the first call.
   * Override this method if you need a different value at runtime.
   */
  POINT_CLOUD_TRANSPORT_PUBLIC
  virtual std::string getMessageType() const;

  ///
  /// \brief Decode the given compressed pointcloud into a raw cloud.
  /// \param[in] compressed The rclcpp::SerializedMessage of the compressed pointcloud
  /// to be decoded.
  /// \return The decoded raw pointcloud (if decoding succeeds), or an error message.
  ///
  virtual DecodeResult decode(const std::shared_ptr<rclcpp::SerializedMessage> & compressed) const =
  0;

  ///
  /// \brief Subscribe to a pointcloud topic, version for arbitrary std::function object.
  ///
  void subscribe(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    const Callback & callback,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    return subscribeImpl(node_interfaces, base_topic, callback, custom_qos, options);
  }

  ///
  /// \brief Subscribe to a pointcloud topic, version for bare function.
  ///
  void subscribe(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    void (* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &),
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    return subscribe(
      node_interfaces, base_topic,
      std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)>(fp),
      custom_qos, options);
  }

  /// \brief Get the underlying ROS subscription handle, if available.
  virtual rclcpp::SubscriptionBase::SharedPtr getSubscription() const = 0;

  ///
  /// \brief Subscribe to a pointcloud topic, version for class member function with bare pointer.
  ///
  template<class T>
  void subscribe(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    void (T::* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &), T * obj,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    return subscribe(
      node_interfaces, base_topic,
      std::bind(fp, obj, std::placeholders::_1),
      custom_qos, options);
  }

  //! Get the transport-specific communication topic.
  virtual std::string getTopic() const = 0;

  //! Returns the number of publishers this subscriber is connected to.
  virtual uint32_t getNumPublishers() const = 0;

  //! Unsubscribe the callback associated with this SubscriberPlugin.
  virtual void shutdown() = 0;

  //! Return the datatype of the transported messages (as text in the form `package/msg/Message`).
  virtual std::string getDataType() const = 0;

  //! Declare parameter with this SubscriberPlugin.
  virtual void declareParameters() = 0;

  ///
  /// \brief Get the name of the topic that this SubscriberPlugin will subscribe to.
  /// \param[in] base_topic The topic that the subscriber was constructed with.
  /// \return The name of the topic that this SubscriberPlugin will subscribe to
  /// (e.g. ``<base_topic>/<transport_name>``).
  virtual std::string getTopicToSubscribe(const std::string & base_topic) const = 0;

  ///
  /// \brief Return the lookup name of the SubscriberPlugin associated with a specific
  /// transport identifier.
  /// \param transport_type The transport identifier.
  /// \return The lookup name of the SubscriberPlugin associated with the given transport.
  ///
  static std::string getLookupName(const std::string & transport_type)
  {
    return "point_cloud_transport/" + transport_type + "_sub";
  }

private:
  // Cache for manifest-discovered data (populated lazily by the base-class
  // implementation of getTransportName() / getMessageType()).
  mutable std::optional<PluginManifestData> manifest_data_;

protected:
  ///
  /// Subscribe to a point cloud transport topic. Must be implemented by the subclass.
  ///
  virtual void subscribeImpl(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    const Callback & callback,
    rclcpp::QoS custom_qos) = 0;

  virtual void subscribeImpl(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    const Callback & callback,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options) = 0;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__SUBSCRIBER_PLUGIN_HPP_
