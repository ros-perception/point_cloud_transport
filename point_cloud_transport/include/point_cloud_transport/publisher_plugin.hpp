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

#ifndef POINT_CLOUD_TRANSPORT__PUBLISHER_PLUGIN_HPP_
#define POINT_CLOUD_TRANSPORT__PUBLISHER_PLUGIN_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/single_subscriber_publisher.hpp>
#include <point_cloud_transport/visibility_control.hpp>

namespace point_cloud_transport
{

//! Base class for plugins to Publisher.
class PublisherPlugin
{
public:
  /// \brief Result of cloud encoding. Either a value holding the compressed cloud,
  /// empty value or error message.
  typedef tl::expected<std::optional<const std::shared_ptr<rclcpp::SerializedMessage>>,
      std::string> EncodeResult;

  PublisherPlugin() = default;
  PublisherPlugin(const PublisherPlugin &) = delete;
  PublisherPlugin & operator=(const PublisherPlugin &) = delete;

  virtual ~PublisherPlugin() = default;

  /**
   * \brief Get a string identifier for the transport provided by this plugin.
   *
   * The default implementation auto-discovers the name from the pluginlib
   * manifest XML by matching the demangled C++ type name of \c *this against
   * the \c type attribute of each \c <class> element.  The result is cached
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
   * Returns the value of the \c <message_type> element from the plugin
   * manifest XML.  The result is cached after the first call.
   * Override this method if you need a different value at runtime.
   */
  POINT_CLOUD_TRANSPORT_PUBLIC
  virtual std::string getMessageType() const;

  //! \brief Advertise a topic, simple version.
  POINT_CLOUD_TRANSPORT_PUBLIC
  void advertise(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    rclcpp::QoS custom_qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions());

  //! Returns the number of subscribers that are currently connected to this PublisherPlugin
  virtual uint32_t getNumSubscribers() const = 0;

  //! Returns the topic that this PublisherPlugin will publish on.
  virtual std::string getTopic() const = 0;

  //! Return the datatype of the transported messages (as text in the form `package/Message`).
  virtual std::string getDataType() const = 0;

  /// \brief Encode the given raw pointcloud into EncodeResult
  /// \param[in] raw The input raw pointcloud.
  /// \return The output EncodeResult holding the compressed cloud message (if encoding succeeds),
  /// or an error message.
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  virtual EncodeResult encode(const sensor_msgs::msg::PointCloud2 & raw) const = 0;

  //! Publish a point cloud using the transport associated with this PublisherPlugin.
  virtual void publish(const sensor_msgs::msg::PointCloud2 & message) const = 0;

  //! Publish a point cloud using the transport associated with this PublisherPlugin.
  POINT_CLOUD_TRANSPORT_PUBLIC
  virtual void publishPtr(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message) const;

  //! Get the underlying ROS publisher handle, if available.
  virtual rclcpp::PublisherBase::SharedPtr getPublisher() const = 0;

  //! Shutdown any advertisements associated with this PublisherPlugin.
  virtual void shutdown() = 0;

  //! Declare parameter with this PublisherPlugin.
  virtual void declareParameters(const std::string & base_topic) = 0;

  virtual std::string getTopicToAdvertise(const std::string & base_topic) const = 0;

  //! Return the lookup name of the PublisherPlugin associated with a specific transport identifier.
  POINT_CLOUD_TRANSPORT_PUBLIC
  static std::string getLookupName(const std::string & transport_name);

private:
  // Cache for manifest-discovered data (populated lazily by the base-class
  // implementation of getTransportName() / getMessageType()).
  mutable std::optional<PluginManifestData> manifest_data_;

protected:
  //! Advertise a topic. Must be implemented by the subclass.
  virtual void advertiseImpl(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    rclcpp::QoS custom_qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions()) = 0;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__PUBLISHER_PLUGIN_HPP_
