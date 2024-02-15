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
#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include <point_cloud_transport/single_subscriber_publisher.hpp>
#include "point_cloud_transport/visibility_control.hpp"

namespace point_cloud_transport
{

//! Base class for plugins to Publisher.
class PublisherPlugin
{
public:
  /// \brief Result of cloud encoding. Either an holding the compressed cloud,
  /// empty value or error message.
  typedef tl::expected<std::optional<const std::shared_ptr<rclcpp::SerializedMessage>>,
      std::string> EncodeResult;

  PublisherPlugin() = default;
  PublisherPlugin(const PublisherPlugin &) = delete;
  PublisherPlugin & operator=(const PublisherPlugin &) = delete;

  virtual ~PublisherPlugin() {}

  //! Get a string identifier for the transport provided by this plugin
  virtual std::string getTransportName() const = 0;

  //! \brief Advertise a topic, simple version.
  POINT_CLOUD_TRANSPORT_PUBLIC
  void advertise(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
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

  //! Shutdown any advertisements associated with this PublisherPlugin.
  virtual void shutdown() = 0;

  //! Declare parameter with this PublisherPlugin.
  virtual void declareParameters(const std::string & base_topic) = 0;

  virtual std::string getTopicToAdvertise(const std::string & base_topic) const = 0;

  //! Return the lookup name of the PublisherPlugin associated with a specific transport identifier.
  POINT_CLOUD_TRANSPORT_PUBLIC
  static std::string getLookupName(const std::string & transport_name);

protected:
  //! Advertise a topic. Must be implemented by the subclass.
  virtual void advertiseImpl(
    std::shared_ptr<rclcpp::Node> node, const std::string & base_topic,
    rmw_qos_profile_t custom_qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions()) = 0;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__PUBLISHER_PLUGIN_HPP_
