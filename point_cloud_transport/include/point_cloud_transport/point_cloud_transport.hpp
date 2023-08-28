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

#ifndef POINT_CLOUD_TRANSPORT__POINT_CLOUD_TRANSPORT_HPP_
#define POINT_CLOUD_TRANSPORT__POINT_CLOUD_TRANSPORT_HPP_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/node.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/single_subscriber_publisher.hpp>
#include <point_cloud_transport/subscriber.hpp>
#include <point_cloud_transport/transport_hints.hpp>

#include "point_cloud_transport/visibility_control.hpp"

namespace point_cloud_transport
{

//! Advertise and subscribe to PointCloud2 topics.
//! PointCloudTransport is analogous to rclcpp::Node in that it contains functions
//! to create publishers and subscriptions of PointCloud2 topics.

class PointCloudTransportLoader
{
public:
  //! Constructor
  POINT_CLOUD_TRANSPORT_PUBLIC
  PointCloudTransportLoader();

  //! Destructor
  POINT_CLOUD_TRANSPORT_PUBLIC
  virtual ~PointCloudTransportLoader();

  //! Returns the names of all transports declared in the system. Declared
  //! transports are not necessarily built or loadable.
  POINT_CLOUD_TRANSPORT_PUBLIC
  std::vector<std::string> getDeclaredTransports() const;

  //! Returns the names of all transports that are loadable in the system
  //! (keys are lookup names, values are names).
  POINT_CLOUD_TRANSPORT_PUBLIC
  std::unordered_map<std::string, std::string> getLoadableTransports() const;

  //! The loader that can load publisher plugins.
  POINT_CLOUD_TRANSPORT_PUBLIC
  PubLoaderPtr getPublisherLoader() const;

  //! The loader that can load subscriber plugins.
  POINT_CLOUD_TRANSPORT_PUBLIC
  SubLoaderPtr getSubscriberLoader() const;

  POINT_CLOUD_TRANSPORT_PUBLIC
  point_cloud_transport::PubLoaderPtr getPubLoader();

  POINT_CLOUD_TRANSPORT_PUBLIC
  point_cloud_transport::SubLoaderPtr getSubLoader();

protected:
  point_cloud_transport::PubLoaderPtr pub_loader_;
  point_cloud_transport::SubLoaderPtr sub_loader_;
};

/// \brief Advertise every available transport on pointcloud topics, free function version.
/// \param node The ROS node to use for any ROS operations
/// \param base_topic The base topic for the publisher
/// \param custom_qos The QoS profile to use for the underlying publisher(s)
/// \param options The publisher options to use for the underlying publisher(s)
/// \return The advertised publisher
POINT_CLOUD_TRANSPORT_PUBLIC
Publisher create_publisher(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & base_topic,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
  const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions());

/// \brief Subscribe to a pointcloud transport topic, free function version.
/// \param node The ROS node to use for any ROS operations
/// \param base_topic The base topic for the sbuscription
/// \param callback The callback to invoke on receipt of a message
/// \param transport The transport to use for the subscription
/// \param custom_qos The QoS profile to use for the underlying publisher
/// \param options The publisher options to use for the underlying publisher
/// \return The subscriber
POINT_CLOUD_TRANSPORT_PUBLIC
Subscriber create_subscription(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & base_topic,
  const Subscriber::Callback & callback,
  const std::string & transport,
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
  rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

class PointCloudTransport : public PointCloudTransportLoader
{
  using VoidPtr = std::shared_ptr<void>;

public:
  //! Constructor
  POINT_CLOUD_TRANSPORT_PUBLIC
  explicit PointCloudTransport(rclcpp::Node::SharedPtr node);

  POINT_CLOUD_TRANSPORT_PUBLIC
  ~PointCloudTransport() override = default;

  POINT_CLOUD_TRANSPORT_PUBLIC
  std::string getTransportOrDefault(const TransportHints * transport_hints)
  {
    std::string ret;
    if (nullptr == transport_hints) {
      TransportHints th(node_);
      ret = th.getTransport();
    } else {
      ret = transport_hints->getTransport();
    }
    return ret;
  }

  //! Advertise a PointCloud2 topic, simple version.
  POINT_CLOUD_TRANSPORT_PUBLIC
  Publisher advertise(
    const std::string & base_topic,
    uint32_t queue_size)
  {
    rclcpp::PublisherOptions options = rclcpp::PublisherOptions();
    rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
    custom_qos.depth = queue_size;
    return Publisher(node_, base_topic, pub_loader_, custom_qos, options);
  }

  //! Advertise a PointCloud2 topic, simple version.
  POINT_CLOUD_TRANSPORT_PUBLIC
  Publisher advertise(
    const std::string & base_topic,
    uint32_t queue_size,
    const rclcpp::PublisherOptions & options)
  {
    rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
    custom_qos.depth = queue_size;
    return Publisher(node_, base_topic, pub_loader_, custom_qos, options);
  }

  POINT_CLOUD_TRANSPORT_PUBLIC
  Publisher advertise(
    const std::string & base_topic,
    rmw_qos_profile_t custom_qos,
    const rclcpp::PublisherOptions & options = rclcpp::PublisherOptions())
  {
    return Publisher(node_, base_topic, pub_loader_, custom_qos, options);
  }

  // //! Subscribe to a point cloud topic, version for arbitrary std::function object.
  // POINT_CLOUD_TRANSPORT_PUBLIC
  // point_cloud_transport::Subscriber subscribe(
  //   const std::string & base_topic,
  //   uint32_t queue_size,
  //   const std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)> & callback)
  // {
  //   rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
  //   custom_qos.depth = queue_size;
  //   return subscribe(
  //     base_topic, custom_qos, callback, {}, nullptr);
  // }

  //! Advertise an PointCloud2 topic with subscriber status callbacks.
  // TODO(ros2) Implement when SubscriberStatusCallback is available
  // point_cloud_transport::Publisher advertise(const std::string& base_topic, uint32_t queue_size,
  //  const point_cloud_transport::SubscriberStatusCallback& connect_cb,
  //  const point_cloud_transport::SubscriberStatusCallback& disconnect_cb = {},
  //  const ros::VoidPtr& tracked_object = {}, bool latch = false);

  //! Subscribe to a point cloud topic, version for arbitrary std::function object.
  POINT_CLOUD_TRANSPORT_PUBLIC
  point_cloud_transport::Subscriber subscribe(
    const std::string & base_topic, rmw_qos_profile_t custom_qos,
    const std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)> & callback,
    const VoidPtr & tracked_object = {},
    const point_cloud_transport::TransportHints * transport_hints = nullptr)
  {
    (void)tracked_object;
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions();
    return Subscriber(
      node_, base_topic, callback, sub_loader_,
      getTransportOrDefault(transport_hints), custom_qos, options);
  }

  //! Subscribe to a point cloud topic, version for arbitrary std::function object.
  POINT_CLOUD_TRANSPORT_PUBLIC
  point_cloud_transport::Subscriber subscribe(
    const std::string & base_topic, uint32_t queue_size,
    const std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)> & callback,
    const VoidPtr & tracked_object = {},
    const point_cloud_transport::TransportHints * transport_hints = nullptr)
  {
    rmw_qos_profile_t custom_qos = rmw_qos_profile_sensor_data;
    custom_qos.depth = queue_size;
    return subscribe(
      base_topic, custom_qos, callback, tracked_object, transport_hints);
  }

  //! Subscribe to a point cloud topic, version for bare function.
  POINT_CLOUD_TRANSPORT_PUBLIC
  point_cloud_transport::Subscriber subscribe(
    const std::string & base_topic, rmw_qos_profile_t custom_qos,
    void (* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &),
    const point_cloud_transport::TransportHints * transport_hints = nullptr)
  {
    return subscribe(
      base_topic, custom_qos,
      std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)>(fp),
      VoidPtr(), transport_hints);
  }

  POINT_CLOUD_TRANSPORT_PUBLIC
  point_cloud_transport::Subscriber subscribe(
    const std::string & base_topic, uint32_t queue_size,
    void (* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &),
    const point_cloud_transport::TransportHints * transport_hints = nullptr)
  {
    return subscribe(
      base_topic, queue_size,
      std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)>(fp),
      VoidPtr(), transport_hints);
  }

  //! Subscribe to a point cloud topic, version for class member function with bare pointer.
  template<class T>
  point_cloud_transport::Subscriber subscribe(
    const std::string & base_topic, rmw_qos_profile_t custom_qos,
    void (T::* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &) const, T * obj,
    const point_cloud_transport::TransportHints * transport_hints = nullptr,
    bool allow_concurrent_callbacks = false)
  {
    return subscribe(
      base_topic, custom_qos, std::bind(
        fp,
        obj.get(), std::placeholders::_1), VoidPtr(), transport_hints);
  }

  template<class T>
  point_cloud_transport::Subscriber subscribe(
    const std::string & base_topic, uint32_t queue_size,
    void (T::* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &) const, T * obj,
    const point_cloud_transport::TransportHints * transport_hints = nullptr,
    bool allow_concurrent_callbacks = false)
  {
    return subscribe(
      base_topic, queue_size, std::bind(
        fp,
        obj.get(), std::placeholders::_1), VoidPtr(), transport_hints);
  }

  //! Subscribe to a point cloud topic, version for class member function with shared_ptr.
  template<class T>
  point_cloud_transport::Subscriber subscribe(
    const std::string & base_topic, rmw_qos_profile_t custom_qos,
    void (T::* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &) const,
    const std::shared_ptr<T> & obj,
    const point_cloud_transport::TransportHints * transport_hints = nullptr)
  {
    return subscribe(
      base_topic, custom_qos, std::bind(
        fp,
        obj.get(), std::placeholders::_1), obj, transport_hints);
  }

  template<class T>
  point_cloud_transport::Subscriber subscribe(
    const std::string & base_topic, uint32_t queue_size,
    void (T::* fp)(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &) const,
    const std::shared_ptr<T> & obj,
    const point_cloud_transport::TransportHints * transport_hints = nullptr)
  {
    return subscribe(
      base_topic, queue_size, std::bind(
        fp,
        obj.get(), std::placeholders::_1), obj, transport_hints);
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace point_cloud_transport

#endif  // POINT_CLOUD_TRANSPORT__POINT_CLOUD_TRANSPORT_HPP_
