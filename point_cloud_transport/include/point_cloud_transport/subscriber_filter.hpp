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

#ifndef POINT_CLOUD_TRANSPORT__SUBSCRIBER_FILTER_HPP_
#define POINT_CLOUD_TRANSPORT__SUBSCRIBER_FILTER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/simple_filter.h>  // NOLINT
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/transport_hints.hpp>
#include "point_cloud_transport/visibility_control.hpp"

namespace point_cloud_transport
{

///
/// PointCloud2 subscription filter.
///
/// This class wraps Subscriber as a "filter" compatible with the message_filters
/// package. It acts as a highest-level filter, simply passing messages from a point cloud
/// transport subscription through to the filters which have connected to it.
///
/// When this object is destroyed it will unsubscribe from the ROS subscription.
///
/// SubscriberFilter has no input connection.
///
/// The output connection for the SubscriberFilter object is the same signature as for rclcpp
/// subscription callbacks.
///
class SubscriberFilter : public message_filters::SimpleFilter<sensor_msgs::msg::PointCloud2>
{
public:
  ///
  /// \brief Constructor
  /// \param node The rclcpp node to use to subscribe.
  /// \param base_topic The topic to subscribe to.
  /// \param queue_size The subscription queue size
  /// \param transport The transport hint to pass along
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  SubscriberFilter(
    std::shared_ptr<rclcpp::Node> node, const std::string & base_topic,
    const std::string & transport);

  //! Empty constructor, use subscribe() to subscribe to a topic
  POINT_CLOUD_TRANSPORT_PUBLIC
  SubscriberFilter();

  POINT_CLOUD_TRANSPORT_PUBLIC
  ~SubscriberFilter();
  ///
  /// \brief Subscribe to a topic. If this Subscriber is already subscribed to a topic,
  /// this function will first unsubscribe.
  /// \param node The rclcpp Node to use to subscribe.
  /// \param base_topic The topic to subscribe to.
  /// \param transport The transport hint to pass along
  /// \param custom_qos Custom quality of service
  /// \param options Subscriber options
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  void subscribe(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    const std::string & transport,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions());

  //! Force immediate unsubscription of this subscriber from its topic
  POINT_CLOUD_TRANSPORT_PUBLIC
  void unsubscribe();

  POINT_CLOUD_TRANSPORT_PUBLIC
  std::string getTopic() const;

  //! Returns the number of publishers this subscriber is connected to.
  POINT_CLOUD_TRANSPORT_PUBLIC
  uint32_t getNumPublishers() const;

  //! Returns the name of the transport being used.
  POINT_CLOUD_TRANSPORT_PUBLIC
  std::string getTransport() const;

  //! Returns the internal point_cloud_transport::Subscriber object.
  POINT_CLOUD_TRANSPORT_PUBLIC
  const Subscriber & getSubscriber() const;

private:
  void cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & m)
  {
    signalMessage(m);
  }

  Subscriber sub_;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__SUBSCRIBER_FILTER_HPP_
