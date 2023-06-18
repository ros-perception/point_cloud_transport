// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak .. 2009, Willow Garage, Inc.

/*
 *
 * BSD 3-Clause License
 *
 * Copyright (c) Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
 * Copyright (c) 2009, Willow Garage, Inc.
 *
 *        All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 *        modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 *       THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *       AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *       IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *       DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *       FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *       DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *       SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *       CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *       OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *       OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <memory>
#include <string>

#include <message_filters/simple_filter.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/transport_hints.hpp>

namespace point_cloud_transport
{

/**
 * PointCloud2 subscription filter.
 *
 * This class wraps Subscriber as a "filter" compatible with the message_filters
 * package. It acts as a highest-level filter, simply passing messages from a point cloud
 * transport subscription through to the filters which have connected to it.
 *
 * When this object is destroyed it will unsubscribe from the ROS subscription.
 *
 * \section connections CONNECTIONS
 *
 * SubscriberFilter has no input connection.
 *
 * The output connection for the SubscriberFilter object is the same signature as for rclcpp
 * subscription callbacks, ie.
 */
class SubscriberFilter : public message_filters::SimpleFilter<sensor_msgs::msg::PointCloud2>
{
public:
  /**
   * Constructor
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param base_topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport The transport hint to pass along
   */
  SubscriberFilter(
    PointCloudTransport& pct,
    rclcpp::Node * node, const std::string & base_topic,
    const std::string & transport)
  {
    subscribe(pct, node, base_topic, transport);
  }

  /**
   * Empty constructor, use subscribe() to subscribe to a topic
   */
  SubscriberFilter()
  {
  }

  ~SubscriberFilter()
  {
    unsubscribe();
  }

  /**
   * Subscribe to a topic.
   *
   * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
   *
   * \param nh The ros::NodeHandle to use to subscribe.
   * \param base_topic The topic to subscribe to.
   */
  void subscribe(
    PointCloudTransport& pct,
    rclcpp::Node * node,
    const std::string & base_topic,
    const std::string & transport,
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default,
    rclcpp::SubscriptionOptions options = rclcpp::SubscriptionOptions())
  {
    unsubscribe();
    // TODO: Not quite right
    sub_ = pct.subscribe(base_topic, 1, std::bind(&SubscriberFilter::cb, this, std::placeholders::_1));
  }

  /**
   * Force immediate unsubscription of this subscriber from its topic
   */
  void unsubscribe()
  {
    sub_.shutdown();
  }

  std::string getTopic() const
  {
    return sub_.getTopic();
  }

  /**
   * Returns the number of publishers this subscriber is connected to.
   */
  uint32_t getNumPublishers() const
  {
    return sub_.getNumPublishers();
  }

  /**
   * Returns the name of the transport being used.
   */
  std::string getTransport() const
  {
    return sub_.getTransport();
  }

  /**
   * Returns the internal point_cloud_transport::Subscriber object.
   */
  const Subscriber& getSubscriber() const
  {
    return sub_;
  }

private:
  void cb(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m)
  {
    signalMessage(m);
  }

  Subscriber sub_;
};

}
