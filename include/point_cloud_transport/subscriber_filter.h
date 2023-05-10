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

#include <string>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <message_filters/simple_filter.h>
#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/point_cloud_transport.h>
#include <point_cloud_transport/transport_hints.h>

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
 * The output connection for the SubscriberFilter object is the same signature as for roscpp
 * subscription callbacks, ie.
 */
class SubscriberFilter : public message_filters::SimpleFilter<sensor_msgs::PointCloud2>
{
public:
  /**
   * Constructor
   *
   * \param pct The transport to use.
   * \param base_topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport_hints The transport hints to pass along
   */
  SubscriberFilter(PointCloudTransport& pct, const std::string& base_topic, uint32_t queue_size,
                   const point_cloud_transport::TransportHints& transport_hints = {})
  {
    subscribe(pct, base_topic, queue_size, transport_hints);
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
   * \param pct The transport to use.
   * \param base_topic The topic to subscribe to.
   * \param queue_size The subscription queue size
   * \param transport_hints The transport hints to pass along
   */
  void subscribe(PointCloudTransport& pct, const std::string& base_topic, uint32_t queue_size,
                 const point_cloud_transport::TransportHints& transport_hints = {})
  {
    unsubscribe();

    sub_ = pct.subscribe(base_topic, queue_size, boost::bind(&SubscriberFilter::cb, this, _1), {}, transport_hints);
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
  void cb(const sensor_msgs::PointCloud2ConstPtr& m)
  {
    signalMessage(m);
  }

  Subscriber sub_;
};

}
