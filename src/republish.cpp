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

#include <memory>
#include <string>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/point_cloud_transport.h>
#include <point_cloud_transport/publisher.h>
#include <point_cloud_transport/publisher_plugin.h>
#include <point_cloud_transport/republish.h>
#include <point_cloud_transport/subscriber.h>
#include <point_cloud_transport/exception.h>

namespace point_cloud_transport
{

void RepublishNodelet::onInit()
{
  if (this->getMyArgv().empty())
  {
    throw std::runtime_error("Usage: republish in_transport in:=<in_base_topic> [out_transport] out:=<out_base_topic>");
  }

  std::string in_transport = this->getMyArgv()[0];
  std::string in_topic = this->getNodeHandle().resolveName("in");
  std::string out_topic = this->getNodeHandle().resolveName("out");

  const auto params = this->params(this->getPrivateNodeHandle());
  const auto in_queue_size = params->getParam("in_queue_size", 10_sz, "messages");
  const auto out_queue_size = params->getParam("out_queue_size", in_queue_size, "messages");

  this->pct = std::make_unique<PointCloudTransport>(this->getMTNodeHandle());
  point_cloud_transport::TransportHints hints(in_transport, {}, this->getPrivateNodeHandle());

  // There might be exceptions thrown by the loaders. We want the propagate so that the nodelet loading fails.

  if (this->getMyArgv().size() < 2)
  {
    // Use all available transports for output
    this->pub.reset(new point_cloud_transport::Publisher);
    *this->pub = this->pct->advertise(out_topic, out_queue_size);

    // Use Publisher::publish as the subscriber callback
    typedef void (point_cloud_transport::Publisher::*PublishMemFn)(const sensor_msgs::PointCloud2ConstPtr&) const;
    PublishMemFn pub_mem_fn = &point_cloud_transport::Publisher::publish;

    this->sub = this->pct->subscribe(in_topic, in_queue_size, boost::bind(pub_mem_fn, this->pub.get(), _1), this->pub,
                                     hints);
  }
  else
  {
    // Use one specific transport for output
    std::string out_transport = this->getMyArgv()[1];

    // Load transport plugin
    typedef point_cloud_transport::PublisherPlugin Plugin;
    auto loader = this->pct->getPublisherLoader();
    std::string lookup_name = Plugin::getLookupName(out_transport);
    this->pubPlugin = loader->createInstance(lookup_name);

    this->pubPlugin->advertise(this->getMTNodeHandle(), out_topic, out_queue_size, {}, {}, this->pubPlugin, false);

    // Use PublisherPlugin::publish as the subscriber callback
    typedef void (Plugin::*PublishMemFn)(const sensor_msgs::PointCloud2ConstPtr&) const;
    PublishMemFn pub_mem_fn = &Plugin::publish;

    this->sub = this->pct->subscribe(in_topic, in_queue_size, boost::bind(pub_mem_fn, this->pubPlugin.get(), _1),
                                     this->pubPlugin, in_transport);
  }
}

}

PLUGINLIB_EXPORT_CLASS(point_cloud_transport::RepublishNodelet, nodelet::Nodelet)
