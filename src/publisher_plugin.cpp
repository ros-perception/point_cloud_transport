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

#include <list>
#include <string>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <dynamic_reconfigure/Config.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <XmlRpcValue.h>

#include <point_cloud_transport/publisher_plugin.h>
#include <point_cloud_transport/single_subscriber_publisher.h>

namespace point_cloud_transport
{

PublisherPlugin::EncodeResult PublisherPlugin::encode(const sensor_msgs::PointCloud2& raw) const
{
  return this->encode(raw, dynamic_reconfigure::Config());
}

PublisherPlugin::EncodeResult PublisherPlugin::encode(const sensor_msgs::PointCloud2& raw,
                                                      const XmlRpc::XmlRpcValue& config) const
{
  dynamic_reconfigure::Config configMsg;
  std::list<std::string> errors;
  if (!cras::convert(config, configMsg, true, &errors))
    return cras::make_unexpected("Invalid encoder config: " + cras::join(errors, " "));
  return this->encode(raw, configMsg);
}

void PublisherPlugin::advertise(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size, bool latch)
{
  advertiseImpl(nh, base_topic, queue_size, {}, {}, {}, latch);
}

void PublisherPlugin::advertise(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                const point_cloud_transport::SubscriberStatusCallback& connect_cb,
                                const point_cloud_transport::SubscriberStatusCallback& disconnect_cb,
                                const ros::VoidPtr& tracked_object, bool latch)
{
  advertiseImpl(nh, base_topic, queue_size, connect_cb, disconnect_cb, tracked_object, latch);
}

void PublisherPlugin::publish(const sensor_msgs::PointCloud2ConstPtr& message) const
{
  publish(*message);
}

std::string PublisherPlugin::getLookupName(const std::string& transport_name)
{
  return "point_cloud_transport/" + transport_name + "_pub";
}

}
