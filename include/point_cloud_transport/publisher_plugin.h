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

#include <boost/noncopyable.hpp>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_topic_tools/shape_shifter.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <XmlRpcValue.h>

#include <point_cloud_transport/single_subscriber_publisher.h>

namespace point_cloud_transport
{

//! Base class for plugins to Publisher.
class PublisherPlugin : public cras::HasLogger, boost::noncopyable
{
public:
  // There has to be cras::ShapeShifter instead of topic_tools::ShapeShifter to avoid Melodic memory corruption issues.
  //! \brief Result of cloud encoding. Either a shapeshifter holding the compressed cloud, empty value or error message.
  typedef cras::expected<cras::optional<cras::ShapeShifter>, std::string> EncodeResult;

  explicit PublisherPlugin(const cras::LogHelperPtr& log = std::make_shared<cras::NodeLogHelper>()) :
      cras::HasLogger(log)
  {
  }
  
  virtual ~PublisherPlugin() = default;

  //! Get a string identifier for the transport provided by this plugin
  virtual std::string getTransportName() const = 0;

  /**
   * Whether the given topic and datatype match this transport.
   */
  virtual bool matchesTopic(const std::string& topic, const std::string& datatype) const = 0;

  /**
   * \brief Encode the given raw pointcloud into the given shapeshifter object.
   * \param[in] raw The input raw pointcloud.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The output shapeshifter holding the compressed cloud message (if encoding succeeds), or an error message.
   */
  virtual EncodeResult encode(const sensor_msgs::PointCloud2& raw, const dynamic_reconfigure::Config& config) const = 0;

  /**
   * \brief Encode the given raw pointcloud into the given shapeshifter object using default configuration.
   * \param[in] raw The input raw pointcloud.
   * \return The output shapeshifter holding the compressed cloud message (if encoding succeeds), or an error message.
   */
  EncodeResult encode(const sensor_msgs::PointCloud2& raw) const;

  /**
 * \brief Encode the given raw pointcloud into the given shapeshifter object.
 * \param[in] raw The input raw pointcloud.
 * \param[in] config Config of the compression (if it has any parameters). Pass a XmlRpc dict.
 * \return The output shapeshifter holding the compressed cloud message (if encoding succeeds), or an error message.
 */
  EncodeResult encode(const sensor_msgs::PointCloud2& raw, const XmlRpc::XmlRpcValue& config) const;

  /**
   * \brief Encode the given raw pointcloud into the given shapeshifter object.
   * \tparam Config Type of the config object. This should be the generated dynamic_reconfigure interface of the
   *                corresponding point_cloud_transport publisher.
   * \param[in] raw The input raw pointcloud.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The output shapeshifter holding the compressed cloud message (if encoding succeeds), or an error message.
   */
  template<typename Config>
  EncodeResult encode(const sensor_msgs::PointCloud2& raw, const Config& config) const
  {
    dynamic_reconfigure::Config configMsg;
    config.__toMessage__(configMsg);
    return this->encode(raw, configMsg);
  }

  //! Advertise a topic, simple version.
  void advertise(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size, bool latch = true);

  //! Advertise a topic with subscriber status callbacks.
  void advertise(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 const point_cloud_transport::SubscriberStatusCallback& connect_cb,
                 const point_cloud_transport::SubscriberStatusCallback& disconnect_cb = {},
                 const ros::VoidPtr& tracked_object = {}, bool latch = true);

  //! Returns the number of subscribers that are currently connected to this PublisherPlugin
  virtual uint32_t getNumSubscribers() const = 0;

  //! Returns the topic that this PublisherPlugin will publish on.
  virtual std::string getTopic() const = 0;

  //! Publish a point cloud using the transport associated with this PublisherPlugin.
  virtual void publish(const sensor_msgs::PointCloud2& message) const = 0;

  //! Publish a point cloud using the transport associated with this PublisherPlugin.
  virtual void publish(const sensor_msgs::PointCloud2ConstPtr& message) const;

  //! Shutdown any advertisements associated with this PublisherPlugin.
  virtual void shutdown() = 0;

  //! Return the lookup name of the PublisherPlugin associated with a specific transport identifier.
  static std::string getLookupName(const std::string& transport_name);

protected:
  //! Advertise a topic. Must be implemented by the subclass.
  virtual void advertiseImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const point_cloud_transport::SubscriberStatusCallback& connect_cb,
                             const point_cloud_transport::SubscriberStatusCallback& disconnect_cb,
                             const ros::VoidPtr& tracked_object, bool latch) = 0;
};

}
