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

#include <list>
#include <memory>
#include <string>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/function.hpp>
#include <boost/noncopyable.hpp>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/log_utils.h>
#include <cras_cpp_common/log_utils/node.h>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/xmlrpc_value_utils.hpp>
#include <dynamic_reconfigure/Config.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <topic_tools/shape_shifter.h>
#include <XmlRpcValue.h>

#include <point_cloud_transport/transport_hints.h>

namespace point_cloud_transport
{

/**
 * Base class for plugins to Subscriber.
 */
class SubscriberPlugin : public cras::HasLogger, boost::noncopyable
{
public:
  typedef boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> Callback;

  //! \brief Result of cloud decoding. Either a `sensor_msgs::PointCloud2` holding the raw message, empty value or
  //! error message.
  typedef cras::expected<cras::optional<sensor_msgs::PointCloud2ConstPtr>, std::string> DecodeResult;

  explicit SubscriberPlugin(const cras::LogHelperPtr& log = std::make_shared<cras::NodeLogHelper>()) :
    cras::HasLogger(log)
  {
  }

  virtual ~SubscriberPlugin() = default;

  /**
   * Get a string identifier for the transport provided by
   * this plugin.
   */
  virtual std::string getTransportName() const = 0;

  /**
   * Whether the given topic and datatype match this transport.
   */
  virtual bool matchesTopic(const std::string& topic, const std::string& datatype) const = 0;

  /**
   * \brief Decode the given compressed pointcloud into a raw cloud.
   * \param[in] compressed The shapeshifter of the compressed pointcloud to be decoded.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The decoded raw pointcloud (if decoding succeeds), or an error message.
   */
  virtual DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                              const dynamic_reconfigure::Config& config) const = 0;

  /**
   * \brief Decode the given compressed pointcloud into a raw cloud using default config.
   * \param[in] compressed The shapeshifter of the compressed pointcloud to be decoded.
   * \return The decoded raw pointcloud (if decoding succeeds), or an error message.
   */
  DecodeResult decode(const topic_tools::ShapeShifter& compressed) const
  {
    return this->decode(compressed, dynamic_reconfigure::Config());
  }

  /**
   * \brief Decode the given compressed pointcloud into a raw cloud.
   * \param[in] compressed The shapeshifter of the compressed pointcloud to be decoded.
   * \param[in] config Config of the decompression (if it has any parameters). Pass a XmlRpc dict.
   * \return The decoded raw pointcloud (if decoding succeeds), or an error message.
   */
  DecodeResult decode(const topic_tools::ShapeShifter& compressed, const XmlRpc::XmlRpcValue& config) const
  {
    dynamic_reconfigure::Config configMsg;
    std::list<std::string> errors;
    if (!cras::convert(config, configMsg, true, &errors))
    {
      return cras::make_unexpected("Invalid decoder config: " + cras::join(errors, " "));
    }
    return this->decode(compressed, configMsg);
  }

  /**
   * \brief Encode the given raw pointcloud into the given shapeshifter object.
   * \tparam Config Type of the config object. This should be the generated dynamic_reconfigure interface of the
   *                corresponding point_cloud_transport subscriber.
   * \param[in] compressed The shapeshifter of the compressed pointcloud to be decoded.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The output shapeshifter holding the compressed cloud message (if encoding succeeds), or an error message.
   */
  template<typename Config>
  DecodeResult decode(const topic_tools::ShapeShifter& compressed, const Config& config) const
  {
    dynamic_reconfigure::Config configMsg;
    config.__toMessage__(configMsg);
    return this->decode(compressed, configMsg);
  }

  /**
   * Subscribe to a point cloud topic, version for arbitrary boost::function object.
   */
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 const Callback& callback, const ros::VoidPtr& tracked_object = {},
                 const point_cloud_transport::TransportHints& transport_hints = {},
                 bool allow_concurrent_callbacks = false)
  {
    return subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints,
                         allow_concurrent_callbacks);
  }

  /**
   * Subscribe to a point cloud topic, version for bare function.
   */
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 void(* fp)(const sensor_msgs::PointCloud2ConstPtr&),
                 const point_cloud_transport::TransportHints& transport_hints = {},
                 bool allow_concurrent_callbacks = false)
  {
    return subscribe(nh, base_topic, queue_size,
                     boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>(fp),
                     ros::VoidPtr(), transport_hints, allow_concurrent_callbacks);
  }

  /**
   * Subscribe to a point cloud topic, version for class member function with bare pointer.
   */
  template<class T>
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 void(T::*fp)(const sensor_msgs::PointCloud2ConstPtr&), T* obj,
                 const point_cloud_transport::TransportHints& transport_hints = {},
                 bool allow_concurrent_callbacks = false)
  {
    return subscribe(nh, base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints,
                     allow_concurrent_callbacks);
  }

  /**
   * Subscribe to a point cloud topic, version for class member function with shared_ptr.
   */
  template<class T>
  void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                 void(T::*fp)(const sensor_msgs::PointCloud2ConstPtr&),
                 const boost::shared_ptr<T>& obj,
                 const point_cloud_transport::TransportHints& transport_hints = {},
                 bool allow_concurrent_callbacks = false)
  {
    return subscribe(nh, base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints,
                     allow_concurrent_callbacks);
  }

  /**
   * Get the transport-specific communication topic.
   */
  virtual std::string getTopic() const = 0;

  /**
   * Returns the number of publishers this subscriber is connected to.
   */
  virtual uint32_t getNumPublishers() const = 0;

  /**
   * Unsubscribe the callback associated with this SubscriberPlugin.
   */
  virtual void shutdown() = 0;

  /**
   * Return the lookup name of the SubscriberPlugin associated with a specific
   * transport identifier.
   */
  static std::string getLookupName(const std::string& transport_type)
  {
    return "point_cloud_transport/" + transport_type + "_sub";
  }

protected:
  /**
   * Subscribe to a point cloud transport topic. Must be implemented by the subclass.
   */
  virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                             const Callback& callback, const ros::VoidPtr& tracked_object,
                             const point_cloud_transport::TransportHints& transport_hints,
                             bool allow_concurrent_callbacks) = 0;
};

class SingleTopicSubscriberPlugin : public SubscriberPlugin
{
public:
  /**
   * Return the communication topic name for a given base topic.
   *
   * Defaults to \<base topic\>/\<transport name\>.
   */
  virtual std::string getTopicToSubscribe(const std::string& base_topic) const = 0;

  /**
   * Return the datatype of the transported messages as text.
   */
  virtual std::string getDataType() const = 0;

  /**
   * Return the datatype of the dynamic reconfigure (as text in the form `package/Config`).
   * 
   * Return empty string if no reconfiguration is supported.
   */
  virtual std::string getConfigDataType() const = 0;
};

}
