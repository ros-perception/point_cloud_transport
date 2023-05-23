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
#include <type_traits>

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/type_utils.hpp>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/server.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <point_cloud_transport/NoConfigConfig.h>
#include <point_cloud_transport/subscriber_plugin.h>

namespace point_cloud_transport
{

/**
 * Base class to simplify implementing most plugins to Subscriber.
 *
 * The base class simplifies implementing a SubscriberPlugin in the common case that
 * all communication with the matching PublisherPlugin happens over a single ROS
 * topic using a transport-specific message type. SimpleSubscriberPlugin is templated
 * on the transport-specific message type.
 *
 * A subclass need implement only two methods:
 * - getTransportName() from SubscriberPlugin
 * - internalCallback() - processes a message and invoked the user PointCloud2 callback if
 * appropriate.
 *
 * For access to the parameter server and name remappings, use nh().
 *
 * getTopicToSubscribe() controls the name of the internal communication topic. It
 * defaults to \<base topic\>/\<transport name\>.
 */
template<class M, class Config = point_cloud_transport::NoConfigConfig>
class SimpleSubscriberPlugin : public SingleTopicSubscriberPlugin
{
public:
  ~SimpleSubscriberPlugin() override
  {
  }

  std::string getTopic() const override
  {
    if (simple_impl_)
    {
      return simple_impl_->sub_.getTopic();
    }
    return {};
  }

  bool matchesTopic(const std::string& topic, const std::string& datatype) const override
  {
    return datatype == ros::message_traits::DataType<M>::value() &&
        cras::endsWith(topic, std::string("/" + getTransportName()));
  }

  uint32_t getNumPublishers() const override
  {
    if (simple_impl_)
    {
      return simple_impl_->sub_.getNumPublishers();
    }
    return 0;
  }

  void shutdown() override
  {
    reconfigure_server_.reset();
    if (simple_impl_)
    {
      simple_impl_->sub_.shutdown();
    }
  }

  /**
   * \brief Decode the given compressed pointcloud into a raw message.
   * \param[in] compressed The input compressed pointcloud.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The raw cloud message (if encoding succeeds), or an error message.
   */
  virtual DecodeResult decodeTyped(const M& compressed, const Config& config) const = 0;

  /**
   * \brief Decode the given compressed pointcloud into a raw message.
   * \param[in] compressed The input compressed pointcloud.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The raw cloud message (if encoding succeeds), or an error message.
   */
  virtual DecodeResult decodeTyped(const typename M::ConstPtr& compressed, const Config& config) const
  {
    return this->decodeTyped(*compressed, config);
  }

  /**
   * \brief Decode the given compressed pointcloud into a raw message with default configuration.
   * \param[in] compressed The input compressed pointcloud.
   * \param[in] config Config of the decompression (if it has any parameters).
   * \return The raw cloud message (if encoding succeeds), or an error message.
   */
  virtual DecodeResult decodeTyped(const M& compressed) const
  {
    return this->decodeTyped(compressed, Config::__getDefault__());
  }

  DecodeResult decode(const topic_tools::ShapeShifter& compressed,
                      const dynamic_reconfigure::Config& configMsg) const override
  {
    Config config = Config::__getDefault__();
    // dynamic_reconfigure has a bug and generates __fromMessage__ with non-const message arg, although it is only read
    if (!config.__fromMessage__(const_cast<dynamic_reconfigure::Config&>(configMsg)))
    {
      return cras::make_unexpected(
          std::string("Wrong configuration options given to " + this->getTransportName() + " transport decoder."));
    }

    typename M::ConstPtr msg;
    try
    {
      msg = compressed.instantiate<M>();
    }
    catch (const ros::Exception& e)
    {
      return cras::make_unexpected(cras::format("Invalid shapeshifter passed to transport decoder: %s.", e.what()));
    }

    return this->decodeTyped(msg, config);
  }

protected:
  std::string base_topic_;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_{Config::__getDefault__()};

  virtual void configCb(Config& config, uint32_t level)
  {
    config_ = config;
  }

  template<typename C, std::enable_if_t<!std::is_same<C, NoConfigConfig>::value, int> = 0>
  void _startDynamicReconfigureServer()
  {
    this->startDynamicReconfigureServer();
  }

  template<typename C, std::enable_if_t<std::is_same<C, NoConfigConfig>::value, int> = 0>
  void _startDynamicReconfigureServer()
  {
    // Do not start reconfigure server if there are no configuration options.
  }

  virtual void startDynamicReconfigureServer()
  {
    // Set up reconfigure server for this topic
    reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
    typename ReconfigureServer::CallbackType f =
        boost::bind(&SimpleSubscriberPlugin<M, Config>::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);
  }

  /**
   * Process a message. Must be implemented by the subclass.
   */
  virtual void callback(const typename M::ConstPtr& message, const Callback& user_cb)
  {
    DecodeResult res = this->decodeTyped(message, config_);
    if (!res)
      ROS_ERROR("Error decoding message by transport %s: %s.", this->getTransportName().c_str(), res.error().c_str());
    else if (res.value())
    {
      user_cb(res.value().value());
    }
  }

  std::string getTopicToSubscribe(const std::string& base_topic) const override
  {
    return base_topic + "/" + getTransportName();
  }

  std::string getDataType() const override
  {
    return ros::message_traits::DataType<M>::value();
  }

  template<typename C, std::enable_if_t<!std::is_same<C, NoConfigConfig>::value, int> = 0>
  std::string _getConfigDataType() const
  {
    return cras::removeSuffix(cras::replace(cras::getTypeName<Config>(), "::", "/"), "Config");
  }

  template<typename C, std::enable_if_t<std::is_same<C, NoConfigConfig>::value, int> = 0>
  std::string _getConfigDataType() const
  {
    return {};
  }

  std::string getConfigDataType() const override
  {
    return _getConfigDataType<Config>();
  }

  void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                     const Callback& callback, const ros::VoidPtr& tracked_object,
                     const point_cloud_transport::TransportHints& transport_hints,
                     bool allow_concurrent_callbacks) override
  {
    // Push each group of transport-specific parameters into a separate sub-namespace
    ros::NodeHandle param_nh(transport_hints.getParameterNH(), getTransportName());
    simple_impl_ = std::make_unique<SimpleSubscriberPluginImpl>(param_nh);

    ros::SubscribeOptions ops;
    ops.init<M>(getTopicToSubscribe(base_topic), queue_size,
                boost::bind(&SimpleSubscriberPlugin::callback, this, _1, callback));
    ops.tracked_object = tracked_object;
    ops.transport_hints = transport_hints.getRosHints();
    ops.allow_concurrent_callbacks = allow_concurrent_callbacks;

    simple_impl_->sub_ = nh.subscribe(ops);

    this->_startDynamicReconfigureServer<Config>();
  }

  /**
   * Returns the ros::NodeHandle to be used for parameter lookup.
   */
  const ros::NodeHandle& nh() const
  {
    return simple_impl_->param_nh_;
  }

private:
  struct SimpleSubscriberPluginImpl
  {
    explicit SimpleSubscriberPluginImpl(const ros::NodeHandle& nh)
        : param_nh_(nh)
    {
    }

    const ros::NodeHandle param_nh_;
    ros::Subscriber sub_;
  };

  std::unique_ptr<SimpleSubscriberPluginImpl> simple_impl_;
};

}
