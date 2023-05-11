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
#include <boost/function.hpp>

#include <cras_cpp_common/expected.hpp>
#include <cras_cpp_common/optional.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_topic_tools/shape_shifter.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/server.h>
#include <ros/assert.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <topic_tools/shape_shifter.h>

#include <point_cloud_transport/publisher_plugin.h>
#include <point_cloud_transport/NoConfigConfig.h>
#include <point_cloud_transport/single_subscriber_publisher.h>

namespace point_cloud_transport
{

/**
 * \brief Base class to simplify implementing most plugins to Publisher.
 *
 * This base class vastly simplifies implementing a PublisherPlugin in the common
 * case that all communication with the matching SubscriberPlugin happens over a
 * single ROS topic using a transport-specific message type. SimplePublisherPlugin
 * is templated on the transport-specific message type and publisher dynamic
 * reconfigure type.
 *
 * A subclass need implement only two methods:
 * - getTransportName() from PublisherPlugin
 * - encodeTyped()
 *
 * For access to the parameter server and name remappings, use nh().
 *
 * getTopicToAdvertise() controls the name of the internal communication topic.
 * It defaults to \<base topic\>/\<transport name\>.
 *
 * \tparam M Type of the published messages.
 * \tparam Config Type of the publisher dynamic configuration.
 */
template<class M, class Config = point_cloud_transport::NoConfigConfig>
class SimplePublisherPlugin : public point_cloud_transport::PublisherPlugin
{
public:
  //! \brief Result of cloud encoding. Either the compressed cloud message, empty value, or error message.
  typedef cras::expected<cras::optional<M>, std::string> TypedEncodeResult;

  ~SimplePublisherPlugin() override
  {
  }

  uint32_t getNumSubscribers() const override
  {
    if (simple_impl_)
    {
      return simple_impl_->pub_.getNumSubscribers();
    }
    return 0;
  }

  std::string getTopic() const override
  {
    if (simple_impl_)
    {
      return simple_impl_->pub_.getTopic();
    }
    return {};
  }

  bool matchesTopic(const std::string& topic, const std::string& datatype) const override
  {
    return datatype == ros::message_traits::DataType<M>::value() &&
        cras::endsWith(topic, std::string("/" + getTransportName()));
  }

  void publish(const sensor_msgs::PointCloud2& message) const override
  {
    if (!simple_impl_ || !simple_impl_->pub_)
    {
      ROS_ASSERT_MSG(false, "Call to publish() on an invalid point_cloud_transport::SimplePublisherPlugin");
      return;
    }

    publish(message, bindInternalPublisher(simple_impl_->pub_));
  }

  void shutdown() override
  {
    if (simple_impl_)
    {
      simple_impl_->pub_.shutdown();
    }
  }

  /**
   * \brief Encode the given raw pointcloud into a compressed message.
   * \param[in] raw The input raw pointcloud.
   * \param[in] config Config of the compression (if it has any parameters).
   * \return The output shapeshifter holding the compressed cloud message (if encoding succeeds), or an error message.
   */
  virtual TypedEncodeResult encodeTyped(
      const sensor_msgs::PointCloud2& raw, const Config& config) const = 0;

  /**
   * \brief Encode the given raw pointcloud into a compressed message with default configuration.
   * \param[in] raw The input raw pointcloud.
   * \return The output shapeshifter holding the compressed cloud message (if encoding succeeds), or an error message.
   */
  virtual TypedEncodeResult encodeTyped(const sensor_msgs::PointCloud2& raw) const
  {
    return this->encodeTyped(raw, Config::__getDefault__());
  }

  EncodeResult encode(const sensor_msgs::PointCloud2& raw, const dynamic_reconfigure::Config& configMsg) const override
  {
    Config config = Config::__getDefault__();
    // dynamic_reconfigure has a bug and generates __fromMessage__ with non-const message arg, although it is only read
    if (!config.__fromMessage__(const_cast<dynamic_reconfigure::Config&>(configMsg)))
    {
      return cras::make_unexpected(
          std::string("Wrong configuration options given to " + this->getTransportName() + " transport encoder."));
    }

    auto res = this->encodeTyped(raw, config);
    if (!res)
    {
      return cras::make_unexpected(res.error());
    }
    if (!res.value())
    {
      return cras::nullopt;
    }
    const M& message = res.value().value();
    cras::ShapeShifter shifter;
    cras::msgToShapeShifter(message, shifter);
    return shifter;
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
    typename ReconfigureServer::CallbackType f = boost::bind(&SimplePublisherPlugin<M, Config>::configCb, this, _1, _2);
    reconfigure_server_->setCallback(f);
  }

  void advertiseImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                     const point_cloud_transport::SubscriberStatusCallback& user_connect_cb,
                     const point_cloud_transport::SubscriberStatusCallback& user_disconnect_cb,
                     const ros::VoidPtr& tracked_object, bool latch) override
  {
    base_topic_ = base_topic;
    std::string transport_topic = getTopicToAdvertise(base_topic);
    ros::NodeHandle param_nh(transport_topic);
    simple_impl_ = std::make_unique<SimplePublisherPluginImpl>(param_nh);
    simple_impl_->pub_ = nh.advertise<M>(transport_topic, queue_size,
                                         bindCB(user_connect_cb, &SimplePublisherPlugin::connectCallback),
                                         bindCB(user_disconnect_cb, &SimplePublisherPlugin::disconnectCallback),
                                         tracked_object, latch);

    this->_startDynamicReconfigureServer<Config>();
  }

  //! Generic function for publishing the internal message type.
  typedef boost::function<void(const M&)> PublishFn;

  /**
   * Publish a point cloud using the specified publish function. Must be implemented by
   * the subclass.
   *
   * The PublishFn publishes the transport-specific message type. This indirection allows
   * SimpleSubscriberPlugin to use this function for both normal broadcast publishing and
   * single subscriber publishing (in subscription callbacks).
   */
  virtual void publish(const sensor_msgs::PointCloud2& message, const PublishFn& publish_fn) const
  {
    const auto res = this->encodeTyped(message, config_);
    if (!res)
      ROS_ERROR("Error encoding message by transport %s: %s.", this->getTransportName().c_str(), res.error().c_str());
    if (res.value())
    {
      publish_fn(res.value().value());
    }
  }

  /**
   * Return the communication topic name for a given base topic.
   *
   * Defaults to \<base topic\>/\<transport name\>.
   */
  virtual std::string getTopicToAdvertise(const std::string& base_topic) const
  {
    return base_topic + "/" + getTransportName();
  }

protected:
  /**
   * Function called when a subscriber connects to the internal publisher.
   *
   * Defaults to noop.
   */
  virtual void connectCallback(const ros::SingleSubscriberPublisher& pub)
  {
  }

  /**
   * Function called when a subscriber disconnects from the internal publisher.
   *
   * Defaults to noop.
   */
  virtual void disconnectCallback(const ros::SingleSubscriberPublisher& pub)
  {
  }

  //! Returns the ros::NodeHandle to be used for parameter lookup.
  const ros::NodeHandle& nh() const
  {
    ROS_ASSERT(simple_impl_);
    return simple_impl_->param_nh_;
  }

  /**
   * Returns the internal ros::Publisher.
   * This really only exists so RawPublisher can implement no-copy intraprocess message
   * passing easily.
   */
  const ros::Publisher& getPublisher() const
  {
    ROS_ASSERT(simple_impl_);
    return simple_impl_->pub_;
  }

private:
  struct SimplePublisherPluginImpl
  {
    explicit SimplePublisherPluginImpl(const ros::NodeHandle& nh)
        : param_nh_(nh)
    {
    }

    const ros::NodeHandle param_nh_;
    ros::Publisher pub_;
  };

  std::unique_ptr<SimplePublisherPluginImpl> simple_impl_;

  typedef void (SimplePublisherPlugin::*SubscriberStatusMemFn)(const ros::SingleSubscriberPublisher& pub);

  /**
   * Binds the user callback to subscriberCB(), which acts as an intermediary to expose
   * a publish(PointCloud2) interface to the user while publishing to an internal topic.
   */
  ros::SubscriberStatusCallback bindCB(const point_cloud_transport::SubscriberStatusCallback& user_cb,
                                       SubscriberStatusMemFn internal_cb_fn)
  {
    ros::SubscriberStatusCallback internal_cb = boost::bind(internal_cb_fn, this, _1);
    if (user_cb)
    {
      return boost::bind(&SimplePublisherPlugin::subscriberCB, this, _1, user_cb, internal_cb);
    }
    else
    {
      return internal_cb;
    }
  }

  /**
   * Forms the ros::SingleSubscriberPublisher for the internal communication topic into
   * a point_cloud_transport::SingleSubscriberPublisher for PointCloud2 messages and passes it
   * to the user subscriber status callback.
   */
  void subscriberCB(const ros::SingleSubscriberPublisher& ros_ssp,
                    const point_cloud_transport::SubscriberStatusCallback& user_cb,
                    const ros::SubscriberStatusCallback& internal_cb)
  {
    // First call the internal callback (for sending setup headers, etc.)
    internal_cb(ros_ssp);

    // Construct a function object for publishing sensor_msgs::PointCloud2 through the
    // subclass-implemented publish() using the ros::SingleSubscriberPublisher to send
    // messages of the transport-specific type.
    typedef void (SimplePublisherPlugin::*PublishMemFn)(const sensor_msgs::PointCloud2&, const PublishFn&) const;
    PublishMemFn pub_mem_fn = &SimplePublisherPlugin::publish;
    PointCloud2PublishFn point_cloud_publish_fn = boost::bind(pub_mem_fn, this, _1, bindInternalPublisher(ros_ssp));

    SingleSubscriberPublisher ssp(ros_ssp.getSubscriberName(), getTopic(),
                                  boost::bind(&SimplePublisherPlugin::getNumSubscribers, this),
                                  point_cloud_publish_fn);
    user_cb(ssp);
  }

  typedef boost::function<void(const sensor_msgs::PointCloud2&)> PointCloud2PublishFn;

  /**
   * Returns a function object for publishing the transport-specific message type
   * through some ROS publisher type.
   *
   * @param pub An object with method void publish(const M&)
   */
  template<class PubT>
  PublishFn bindInternalPublisher(const PubT& pub) const
  {
    // Bind PubT::publish(const Message&) as PublishFn
    typedef void (PubT::*InternalPublishMemFn)(const M&) const;
    InternalPublishMemFn internal_pub_mem_fn = &PubT::publish;
    return boost::bind(internal_pub_mem_fn, &pub, _1);
  }
};

}
