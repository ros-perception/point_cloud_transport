/*
 * Copyright (c) 2023, Czech Technical University in Prague
 * Copyright (c) 2019, paplhjak
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef POINT_CLOUD_TRANSPORT__SIMPLE_SUBSCRIBER_PLUGIN_HPP_
#define POINT_CLOUD_TRANSPORT__SIMPLE_SUBSCRIBER_PLUGIN_HPP_


#include <functional>
#include <memory>
#include <string>
#include <type_traits>

#include "rclcpp/serialization.hpp"
#include "rclcpp/subscription.hpp"

#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

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
 * - callback() - processes a message and invoked the user PointCloud2 callback if
 * appropriate.
 *
 * For access to the parameter server and name remappings, use nh().
 *
 * getTopicToSubscribe() controls the name of the internal communication topic. It
 * defaults to \<base topic\>/\<transport name\>.
 */
template<class M>
class SimpleSubscriberPlugin : public SubscriberPlugin
{
public:
  virtual ~SimpleSubscriberPlugin()
  {
  }

  std::string getTopic() const override
  {
    if (impl_) {
      return impl_->sub_->get_topic_name();
    }
    return {};
  }

  template<typename T>
  bool declareParam(const std::string parameter_name, const T value)
  {
    if (impl_) {
      impl_->node_->template declare_parameter<T>(parameter_name, value);
      return true;
    }
    return false;
  }

  void setParamCallback(
    rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType
    param_change_callback)
  {
    if (impl_) {
      impl_->on_set_parameters_callback_handle_ =
        impl_->node_->add_on_set_parameters_callback(param_change_callback);
    }
  }

  uint32_t getNumPublishers() const override
  {
    if (impl_) {
      return static_cast<uint32_t>(impl_->sub_->get_publisher_count());
    }
    return 0;
  }

  void shutdown() override
  {
    impl_.reset();
  }

  void declareParameters() override
  {
  }

  /**
   * \brief Decode the given compressed pointcloud into a raw message.
   * \param[in] compressed The input compressed pointcloud.
   * \return The raw cloud message (if encoding succeeds), or an error message.
   */
  virtual DecodeResult decodeTyped(const M & compressed) const = 0;

  DecodeResult decode(const std::shared_ptr<rclcpp::SerializedMessage>& compressed) const override
  {
    auto msg = std::make_shared<M>();
    try
    {
      auto serializer = rclcpp::Serialization<M>();
      serializer.deserialize_message(compressed.get(), msg.get());
    }
    catch (const std::exception& e)
    {
      return cras::make_unexpected("Error deserializing message for transport decoder: "+std::string(e.what())+".");
    }

    return this->decodeTyped(*msg);
  }

protected:
  /**
   * Process a message. Must be implemented by the subclass.
   */
  virtual void callback(const typename std::shared_ptr<const M> & message, const Callback & user_cb)
  {
    DecodeResult res = this->decodeTyped(*message);
    if (!res) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "point_cloud_transport"), "Error decoding message by transport %s: %s.",
        this->getTransportName().c_str(), res.error().c_str());
    } else if (res.value()) {
      user_cb(res.value().value());
    }
  }

  /**
   * \brief Return the communication topic name for a given base topic.
   *
   * Defaults to \<base topic\>/\<transport name\>.
   */
  std::string getTopicToSubscribe(const std::string & base_topic) const override
  {
    return base_topic + "/" + getTransportName();
  }

  void subscribeImpl(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos) override
  {
    impl_ = std::make_unique<Impl>();
    // Push each group of transport-specific parameters into a separate sub-namespace
    // ros::NodeHandle param_nh(transport_hints.getParameterNH(), getTransportName());
    //
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos);
    impl_->node_ = node;
    impl_->sub_ = node->create_subscription<M>(
      getTopicToSubscribe(base_topic), qos,
      [this, callback](const typename std::shared_ptr<const M> msg) {
        this->callback(msg, callback);
      });
    this->declareParameters();
  }

  void subscribeImplWithOptions(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options)
  {
    impl_ = std::make_unique<Impl>();
    // Push each group of transport-specific parameters into a separate sub-namespace
    // ros::NodeHandle param_nh(transport_hints.getParameterNH(), getTransportName());
    //
    impl_->node_ = node;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos);
    impl_->sub_ = node->create_subscription<M>(
      getTopicToSubscribe(base_topic), qos,
      [this, callback](const typename std::shared_ptr<const M> msg) {
        this->callback(msg, callback);
      },
      options);
    this->declareParameters();
  }

private:
  struct Impl
  {
    rclcpp::SubscriptionBase::SharedPtr sub_;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  };

  std::unique_ptr<Impl> impl_;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__SIMPLE_SUBSCRIBER_PLUGIN_HPP_
