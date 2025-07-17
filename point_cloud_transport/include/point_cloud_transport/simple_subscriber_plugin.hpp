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

#ifndef POINT_CLOUD_TRANSPORT__SIMPLE_SUBSCRIBER_PLUGIN_HPP_
#define POINT_CLOUD_TRANSPORT__SIMPLE_SUBSCRIBER_PLUGIN_HPP_


#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <type_traits>

#include "rclcpp/serialization.hpp"
#include "rclcpp/subscription.hpp"

#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

namespace point_cloud_transport
{

///
/// \brief Base class to simplify implementing most plugins to Subscriber.
///
/// The base class simplifies implementing a SubscriberPlugin in the common case that
/// all communication with the matching PublisherPlugin happens over a single ROS
/// topic using a transport-specific message type. SimpleSubscriberPlugin is templated
/// on the transport-specific message type.
///
/// A subclass needs to implement:
/// - getTransportName() from SubscriberPlugin
/// - decodeTyped()
/// - getDataType()
/// - declareParameters()
///
/// \tparam M Type of the subscribed messages.
///
template<class M>
class SimpleSubscriberPlugin : public SubscriberPlugin
{
public:
  virtual ~SimpleSubscriberPlugin()
  {
  }

  rclcpp::Logger getLogger() const
  {
    if (impl_) {
      return impl_->logger_;
    }
    return rclcpp::get_logger("point_cloud_transport");
  }

  std::string getTopic() const override
  {
    if (impl_) {
      return impl_->sub_->get_topic_name();
    }
    return {};
  }

  //! template function for getting parameter of a given type
  template<typename T>
  bool getParam(const std::string & parameter_name, T & value) const
  {
    if (impl_) {
      uint ns_len = strlen(impl_->node_interfaces_.get_node_base_interface()->get_namespace());
      std::string param_base_name = getTopic().substr(ns_len);
      std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

      std::string param_name = param_base_name + "." + parameter_name;

      rclcpp::Parameter param;
      if (impl_->node_interfaces_.get_node_parameters_interface()
        ->get_parameter(param_name, param))
      {
        value = param.get_value<T>();
        return true;
      }
    }
    return false;
  }

  template<typename T>
  bool declareParam(
    const std::string parameter_name, const T value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (impl_) {
      uint ns_len = strlen(impl_->node_interfaces_.get_node_base_interface()->get_namespace());
      std::string param_base_name = getTopic().substr(ns_len);
      std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

      std::string param_name = param_base_name + "." + parameter_name;

      rcl_interfaces::msg::ParameterDescriptor param_descriptor = parameter_descriptor;
      param_descriptor.name = param_name;

      try {
        impl_->node_interfaces_.get_node_parameters_interface()
        ->declare_parameter(param_name, rclcpp::ParameterValue(value));
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        RCLCPP_DEBUG(
          impl_->logger_, "%s was previously declared",
          param_descriptor.name.c_str());
      }

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
        impl_->node_interfaces_.get_node_parameters_interface()
        ->add_on_set_parameters_callback(param_change_callback);
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

  ///
  /// \brief Decode the given compressed pointcloud into a raw message.
  /// \param[in] compressed The input compressed pointcloud.
  /// \return The raw cloud message (if encoding succeeds), or an error message.
  ///
  virtual DecodeResult decodeTyped(const M & compressed) const = 0;

  DecodeResult decode(const std::shared_ptr<rclcpp::SerializedMessage> & compressed) const override
  {
    auto msg = std::make_shared<M>();
    try {
      auto serializer = rclcpp::Serialization<M>();
      serializer.deserialize_message(compressed.get(), msg.get());
    } catch (const std::exception & e) {
      return tl::make_unexpected(
        "Error deserializing message for transport decoder: " + std::string(
          e.what()) + ".");
    }

    return this->decodeTyped(*msg);
  }

protected:
  ///
  /// \brief Process a message. Must be implemented by the subclass.
  ///
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

  ///
  /// \brief Return the communication topic name for a given base topic.
  ///
  /// Defaults to \<base topic\>/\<transport name\>.
  ///
  std::string getTopicToSubscribe(const std::string & base_topic) const override
  {
    return base_topic + "/" + getTransportName();
  }

  void subscribeImpl(
    std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos) override
  {
    subscribeImpl(*node_interfaces, base_topic, callback,
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos));
  }

  void subscribeImpl(
    std::shared_ptr<rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface>> node_interfaces,
    const std::string & base_topic,
    const Callback & callback,
    rmw_qos_profile_t custom_qos,
    rclcpp::SubscriptionOptions options) override
  {
    subscribeImpl(*node_interfaces, base_topic, callback,
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos), options);
  }

  void subscribeImpl(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    const Callback & callback,
    rclcpp::QoS custom_qos) override
  {
    impl_ = std::make_unique<Impl>(node_interfaces);
    auto node_parameters = node_interfaces.get_node_parameters_interface();
    auto node_topics = node_interfaces.get_node_topics_interface();
    impl_->sub_ = rclcpp::create_subscription<M>(
      node_parameters, node_topics,
      getTopicToSubscribe(base_topic), custom_qos,
      [this, callback](const typename std::shared_ptr<const M> msg) {
        this->callback(msg, callback);
      });
    this->declareParameters();
  }

  void subscribeImpl(
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces,
    const std::string & base_topic,
    const Callback & callback,
    rclcpp::QoS custom_qos,
    rclcpp::SubscriptionOptions options) override
  {
    impl_ = std::make_unique<Impl>(node_interfaces);
    auto node_parameters = node_interfaces.get_node_parameters_interface();
    auto node_topics = node_interfaces.get_node_topics_interface();
    impl_->sub_ = rclcpp::create_subscription<M>(
      node_parameters, node_topics,
      getTopicToSubscribe(base_topic), custom_qos,
      [this, callback](const typename std::shared_ptr<const M> msg) {
        this->callback(msg, callback);
      },
      options);
    this->declareParameters();
  }

private:
  struct Impl
  {
    explicit Impl(
      rclcpp::node_interfaces::NodeInterfaces<
        rclcpp::node_interfaces::NodeBaseInterface,
        rclcpp::node_interfaces::NodeParametersInterface,
        rclcpp::node_interfaces::NodeTopicsInterface,
        rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces)
    :node_interfaces_(node_interfaces),
      logger_(node_interfaces_.get_node_logging_interface()->get_logger())
    {
    }

    rclcpp::SubscriptionBase::SharedPtr sub_;
    rclcpp::node_interfaces::NodeInterfaces<
      rclcpp::node_interfaces::NodeBaseInterface,
      rclcpp::node_interfaces::NodeParametersInterface,
      rclcpp::node_interfaces::NodeTopicsInterface,
      rclcpp::node_interfaces::NodeLoggingInterface> node_interfaces_;
    rclcpp::Logger logger_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
  };

  std::unique_ptr<Impl> impl_;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__SIMPLE_SUBSCRIBER_PLUGIN_HPP_
