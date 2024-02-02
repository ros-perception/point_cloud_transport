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

#ifndef POINT_CLOUD_TRANSPORT__SIMPLE_PUBLISHER_PLUGIN_HPP_
#define POINT_CLOUD_TRANSPORT__SIMPLE_PUBLISHER_PLUGIN_HPP_


#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/serialization.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rcpputils/tl_expected/expected.hpp>

#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/single_subscriber_publisher.hpp>
#include "point_cloud_transport/visibility_control.hpp"

namespace point_cloud_transport
{

///
/// \brief Base class to simplify implementing most plugins to Publisher.
///
/// This base class vastly simplifies implementing a PublisherPlugin in the common
/// case that all communication with the matching SubscriberPlugin happens over a
/// single ROS topic using a transport-specific message type. SimplePublisherPlugin
/// is templated on the transport-specific message type and publisher dynamic
/// reconfigure type.
///
/// A subclass needs to implement:
/// - getTransportName() from PublisherPlugin
/// - encodeTyped()
/// - getDataType()
/// - declareParameters()
///
/// \tparam M Type of the published messages.
///
template<class M>
class SimplePublisherPlugin : public point_cloud_transport::PublisherPlugin
{
public:
  /// \brief Result of cloud encoding. Either the compressed cloud message,
  /// empty value, or error message.
  typedef tl::expected<std::optional<M>, std::string> TypedEncodeResult;

  ~SimplePublisherPlugin()
  {
  }

  rclcpp::Logger getLogger() const
  {
    if (simple_impl_) {
      return simple_impl_->logger_;
    }
    return rclcpp::get_logger("point_cloud_transport");
  }

  //! template function for getting parameter of a given type
  template<typename T>
  bool getParam(const std::string & name, T & value) const
  {
    if (simple_impl_) {
      return simple_impl_->node_->get_parameter(name, value);
    }
    return false;
  }

  template<typename T>
  bool declareParam(
    const std::string parameter_name, const T value,
    const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
    rcl_interfaces::msg::ParameterDescriptor())
  {
    if (simple_impl_) {
      // Declare Parameters
      uint ns_len = simple_impl_->node_->get_effective_namespace().length();
      std::string param_base_name = getTopic().substr(ns_len);
      std::replace(param_base_name.begin(), param_base_name.end(), '/', '.');

      std::string param_name = param_base_name + "." + parameter_name;

      rcl_interfaces::msg::ParameterDescriptor param_descriptor = parameter_descriptor;
      param_descriptor.name = param_name;

      simple_impl_->node_->template declare_parameter<T>(
        param_name, value, param_descriptor);
      return true;
    }
    return false;
  }

  void setParamCallback(
    rclcpp::node_interfaces::NodeParametersInterface::OnSetParametersCallbackType
    param_change_callback)
  {
    if (simple_impl_) {
      simple_impl_->on_set_parameters_callback_handle_ =
        simple_impl_->node_->add_on_set_parameters_callback(param_change_callback);
    }
  }

  uint32_t getNumSubscribers() const override
  {
    if (simple_impl_) {
      return static_cast<uint32_t>(simple_impl_->pub_->get_subscription_count());
    }
    return 0;
  }

  std::string getTopic() const override
  {
    if (simple_impl_) {
      return simple_impl_->pub_->get_topic_name();
    }
    return {};
  }

  void publish(const sensor_msgs::msg::PointCloud2 & message) const override
  {
    if (!simple_impl_ || !simple_impl_->pub_) {
      RCLCPP_ERROR(
        this->getLogger(),
        "Call to publish() on an invalid point_cloud_transport::SimplePublisherPlugin");
      return;
    }

    publish(message, bindInternalPublisher(simple_impl_->pub_.get()));
  }

  void shutdown() override
  {
    simple_impl_.reset();
  }

  ///
  /// \brief Encode the given raw pointcloud into a compressed message.
  /// \param[in] raw The input raw pointcloud.
  /// \return The output rmw serialized msg holding the compressed cloud message
  /// (if encoding succeeds), or an error message.
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  virtual TypedEncodeResult encodeTyped(
    const sensor_msgs::msg::PointCloud2 & raw) const = 0;

  EncodeResult encode(const sensor_msgs::msg::PointCloud2 & raw) const override
  {
    // encode the message using the expected transport method
    auto res = this->encodeTyped(raw);
    if (!res) {
      return tl::make_unexpected(res.error());
    }
    if (!res.value()) {
      return std::nullopt;
    }

    // publish the message (of some unknown type) as a serialized message
    auto serialized_msg_ptr = std::make_shared<rclcpp::SerializedMessage>();
    static rclcpp::Serialization<M> serializer;
    serializer.serialize_message(&(res.value().value()), serialized_msg_ptr.get());
    return serialized_msg_ptr;
  }

protected:
  std::string base_topic_;

  virtual void advertiseImpl(
    std::shared_ptr<rclcpp::Node> node, const std::string & base_topic,
    rmw_qos_profile_t custom_qos,
    const rclcpp::PublisherOptions & options)
  {
    std::string transport_topic = getTopicToAdvertise(base_topic);
    simple_impl_ = std::make_unique<SimplePublisherPluginImpl>(node);

    RCLCPP_DEBUG(node->get_logger(), "getTopicToAdvertise: %s", transport_topic.c_str());
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos);
    simple_impl_->pub_ = node->create_publisher<M>(transport_topic, qos, options);

    base_topic_ = simple_impl_->pub_->get_topic_name();

    this->declareParameters(base_topic_);
  }

  //! Generic function for publishing the internal message type.
  typedef std::function<void (const M &)> PublishFn;

  ///
  /// \brief Publish a point cloud using the specified publish function.
  ///
  /// The PublishFn publishes the transport-specific message type. This indirection allows
  /// SimplePublisherPlugin to use this function for both normal broadcast publishing and
  /// single subscriber publishing (in subscription callbacks).
  ///
  virtual void publish(
    const sensor_msgs::msg::PointCloud2 & message,
    const PublishFn & publish_fn) const
  {
    const auto res = this->encodeTyped(message);
    if (!res) {
      RCLCPP_ERROR(
        this->getLogger(), "Error encoding message by transport %s: %s.",
        this->getTransportName().c_str(), res.error().c_str());
    } else if (res.value()) {
      publish_fn(res.value().value());
    }
  }

  ///
  /// \brief Return the communication topic name for a given base topic.
  ///
  /// Defaults to \<base topic\>/\<transport name\>.
  ///
  std::string getTopicToAdvertise(const std::string & base_topic) const override
  {
    return base_topic + "/" + getTransportName();
  }

private:
  struct SimplePublisherPluginImpl
  {
    explicit SimplePublisherPluginImpl(std::shared_ptr<rclcpp::Node> node)
    : node_(node),
      logger_(node->get_logger())
    {
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
    rclcpp::Logger logger_;
    typename rclcpp::Publisher<M>::SharedPtr pub_;
  };

  std::unique_ptr<SimplePublisherPluginImpl> simple_impl_;

  typedef std::function<void (const sensor_msgs::msg::PointCloud2 &)> PointCloudPublishFn;

  ///
  /// \brief Returns a function object for publishing the transport-specific message type
  /// through some ROS publisher type.
  /// \param pub An object with method void publish(const M&)
  ///
  template<class PubT>
  PublishFn bindInternalPublisher(PubT * pub) const
  {
    // Bind PubT::publish(const Message&) as PublishFn
    typedef void (PubT::* InternalPublishMemFn)(const M &);
    InternalPublishMemFn internal_pub_mem_fn = &PubT::publish;
    return std::bind(internal_pub_mem_fn, pub, std::placeholders::_1);
  }
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__SIMPLE_PUBLISHER_PLUGIN_HPP_
