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

#ifndef POINT_CLOUD_TRANSPORT__POINT_CLOUD_CODEC_HPP_
#define POINT_CLOUD_TRANSPORT__POINT_CLOUD_CODEC_HPP_

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <pluginlib/exceptions.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/loader_fwds.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>
#include "point_cloud_transport/visibility_control.hpp"

namespace point_cloud_transport
{

//! Class to expose all the functionality of pointcloud transport (encode/decode msgs)
//! without needing to spin a node.

class PointCloudCodec
{
public:
  //! Constructor
  POINT_CLOUD_TRANSPORT_PUBLIC
  PointCloudCodec();

  //! Destructor
  POINT_CLOUD_TRANSPORT_PUBLIC
  virtual ~PointCloudCodec();

  /// \brief Get a shared pointer to an instance of a publisher plugin given its
  /// transport name (publishers encode messages).
  /// e.g. if you want the raw encoder, call getEncoderByName("raw").
  ///
  /// \param name The name of the transport to load.
  /// \returns A shared pointer to the publisher plugin.
  POINT_CLOUD_TRANSPORT_PUBLIC
  std::shared_ptr<point_cloud_transport::PublisherPlugin> getEncoderByName(
    const std::string & name);

  /// \brief Get a shared pointer to an instance of a publisher plugin given its transport name
  /// (subscribers decode messages).
  /// e.g. if you want the raw decoder, call getDecoderByName("raw").
  ///
  /// \param name The name of the transport to load.
  /// \returns A shared pointer to the subscriber plugin.
  POINT_CLOUD_TRANSPORT_PUBLIC
  std::shared_ptr<point_cloud_transport::SubscriberPlugin> getDecoderByName(
    const std::string & name);

  ///
  /// \brief Get a list of all the transports that can be loaded.
  ///
  /// \param[out] transports Vector of the loadable transport plugins.
  /// \param[out] names Vector of string identifieries for the transport provided by each plugin
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  void getLoadableTransports(
    std::vector<std::string> & transports,
    std::vector<std::string> & names);

  ///
  /// \brief Get a list of all the transport plugins, topics, transport names, and their data
  /// types that can be loaded.
  ///
  /// \param[in] baseTopic The base topic to use for the transport.
  /// \param[out] transports Vector of the loadable transport plugins.
  /// \param[out] topics Vector of the topics that can be published.
  /// \param[out] names Vector of string identifieries for the transport provided by each plugin
  /// \param[out] dataTypes Vector of the data types the transports encode a PointCloud2 into
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  void getTopicsToPublish(
    const std::string & baseTopic,
    std::vector<std::string> & transports,
    std::vector<std::string> & topics,
    std::vector<std::string> & names,
    std::vector<std::string> & dataTypes);

  ///
  /// \brief Get the topic, transport name, and data type that a given topic is published on for a
  /// particular transport plugin.
  ///
  /// \param[in] baseTopic The base topic to use for the transport.
  /// \param[in] transport The transport plugin to load.
  /// \param[out] topic The topic that should be subscribed to.
  /// \param[out] name String identifier for the transport provided by the plugin
  /// \param[out] dataType The data type the transport encodes a PointCloud2 into
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  void getTopicToSubscribe(
    const std::string & baseTopic,
    const std::string & transport,
    std::string & topic,
    std::string & name,
    std::string & dataType);

  ///
  /// \brief Encode a PointCloud2 message into a serialized message
  /// using the specified transport plugin. The underlying type
  /// of the serialized message is determined by the transport plugin,
  /// but doesnt need to be known by this function.
  ///
  /// \param[in] transport_name The name of the transport plugin to use.
  /// \param[in] msg The message to encode.
  /// \param[out] serialized_msg The serialized message to store the encoded message in.
  /// \returns True if the message was successfully encoded, false otherwise.
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  bool encode(
    const std::string & transport_name,
    const sensor_msgs::msg::PointCloud2 & msg,
    rclcpp::SerializedMessage & serialized_msg);

  ///
  /// \brief Encode a PointCloud2 message into some compressed message type
  /// using the specified transport plugin. The compressed message type
  /// is determined by the transport plugin.
  ///
  /// \param[in] transport_name The name of the transport plugin to use.
  /// \param[in] msg The message to encode.
  /// \param[out] compressed_msg The compressed message to store the encoded message in.
  /// \returns True if the message was successfully encoded, false otherwise.
  ///
  template<class M>
  POINT_CLOUD_TRANSPORT_PUBLIC
  bool encodeTyped(
    const std::string & transport_name,
    const sensor_msgs::msg::PointCloud2 & msg,
    M & compressed_msg);

  /// \brief Decode a serialized message into a PointCloud2
  /// using the specified transport plugin. The underlying type
  /// of the serialized message is determined by the transport plugin,
  /// but doesnt need to be known by this function.
  ///
  /// \param[in] transport_name The name of the transport plugin to use.
  /// \param[in] serialized_msg The serialized message to decode.
  /// \param[out] msg The message to store the decoded output in.
  /// \returns True if the message was successfully decoded, false otherwise.
  ///
  POINT_CLOUD_TRANSPORT_PUBLIC
  bool decode(
    const std::string & transport_name,
    const rclcpp::SerializedMessage & serialized_msg,
    sensor_msgs::msg::PointCloud2 & msg);

  /// \brief Decode some compressed message type
  /// into a PointCloud2 based on the specified transport plugin. The compressed message type
  /// is determined by the transport plugin.
  ///
  /// \param[in] transport_name The name of the transport plugin to use.
  /// \param[in] compressed_msg The compressed message to decode.
  /// \param[out] msg The message to store the decoded output in.
  /// \returns True if the message was successfully decoded, false otherwise.
  ///
  template<class M>
  POINT_CLOUD_TRANSPORT_PUBLIC
  bool decodeTyped(
    const std::string & transport_name,
    const M & compressed_msg,
    sensor_msgs::msg::PointCloud2 & msg);

private:
  point_cloud_transport::PubLoaderPtr enc_loader_;
  point_cloud_transport::SubLoaderPtr dec_loader_;
};

}  // namespace point_cloud_transport

#endif  // POINT_CLOUD_TRANSPORT__POINT_CLOUD_CODEC_HPP_
