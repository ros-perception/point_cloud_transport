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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_codec.hpp>
#include <point_cloud_transport/simple_publisher_plugin.hpp>
#include <point_cloud_transport/simple_subscriber_plugin.hpp>
#include <point_cloud_transport/point_cloud_common.hpp>

namespace point_cloud_transport
{

PointCloudCodec::PointCloudCodec()
: enc_loader_(std::make_shared<PubLoader>(
      "point_cloud_transport", "point_cloud_transport::PublisherPlugin")),
  dec_loader_(std::make_shared<SubLoader>(
      "point_cloud_transport",
      "point_cloud_transport::SubscriberPlugin")) {}

PointCloudCodec::~PointCloudCodec() {}

std::shared_ptr<point_cloud_transport::PublisherPlugin>
PointCloudCodec::getEncoderByName(const std::string & name)
{
  for (const auto & lookup_name : enc_loader_->getDeclaredClasses()) {
    if (transportNameMatches(lookup_name, name, "_pub")) {
      auto encoder = enc_loader_->createSharedInstance(lookup_name);
      return encoder;
    }
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("point_cloud_transport"),
    "Failed to find encoder %s.", name.c_str());
  return nullptr;
}

std::shared_ptr<point_cloud_transport::SubscriberPlugin>
PointCloudCodec::getDecoderByName(const std::string & name)
{
  for (const auto & lookup_name : dec_loader_->getDeclaredClasses()) {
    if (transportNameMatches(lookup_name, name, "_sub")) {
      auto decoder = dec_loader_->createSharedInstance(lookup_name);
      return decoder;
    }
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("point_cloud_transport"),
    "Failed to find decoder %s.", name.c_str());
  return nullptr;
}

void PointCloudCodec::getLoadableTransports(
  std::vector<std::string> & transports,
  std::vector<std::string> & names)
{
  for (const auto & transportPlugin : dec_loader_->getDeclaredClasses()) {
    // If the plugin loads without throwing an exception, add its transport name to the list of
    // valid plugins, otherwise ignore it.
    try {
      auto sub = dec_loader_->createSharedInstance(transportPlugin);
      // Remove the "_sub" at the end of each class name.
      transports.push_back(erase_last_copy(transportPlugin, "_sub"));
      names.push_back(sub->getTransportName());
    } catch (const pluginlib::LibraryLoadException & e) {
      (void)e;
    } catch (const pluginlib::CreateClassException & e) {
      (void)e;
    }
  }
}

void PointCloudCodec::getTopicsToPublish(
  const std::string & baseTopic,
  std::vector<std::string> & transports,
  std::vector<std::string> & topics,
  std::vector<std::string> & names,
  std::vector<std::string> & dataTypes)
{
  for (const auto & transportPlugin : enc_loader_->getDeclaredClasses()) {
    try {
      auto pub = enc_loader_->createSharedInstance(transportPlugin);

      if (pub == nullptr) {
        continue;
      }

      // Remove the "_pub" at the end of each class name.
      transports.push_back(erase_last_copy(transportPlugin, "_pub"));
      names.push_back(pub->getTransportName());
      topics.push_back(pub->getTopicToAdvertise(baseTopic));
      dataTypes.push_back(pub->getDataType());
    } catch (const pluginlib::PluginlibException & e) {
      std::cout << "pointCloudTransportGetTopicsToPublish: " << e.what() << "\n"
                << std::endl;
    }
  }
}

void PointCloudCodec::getTopicToSubscribe(
  const std::string & baseTopic,
  const std::string & transport,
  std::string & topic,
  std::string & name,
  std::string & dataType)
{
  for (const auto & transportPlugin : dec_loader_->getDeclaredClasses()) {
    try {
      auto sub = dec_loader_->createSharedInstance(transportPlugin);

      const auto transportClass = erase_last_copy(transportPlugin, "_sub");
      if (transportClass != transport && sub->getTransportName() != transport) {
        continue;
      }

      if (sub == nullptr) {
        continue;
      }

      topic = sub->getTopicToSubscribe(baseTopic);
      name = sub->getTransportName();
      dataType = sub->getDataType();
      return;
    } catch (const pluginlib::PluginlibException & e) {
      std::cout << "pointCloudTransportGetTopicToSubscribe: " << e.what() << "\n"
                << std::endl;
    }
  }
}

bool PointCloudCodec::encode(
  const std::string & transport_name, const sensor_msgs::msg::PointCloud2 & msg,
  rclcpp::SerializedMessage & serialized_msg)
{
  auto encoder = getEncoderByName(transport_name);
  if (!encoder) {
    return false;
  }

  const auto compressed = encoder->encode(msg);

  if (!compressed) {
    return false;
  }
  if (!compressed.value()) {
    return false;
  }

  serialized_msg = *(compressed.value()->get());
  return true;
}

template<class M>
bool PointCloudCodec::encodeTyped(
  const std::string & transport_name, const sensor_msgs::msg::PointCloud2 & msg,
  M & compressed_msg)
{
  auto encoder = getEncoderByName(transport_name);
  if (!encoder) {
    return false;
  }

  auto typed_encoder =
    reinterpret_cast<point_cloud_transport::SimplePublisherPlugin<M> *>(encoder.get());
  const auto compressed = typed_encoder->encodeTyped(msg);

  if (!compressed) {
    return false;
  }
  if (!compressed.value()) {
    return false;
  }

  compressed_msg = *(compressed.value()->get());
  return true;
}

bool PointCloudCodec::decode(
  const std::string & transport_name,
  const rclcpp::SerializedMessage & serialized_msg,
  sensor_msgs::msg::PointCloud2 & msg)
{
  // decode the serialized msg into a pointcloud
  auto decoder = getDecoderByName(transport_name);

  if (!decoder) {
    return false;
  }

  const auto decompressed = decoder->decode(
    std::make_shared<rclcpp::SerializedMessage>(serialized_msg));

  if (!decompressed) {
    return false;
  }
  if (!decompressed.value()) {
    return false;
  }

  msg = *(decompressed.value()->get());
  return true;
}

template<class M>
bool PointCloudCodec::decodeTyped(
  const std::string & transport_name,
  const M & compressed_msg,
  sensor_msgs::msg::PointCloud2 & msg)
{
  // decode the serialized msg into a pointcloud
  auto decoder = getDecoderByName(transport_name);

  if (!decoder) {
    return false;
  }

  auto typed_decoder =
    reinterpret_cast<point_cloud_transport::SimpleSubscriberPlugin<M> *>(decoder.get());

  const auto decompressed = typed_decoder->decodeTyped(compressed_msg);

  if (!decompressed) {
    return false;
  }
  if (!decompressed.value()) {
    return false;
  }

  msg = *(decompressed.value()->get());
  return true;
}

}  // namespace point_cloud_transport
