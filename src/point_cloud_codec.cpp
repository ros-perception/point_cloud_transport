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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/loader_fwds.hpp>
#include <point_cloud_transport/point_cloud_codec.hpp>
#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

namespace point_cloud_transport
{

  PointCloudCodec::PointCloudCodec() : enc_loader_(std::make_shared<PubLoader>(
                                           "point_cloud_transport", "point_cloud_transport::PublisherPlugin")),
                                       dec_loader_(std::make_shared<SubLoader>(
                                           "point_cloud_transport",
                                           "point_cloud_transport::SubscriberPlugin")) {}

  PointCloudCodec::~PointCloudCodec() {}

  static PointCloudCodec *kImpl = new PointCloudCodec();

  std::shared_ptr<point_cloud_transport::PublisherPlugin>
  getEncoderByName(const std::string &name)
  {
    for (const auto &lookup_name : kImpl->enc_loader_->getDeclaredClasses())
    {
      if (transportNameMatches(lookup_name, name, "_pub"))
      {
        auto encoder = kImpl->enc_loader_->createSharedInstance(lookup_name);
        return encoder;
      }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("point_cloud_transport"),
                 "Failed to find encoder %s.", name.c_str());
    return nullptr;
  }

  std::shared_ptr<point_cloud_transport::PublisherPlugin>
  getEncoderByTopic(const std::string &topic,
                    const std::string &datatype)
  {
    if (kImpl->encoders_for_topics_.find(topic) !=
        kImpl->encoders_for_topics_.end())
    {
      auto encoder = kImpl->enc_loader_->createSharedInstance(
          kImpl->encoders_for_topics_[topic]);
      return encoder;
    }

    for (const auto &lookup_name : kImpl->enc_loader_->getDeclaredClasses())
    {
      const auto &encoder = kImpl->enc_loader_->createSharedInstance(lookup_name);
      if (!encoder)
      {
        continue;
      }
      if (encoder->matchesTopic(topic, datatype))
      {
        kImpl->encoders_for_topics_[topic] = lookup_name;
        return encoder;
      }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("point_cloud_transport"),
                 "Failed to find encoder for topic %s with data type %s.",
                 topic.c_str(), datatype.c_str());
    return nullptr;
  }

  std::shared_ptr<point_cloud_transport::SubscriberPlugin>
  getDecoderByName(const std::string &name)
  {
    for (const auto &lookup_name : kImpl->dec_loader_->getDeclaredClasses())
    {
      if (transportNameMatches(lookup_name, name, "_sub"))
      {
        auto decoder = kImpl->dec_loader_->createSharedInstance(lookup_name);
        return decoder;
      }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("point_cloud_transport"),
                 "Failed to find decoder %s.", name.c_str());
    return nullptr;
  }

  std::shared_ptr<point_cloud_transport::SubscriberPlugin>
  getDecoderByTopic(const std::string &topic,
                    const std::string &datatype)
  {
    if (kImpl->decoders_for_topics_.find(topic) !=
        kImpl->decoders_for_topics_.end())
    {
      auto decoder = kImpl->dec_loader_->createSharedInstance(
          kImpl->decoders_for_topics_[topic]);
      return decoder;
    }

    for (const auto &lookup_name : kImpl->dec_loader_->getDeclaredClasses())
    {
      const auto &decoder = kImpl->dec_loader_->createSharedInstance(lookup_name);
      if (!decoder)
      {
        continue;
      }
      if (decoder->matchesTopic(topic, datatype))
      {
        kImpl->decoders_for_topics_[topic] = lookup_name;
        return decoder;
      }
    }

    RCLCPP_DEBUG(rclcpp::get_logger("point_cloud_transport"),
                 "Failed to find decoder for topic %s with data type %s.",
                 topic.c_str(), datatype.c_str());

    return nullptr;
  }

  bool pointCloudTransportCodecsEncode(
      const std::string &transport_name, const sensor_msgs::msg::PointCloud2 &msg,
      rclcpp::SerializedMessage &serialized_msg)
  {
    auto encoder = getEncoderByName(transport_name);
    if (!encoder)
    {
      return false;
    }

    const auto compressed = encoder->encode(msg);

    if (!compressed)
    {
      return false;
    }
    if (!compressed.value())
    {
      return false;
    }

    serialized_msg = *(compressed.value()->get());
    return true;
  }

  bool pointCloudTransportCodecsDecode(
      const std::string &transport_name,
      const rclcpp::SerializedMessage &serialized_msg,
      sensor_msgs::msg::PointCloud2 &msg)
  {

    // decode the serialized msg into a pointcloud
    auto decoder = getDecoderByName(transport_name);

    if (!decoder)
    {
      return false;
    }

    const auto decompressed = decoder->decode(
        std::make_shared<rclcpp::SerializedMessage>(serialized_msg));

    if (!decompressed)
    {
      return false;
    }
    if (!decompressed.value())
    {
      return false;
    }

    msg = *(decompressed.value()->get());
    return true;
  }

} // namespace point_cloud_transport
