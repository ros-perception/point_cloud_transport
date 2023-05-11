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

#include <string>
#include <unordered_map>
#include <vector>

#include <boost/make_shared.hpp>

#include <cras_cpp_common/c_api.h>
#include <pluginlib/class_loader.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/loader_fwds.h>
#include <point_cloud_transport/point_cloud_codec.h>
#include <point_cloud_transport/publisher_plugin.h>
#include <point_cloud_transport/subscriber_plugin.h>

namespace point_cloud_transport
{

struct PointCloudCodec::Impl
{
  point_cloud_transport::PubLoaderPtr enc_loader_;
  point_cloud_transport::SubLoaderPtr dec_loader_;
  std::unordered_map<std::string, std::string> encoders_for_topics_;
  std::unordered_map<std::string, std::string> decoders_for_topics_;

  Impl() :
      enc_loader_(boost::make_shared<PubLoader>("point_cloud_transport", "point_cloud_transport::PublisherPlugin")),
      dec_loader_(boost::make_shared<SubLoader>("point_cloud_transport", "point_cloud_transport::SubscriberPlugin"))
  {
  }
};

PointCloudCodec::PointCloudCodec() : impl_(new Impl)
{
}

bool transportNameMatches(const std::string& lookup_name, const std::string& name, const std::string& suffix)
{
  if (lookup_name == name)
  {
    return true;
  }
  const std::string transport_name = cras::removeSuffix(lookup_name, suffix);
  if (transport_name == name)
  {
    return true;
  }
  const auto parts = cras::split(transport_name, "/");
  if (parts.size() == 2 && parts[1] == name)
  {
    return true;
  }
  return false;
}

boost::shared_ptr<point_cloud_transport::PublisherPlugin> PointCloudCodec::getEncoderByName(
    const std::string& name) const
{
  for (const auto& lookup_name : impl_->enc_loader_->getDeclaredClasses())
  {
    if (transportNameMatches(lookup_name, name, "_pub"))
    {
      return impl_->enc_loader_->createInstance(lookup_name);
    }
  }

  ROS_DEBUG("Failed to find encoder %s.", name.c_str());
  return nullptr;
}

boost::shared_ptr<point_cloud_transport::PublisherPlugin> PointCloudCodec::getEncoderByTopic(
    const std::string& topic, const std::string& datatype) const
{
  if (impl_->encoders_for_topics_.find(topic) != impl_->encoders_for_topics_.end())
  {
    return impl_->enc_loader_->createInstance(impl_->encoders_for_topics_[topic]);
  }

  for (const auto& lookup_name : impl_->enc_loader_->getDeclaredClasses())
  {
    const auto& encoder = impl_->enc_loader_->createInstance(lookup_name);
    if (!encoder)
    {
      continue;
    }
    if (encoder->matchesTopic(topic, datatype))
    {
      impl_->encoders_for_topics_[topic] = lookup_name;
      return encoder;
    }
  }

  ROS_DEBUG("Failed to find encoder for topic %s with data type %s.", topic.c_str(), datatype.c_str());
  return nullptr;
}

boost::shared_ptr<point_cloud_transport::SubscriberPlugin> PointCloudCodec::getDecoderByName(
    const std::string& name) const
{
  for (const auto& lookup_name : impl_->dec_loader_->getDeclaredClasses())
  {
    if (transportNameMatches(lookup_name, name, "_sub"))
    {
      return impl_->dec_loader_->createInstance(lookup_name);
    }
  }

  ROS_DEBUG("Failed to find decoder %s.", name.c_str());
  return nullptr;
}

boost::shared_ptr<point_cloud_transport::SubscriberPlugin> PointCloudCodec::getDecoderByTopic(
    const std::string& topic, const std::string& datatype) const
{
  if (impl_->decoders_for_topics_.find(topic) != impl_->decoders_for_topics_.end())
  {
    return impl_->dec_loader_->createInstance(impl_->decoders_for_topics_[topic]);
  }

  for (const auto& lookup_name : impl_->dec_loader_->getDeclaredClasses())
  {
    const auto& decoder = impl_->dec_loader_->createInstance(lookup_name);
    if (!decoder)
    {
      continue;
    }
    if (decoder->matchesTopic(topic, datatype))
    {
      impl_->decoders_for_topics_[topic] = lookup_name;
      return decoder;
    }
  }

  ROS_DEBUG("Failed to find decoder for topic %s with data type %s.", topic.c_str(), datatype.c_str());
  return nullptr;
}

thread_local PointCloudCodec point_cloud_transport_codec_instance;

}

bool pointCloudTransportCodecsEncode(
    const char* codec,
    sensor_msgs::PointCloud2::_height_type rawHeight,
    sensor_msgs::PointCloud2::_width_type rawWidth,
    size_t rawNumFields,
    const char* rawFieldNames[],
    sensor_msgs::PointField::_offset_type rawFieldOffsets[],
    sensor_msgs::PointField::_datatype_type rawFieldDatatypes[],
    sensor_msgs::PointField::_count_type rawFieldCounts[],
    sensor_msgs::PointCloud2::_is_bigendian_type rawIsBigendian,
    sensor_msgs::PointCloud2::_point_step_type rawPointStep,
    sensor_msgs::PointCloud2::_row_step_type rawRowStep,
    size_t rawDataLength,
    const uint8_t rawData[],
    sensor_msgs::PointCloud2::_is_dense_type rawIsDense,
    cras::allocator_t compressedTypeAllocator,
    cras::allocator_t compressedMd5SumAllocator,
    cras::allocator_t compressedDataAllocator,
    size_t serializedConfigLength,
    const uint8_t serializedConfig[],
    cras::allocator_t errorStringAllocator
)
{
  dynamic_reconfigure::Config config;
  if (serializedConfigLength > 0)
  {
    ros::serialization::IStream data(const_cast<uint8_t*>(serializedConfig), serializedConfigLength);
    try
    {
      ros::serialization::deserialize(data, config);
    }
    catch (const ros::Exception& e)
    {
      cras::outputString(errorStringAllocator, cras::format("Could not deserialize encoder config: %s.", e.what()));
      return false;
    }
  }
  sensor_msgs::PointCloud2 raw;
  raw.height = rawHeight;
  raw.width = rawWidth;
  for (size_t i = 0; i < rawNumFields; ++i)
  {
    sensor_msgs::PointField field;
    field.name = rawFieldNames[i];
    field.offset = rawFieldOffsets[i];
    field.datatype = rawFieldDatatypes[i];
    field.count = rawFieldCounts[i];
    raw.fields.push_back(field);
  }
  raw.is_bigendian = rawIsBigendian;
  raw.point_step = rawPointStep;
  raw.row_step = rawRowStep;
  raw.data.resize(rawDataLength);
  memcpy(raw.data.data(), rawData, rawDataLength);
  raw.is_dense = rawIsDense;

  auto encoder = point_cloud_transport::point_cloud_transport_codec_instance.getEncoderByName(codec);
  if (!encoder)
  {
    cras::outputString(errorStringAllocator, std::string("Could not find encoder for ") + codec);
    return false;
  }

  const auto compressed = encoder->encode(raw, config);
  if (!compressed)
  {
    cras::outputString(errorStringAllocator, compressed.error());
    return false;
  }
  if (!compressed.value())
  {
    return true;
  }

  cras::outputString(compressedTypeAllocator, compressed.value()->getDataType());
  cras::outputString(compressedMd5SumAllocator, compressed.value()->getMD5Sum());
  cras::outputByteBuffer(compressedDataAllocator, cras::getBuffer(compressed->value()), compressed.value()->size());
  return true;
}

bool pointCloudTransportCodecsDecode(
    const char* topicOrCodec,
    const char* compressedType,
    const char* compressedMd5sum,
    size_t compressedDataLength,
    const uint8_t compressedData[],
    sensor_msgs::PointCloud2::_height_type& rawHeight,
    sensor_msgs::PointCloud2::_width_type& rawWidth,
    uint32_t& rawNumFields,
    cras::allocator_t rawFieldNamesAllocator,
    cras::allocator_t rawFieldOffsetsAllocator,
    cras::allocator_t rawFieldDatatypesAllocator,
    cras::allocator_t rawFieldCountsAllocator,
    sensor_msgs::PointCloud2::_is_bigendian_type& rawIsBigEndian,
    sensor_msgs::PointCloud2::_point_step_type& rawPointStep,
    sensor_msgs::PointCloud2::_row_step_type& rawRowStep,
    cras::allocator_t rawDataAllocator,
    sensor_msgs::PointCloud2::_is_dense_type& rawIsDense,
    size_t serializedConfigLength,
    const uint8_t serializedConfig[],
    cras::allocator_t errorStringAllocator
)
{
  dynamic_reconfigure::Config config;
  if (serializedConfigLength > 0)
  {
    ros::serialization::IStream data(const_cast<uint8_t*>(serializedConfig), serializedConfigLength);
    try
    {
      ros::serialization::deserialize(data, config);
    }
    catch (const ros::Exception& e)
    {
      cras::outputString(errorStringAllocator, cras::format("Could not deserialize decoder config: %s.", e.what()));
      return false;
    }
  }

  topic_tools::ShapeShifter compressed;
  compressed.morph(compressedMd5sum, compressedType, "", "");
  cras::resizeBuffer(compressed, compressedDataLength);
  memcpy(cras::getBuffer(compressed), compressedData, compressedDataLength);

  auto decoder = point_cloud_transport::point_cloud_transport_codec_instance.getDecoderByTopic(
      topicOrCodec, compressedType);
  if (!decoder)
  {
    decoder = point_cloud_transport::point_cloud_transport_codec_instance.getDecoderByName(topicOrCodec);
  }
  if (!decoder)
  {
    cras::outputString(errorStringAllocator, std::string("Could not find decoder for ") + topicOrCodec);
    return false;
  }

  const auto res = decoder->decode(compressed, config);
  if (!res)
  {
    cras::outputString(errorStringAllocator, res.error());
    return false;
  }

  if (!res.value())
  {
    return true;
  }

  const auto& raw = res->value();

  rawHeight = raw->height;
  rawWidth = raw->width;
  rawNumFields = raw->fields.size();
  for (size_t i = 0; i < rawNumFields; ++i)
  {
    cras::outputString(rawFieldNamesAllocator, raw->fields[i].name);
    cras::outputByteBuffer(rawFieldOffsetsAllocator, reinterpret_cast<const uint8_t*>(&raw->fields[i].offset), 4);
    cras::outputByteBuffer(rawFieldDatatypesAllocator, reinterpret_cast<const uint8_t*>(&raw->fields[i].datatype), 1);
    cras::outputByteBuffer(rawFieldCountsAllocator, reinterpret_cast<const uint8_t*>(&raw->fields[i].count), 4);
  }
  rawIsBigEndian = raw->is_bigendian;
  rawPointStep = raw->point_step;
  rawRowStep = raw->row_step;
  cras::outputByteBuffer(rawDataAllocator, raw->data);
  rawIsDense = raw->is_dense;
  return true;
}
