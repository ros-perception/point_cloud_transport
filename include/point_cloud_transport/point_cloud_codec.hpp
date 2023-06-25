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

#ifndef POINT_CLOUD_TRANSPORT__POINT_CLOUD_CODEC_HPP_
#define POINT_CLOUD_TRANSPORT__POINT_CLOUD_CODEC_HPP_

#include <point_cloud_transport/c_api.h>

#include <memory>
#include <string>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

namespace point_cloud_transport
{

/**
* Advertise and subscribe to PointCloud2 topics.
*
* PointCloudTransport is analogous to ros::NodeHandle in that it contains advertise() and
* subscribe() functions for creating advertisements and subscriptions of PointCloud2 topics.
*/

class PointCloudCodec
{
public:
  //! Constructor
  PointCloudCodec();

  /**
   * @brief Get a shared pointer to an instance of a publisher plugin given its transport name (publishers encode messages).
   * e.g. if you want the raw encoder, call getEncoderByName("raw").
   * @param name The name of the transport to load.
  */
  std::shared_ptr<point_cloud_transport::PublisherPlugin> getEncoderByName(const std::string & name)
  const;

  std::shared_ptr<point_cloud_transport::PublisherPlugin> getEncoderByTopic(
    const std::string & topic, const std::string & datatype) const;

  /**
   * @brief Get a shared pointer to an instance of a publisher plugin given its transport name (subscribers decode messages).
   * e.g. if you want the raw decoder, call getDecoderByName("raw").
   * @param name The name of the transport to load.
  */
  std::shared_ptr<point_cloud_transport::SubscriberPlugin> getDecoderByName(
    const std::string & name) const;

  std::shared_ptr<point_cloud_transport::SubscriberPlugin> getDecoderByTopic(
    const std::string & topic, const std::string & datatype) const;

private:
  struct Impl;
  typedef std::shared_ptr<Impl> ImplPtr;
  typedef std::weak_ptr<Impl> ImplWPtr;

  ImplPtr impl_;
};

}  // namespace point_cloud_transport

extern "C" bool pointCloudTransportCodecsEncode(
  const char * codec,
  sensor_msgs::msg::PointCloud2::_height_type rawHeight,
  sensor_msgs::msg::PointCloud2::_width_type rawWidth,
  size_t rawNumFields,
  const char * rawFieldNames[],
  sensor_msgs::PointField::_offset_type rawFieldOffsets[],
  sensor_msgs::PointField::_datatype_type rawFieldDatatypes[],
  sensor_msgs::PointField::_count_type rawFieldCounts[],
  sensor_msgs::msg::PointCloud2::_is_bigendian_type rawIsBigendian,
  sensor_msgs::msg::PointCloud2::_point_step_type rawPointStep,
  sensor_msgs::msg::PointCloud2::_row_step_type rawRowStep,
  size_t rawDataLength,
  const uint8_t rawData[],
  sensor_msgs::msg::PointCloud2::_is_dense_type rawIsDense,
  cras::allocator_t compressedTypeAllocator,
  cras::allocator_t compressedMd5SumAllocator,
  cras::allocator_t compressedDataAllocator,
  size_t serializedConfigLength,
  const uint8_t serializedConfig[],
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
);

extern "C" bool pointCloudTransportCodecsDecode(
  const char * topicOrCodec,
  const char * compressedType,
  const char * compressedMd5sum,
  size_t compressedDataLength,
  const uint8_t compressedData[],
  sensor_msgs::msg::PointCloud2::_height_type & rawHeight,
  sensor_msgs::msg::PointCloud2::_width_type & rawWidth,
  uint32_t & rawNumFields,
  cras::allocator_t rawFieldNamesAllocator,
  cras::allocator_t rawFieldOffsetsAllocator,
  cras::allocator_t rawFieldDatatypesAllocator,
  cras::allocator_t rawFieldCountsAllocator,
  sensor_msgs::msg::PointCloud2::_is_bigendian_type & rawIsBigEndian,
  sensor_msgs::msg::PointCloud2::_point_step_type & rawPointStep,
  sensor_msgs::msg::PointCloud2::_row_step_type & rawRowStep,
  cras::allocator_t rawDataAllocator,
  sensor_msgs::msg::PointCloud2::_is_dense_type & rawIsDense,
  size_t serializedConfigLength,
  const uint8_t serializedConfig[],
  cras::allocator_t errorStringAllocator,
  cras::allocator_t logMessagesAllocator
);
#endif  // POINT_CLOUD_TRANSPORT__POINT_CLOUD_CODEC_HPP_
