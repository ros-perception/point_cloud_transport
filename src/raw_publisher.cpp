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

#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/raw_publisher.h>

class PointCloudTransportPointCloud
{
public:
  sensor_msgs::PointCloud2 PC2_;  //!< ROS header
  const uint8_t* data_ {nullptr};  //!< Data

  /**
   Empty constructor.
   */
  PointCloudTransportPointCloud() = default;

  /**
   Constructor.
   */
  PointCloudTransportPointCloud(const sensor_msgs::PointCloud2& point_cloud, const uint8_t* data)
      : PC2_(point_cloud), data_(data)
  {
  }
};

/// @cond DOXYGEN_IGNORE
namespace ros
{

namespace message_traits
{

template<>
struct MD5Sum<PointCloudTransportPointCloud>
{
  static const char* value()
  {
    return MD5Sum<sensor_msgs::PointCloud2>::value();
  }
  static const char* value(const PointCloudTransportPointCloud&)
  {
    return value();
  }

  static const uint64_t static_value1 = MD5Sum<sensor_msgs::PointCloud2>::static_value1;
  static const uint64_t static_value2 = MD5Sum<sensor_msgs::PointCloud2>::static_value2;
};

template<>
struct DataType<PointCloudTransportPointCloud>
{
  static const char* value() { return DataType<sensor_msgs::PointCloud2>::value(); }
  static const char* value(const PointCloudTransportPointCloud&) { return value(); }
};

template<> struct Definition<PointCloudTransportPointCloud>
{
  static const char* value() { return Definition<sensor_msgs::PointCloud2>::value(); }
  static const char* value(const PointCloudTransportPointCloud&) { return value(); }
};

template<> struct HasHeader<PointCloudTransportPointCloud> : TrueType {};

}  // namespace ros::message_traits

namespace serialization {

template<> struct Serializer<PointCloudTransportPointCloud>
{
  template<typename Stream>
  inline static void write(Stream& stream, const PointCloudTransportPointCloud& m)
  {
    stream.next(m.PC2_.header);
    stream.next(m.PC2_.height);  // height
    stream.next(m.PC2_.width);  // width
    stream.next(m.PC2_.fields);
    stream.next(m.PC2_.is_bigendian);
    stream.next(m.PC2_.point_step);
    stream.next(m.PC2_.row_step);
    size_t data_size = (m.PC2_.point_step*m.PC2_.height*m.PC2_.width);
    stream.next(data_size);
    if (data_size > 0)
      memcpy(stream.advance(data_size), m.data_, data_size);
    stream.next(m.PC2_.is_dense);
  }

  inline static uint32_t serializedLength(const PointCloudTransportPointCloud& m)
  {
    size_t data_size = m.PC2_.point_step*m.PC2_.height*m.PC2_.width;
    // bool serialized in uint8_t => 5*uint32_t + 2*uint8_t = 5*4 + 2*1
    return serializationLength(m.PC2_.header) + 4 + 4 + serializationLength(m.PC2_.fields) + \
      1 + 4 + 4 + 4 + data_size + 1;
  }
};

}  // namespace ros::serialization

}  // namespace ros


namespace point_cloud_transport {

void RawPublisher::publish(const sensor_msgs::PointCloud2& message, const uint8_t* data) const
{
  getPublisher().publish(PointCloudTransportPointCloud(message, data));
}

}  // namespace point_cloud_transport
