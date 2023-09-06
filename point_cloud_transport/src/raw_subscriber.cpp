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

#include <string>

#include <point_cloud_transport/raw_subscriber.hpp>

namespace point_cloud_transport
{

std::string RawSubscriber::getTransportName() const
{
  return "raw";
}

std::string RawSubscriber::getTopicToSubscribe(const std::string & base_topic) const
{
  return base_topic;
}

std::string RawSubscriber::getDataType() const
{
  return "sensor_msgs/msg/PointCloud2";
}

void RawSubscriber::declareParameters()
{
}

SubscriberPlugin::DecodeResult RawSubscriber::decodeTyped(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & compressed) const
{
  return compressed;
}

SubscriberPlugin::DecodeResult RawSubscriber::decodeTyped(
  const sensor_msgs::msg::PointCloud2 & compressed) const
{
  auto compressedPtr = std::make_shared<const sensor_msgs::msg::PointCloud2>(compressed);
  return this->decodeTyped(compressedPtr);
}


void RawSubscriber::callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & message,
  const SubscriberPlugin::Callback & user_cb)
{
  user_cb(message);
}

}  // namespace point_cloud_transport
