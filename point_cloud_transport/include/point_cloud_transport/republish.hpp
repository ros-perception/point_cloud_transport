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

#ifndef POINT_CLOUD_TRANSPORT__REPUBLISH_HPP_
#define POINT_CLOUD_TRANSPORT__REPUBLISH_HPP_

#include <memory>

#include "point_cloud_transport/visibility_control.hpp"

#include <point_cloud_transport/point_cloud_transport.hpp>

#include <rclcpp/node.hpp>

namespace point_cloud_transport
{

class Republisher : public rclcpp::Node
{
public:
  //! Constructor
  POINT_CLOUD_TRANSPORT_PUBLIC
  explicit Republisher(const rclcpp::NodeOptions & options);

private:
  POINT_CLOUD_TRANSPORT_PUBLIC
  void initialize();

  std::shared_ptr<point_cloud_transport::PointCloudTransport> pct;
  rclcpp::TimerBase::SharedPtr timer_;
  bool initialized_{false};
  point_cloud_transport::Subscriber sub;
  std::shared_ptr<point_cloud_transport::PublisherPlugin> pub;
  std::shared_ptr<point_cloud_transport::Publisher> simple_pub;
};

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__REPUBLISH_HPP_
