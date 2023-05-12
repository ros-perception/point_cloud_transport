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

#include <set>
#include <string>
#include <vector>

#include <boost/algorithm/string/erase.hpp>
#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/exception.h>
#include <point_cloud_transport/publisher.h>
#include <point_cloud_transport/publisher_plugin.h>
#include <point_cloud_transport/single_subscriber_publisher.h>

namespace point_cloud_transport
{

struct Publisher::Impl
{
  Impl() : unadvertised_(false)
  {
  }

  ~Impl()
  {
    shutdown();
  }

  uint32_t getNumSubscribers() const
  {
    uint32_t count = 0;
    for (const auto& pub : publishers_)
      count += pub->getNumSubscribers();
    return count;
  }

  std::string getTopic() const
  {
    return base_topic_;
  }

  bool isValid() const
  {
    return !unadvertised_;
  }

  void shutdown()
  {
    if (!unadvertised_)
    {
      unadvertised_ = true;
      for (auto& pub : publishers_)
        pub->shutdown();
      publishers_.clear();
    }
  }

  void subscriberCB(const point_cloud_transport::SingleSubscriberPublisher& plugin_pub,
                    const point_cloud_transport::SubscriberStatusCallback& user_cb)
  {
    point_cloud_transport::SingleSubscriberPublisher ssp(
        plugin_pub.getSubscriberName(), getTopic(), boost::bind(&Publisher::Impl::getNumSubscribers, this),
        plugin_pub.publish_fn_);
    user_cb(ssp);
  }

  std::string base_topic_;
  PubLoaderPtr loader_;
  std::vector<boost::shared_ptr<point_cloud_transport::PublisherPlugin> > publishers_;
  bool unadvertised_;
};

Publisher::Publisher() = default;

Publisher::Publisher(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                     const point_cloud_transport::SubscriberStatusCallback& connect_cb,
                     const point_cloud_transport::SubscriberStatusCallback& disconnect_cb,
                     const ros::VoidPtr& tracked_object, bool latch,
                     const point_cloud_transport::PubLoaderPtr& loader)
    : impl_(new Impl)
{
  // Resolve the name explicitly because otherwise the compressed topics don't remap properly
  impl_->base_topic_ = nh.resolveName(base_topic);
  impl_->loader_ = loader;

  // sequence container which encapsulates dynamic size arrays
  std::vector<std::string> blacklist_vec;
  // call to parameter server
  nh.getParam(impl_->base_topic_ + "/disable_pub_plugins", blacklist_vec);

  // set
  std::set<std::string> blacklist(blacklist_vec.begin(), blacklist_vec.end());

  for (const auto& lookup_name : loader->getDeclaredClasses())
  {
    const std::string transport_name = boost::erase_last_copy(lookup_name, "_pub");
    if (blacklist.find(transport_name) != blacklist.end())
      continue;

    try
    {
      auto pub = loader->createInstance(lookup_name);
      impl_->publishers_.push_back(pub);
      pub->advertise(nh, impl_->base_topic_, queue_size, rebindCB(connect_cb),
                     rebindCB(disconnect_cb), tracked_object, latch);
    }
    catch (const std::runtime_error& e)
    {
      ROS_ERROR("Failed to load plugin %s, error string: %s", lookup_name.c_str(), e.what());
    }
  }

  if (impl_->publishers_.empty())
  {
    throw point_cloud_transport::Exception("No plugins found! Does `rospack plugins --attrib=plugin "
                                            "point_cloud_transport` find any packages?");
  }
}

uint32_t Publisher::getNumSubscribers() const
{
  if (impl_ && impl_->isValid())
    return impl_->getNumSubscribers();
  return 0;
}

std::string Publisher::getTopic() const
{
  if (impl_)
    return impl_->getTopic();
  return {};
}

void Publisher::publish(const sensor_msgs::PointCloud2& message) const
{
  if (!impl_ || !impl_->isValid())
  {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid point_cloud_transport::Publisher");
    return;
  }

  for (const auto& pub : impl_->publishers_)
  {
    if (pub->getNumSubscribers() > 0)
    {
      pub->publish(message);
    }
  }
}

void Publisher::publish(const sensor_msgs::PointCloud2ConstPtr& message) const
{
  if (!impl_ || !impl_->isValid())
  {
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid point_cloud_transport::Publisher");
    return;
  }

  for (const auto& pub : impl_->publishers_)
  {
    if (pub->getNumSubscribers() > 0)
    {
      pub->publish(message);
    }
  }
}

void Publisher::shutdown()
{
  if (impl_)
  {
    impl_->shutdown();
    impl_.reset();
  }
}

Publisher::operator void*() const
{
  return (impl_ && impl_->isValid()) ? reinterpret_cast<void*>(1) : nullptr;
}

void Publisher::weakSubscriberCb(const ImplWPtr& impl_wptr,
                                 const point_cloud_transport::SingleSubscriberPublisher& plugin_pub,
                                 const point_cloud_transport::SubscriberStatusCallback& user_cb)
{
  if (ImplPtr impl = impl_wptr.lock())
  {
    impl->subscriberCB(plugin_pub, user_cb);
  }
}

SubscriberStatusCallback Publisher::rebindCB(const point_cloud_transport::SubscriberStatusCallback& user_cb)
{
  // Note: the subscriber callback must be bound to the internal Impl object, not
  // 'this'. Due to copying behavior the Impl object may outlive the original Publisher
  // instance. But it should not outlive the last Publisher, so we use a weak_ptr.
  if (user_cb)
  {
    ImplWPtr impl_wptr(impl_);
    return boost::bind(&Publisher::weakSubscriberCb, impl_wptr, _1, user_cb);
  }

  return {};
}

}
