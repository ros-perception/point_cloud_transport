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

#include <pluginlib/class_loader.hpp>
#include <pluginlib/exceptions.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/loader_fwds.hpp>
#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/single_subscriber_publisher.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>
#include <point_cloud_transport/transport_hints.hpp>

namespace point_cloud_transport
{

PointCloudTransportLoader::PointCloudTransportLoader()
: pub_loader_(std::make_shared<PubLoader>(
      "point_cloud_transport",
      "point_cloud_transport::PublisherPlugin")),
  sub_loader_(std::make_shared<SubLoader>(
      "point_cloud_transport",
      "point_cloud_transport::SubscriberPlugin"))
{
}

PointCloudTransportLoader::~PointCloudTransportLoader()
{
}

std::vector<std::string> PointCloudTransportLoader::getDeclaredTransports() const
{
  auto transports = sub_loader_->getDeclaredClasses();

  // Remove the "_sub" at the end of each class name.
  for (auto & transport : transports) {
    transport = erase_last_copy(transport, "_sub");
  }
  return transports;
}

std::unordered_map<std::string,
  std::string> PointCloudTransportLoader::getLoadableTransports() const
{
  std::unordered_map<std::string, std::string> loadableTransports;

  for (const auto & transportPlugin : sub_loader_->getDeclaredClasses()) {
    // If the plugin loads without throwing an exception, add its transport name to the list of
    // valid plugins, otherwise ignore it.
    try {
      auto sub = sub_loader_->createSharedInstance(transportPlugin);
      // Remove the "_sub" at the end of each class name.
      loadableTransports[erase_last_copy(transportPlugin, "_sub")] = sub->getTransportName();
    } catch (const pluginlib::LibraryLoadException & e) {
      (void) e;
    } catch (const pluginlib::CreateClassException & e) {
      (void) e;
    }
  }

  return loadableTransports;
}

PubLoaderPtr PointCloudTransportLoader::getPublisherLoader() const
{
  return pub_loader_;
}

SubLoaderPtr PointCloudTransportLoader::getSubscriberLoader() const
{
  return sub_loader_;
}

thread_local std::unique_ptr<point_cloud_transport::PointCloudTransportLoader> loader;

point_cloud_transport::PointCloudTransportLoader & getLoader()
{
  if (point_cloud_transport::loader == nullptr) {
    point_cloud_transport::loader =
      std::make_unique<point_cloud_transport::PointCloudTransportLoader>();
  }
  return *point_cloud_transport::loader;
}

PointCloudTransport::PointCloudTransport(rclcpp::Node::SharedPtr node)
{
  PointCloudTransportLoader();
  node_ = node;
}


// TODO(anyone): Are these needed?
// void pointCloudTransportGetLoadableTransports(
// cras::allocator_t transportAllocator, cras::allocator_t nameAllocator)
// {
//   for (const auto& transport : getLoader().getLoadableTransports())
//   {
//     cras::outputString(transportAllocator, transport.first);
//     cras::outputString(nameAllocator, transport.second);
//   }
// }

// void pointCloudTransportGetTopicsToPublish(const char* baseTopic,
//                                            cras::allocator_t transportAllocator,
//                                            cras::allocator_t nameAllocator,
//                                            cras::allocator_t topicAllocator,
//                                            cras::allocator_t dataTypeAllocator,
//                                            cras::allocator_t configTypeAllocator)
// {
//   auto pubLoader = getLoader().getPublisherLoader();
//   for (const auto& transportPlugin : pubLoader->getDeclaredClasses())
//   {
//     try
//     {
//       auto pub = pubLoader->createSharedInstance(transportPlugin);
//       auto singleTopicPub =
// std::dynamic_pointer_cast<point_cloud_transport::SingleTopicPublisherPlugin>(pub);
//       if (singleTopicPub == nullptr)
//         continue;
//       // Remove the "_pub" at the end of each class name.
//       cras::outputString(transportAllocator, erase_last_copy(transportPlugin, "_pub"));
//       cras::outputString(nameAllocator, singleTopicPub->getTransportName());
//       cras::outputString(topicAllocator, singleTopicPub->getTopicToAdvertise(baseTopic));
//       cras::outputString(dataTypeAllocator, singleTopicPub->getDataType());
//       cras::outputString(configTypeAllocator, singleTopicPub->getConfigDataType());
//     }
//     catch (const pluginlib::PluginlibException& e)
//     {
//     }
//   }
// }

// void pointCloudTransportGetTopicToSubscribe(const char* baseTopic,
//                                             const char* transport,
//                                             cras::allocator_t nameAllocator,
//                                             cras::allocator_t topicAllocator,
//                                             cras::allocator_t dataTypeAllocator,
//                                             cras::allocator_t configTypeAllocator)
// {
//   auto subLoader = getLoader().getSubscriberLoader();
//   for (const auto& transportPlugin : subLoader->getDeclaredClasses())
//   {
//     try
//     {
//       auto sub = subLoader->createSharedInstance(transportPlugin);

//       const auto transportClass = erase_last_copy(transportPlugin, "_sub");
//       if (transportClass != transport && sub->getTransportName() != transport)
//         continue;

//       auto singleTopicSub =
// std::dynamic_pointer_cast<point_cloud_transport::SingleTopicSubscriberPlugin>(sub);
//       if (singleTopicSub == nullptr)
//         continue;

//       cras::outputString(nameAllocator, singleTopicSub->getTransportName());
//       cras::outputString(topicAllocator, singleTopicSub->getTopicToSubscribe(baseTopic));
//       cras::outputString(dataTypeAllocator, singleTopicSub->getDataType());
//       cras::outputString(configTypeAllocator, singleTopicSub->getConfigDataType());
//       return;
//     }
//     catch (const pluginlib::PluginlibException& e)
//     {
//     }
//   }
// }

}  // namespace point_cloud_transport
