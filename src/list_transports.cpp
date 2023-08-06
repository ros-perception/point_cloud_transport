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

#include <iostream>
#include <cstdio>
#include <map>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <pluginlib/exceptions.hpp>

#include <point_cloud_transport/point_cloud_common.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

enum PluginStatus
{
  SUCCESS, CREATE_FAILURE, LIB_LOAD_FAILURE, DOES_NOT_EXIST
};

struct TransportDesc
{
  TransportDesc()
  : pub_status(DOES_NOT_EXIST), sub_status(DOES_NOT_EXIST)
  {
  }

  std::string package_name;
  std::string pub_name;
  PluginStatus pub_status;
  std::string sub_name;
  PluginStatus sub_status;
};

int main(int /*argc*/, char ** /*argv*/)
{
  pluginlib::ClassLoader<point_cloud_transport::PublisherPlugin> pub_loader(
    "point_cloud_transport",
    "point_cloud_transport::PublisherPlugin");
  pluginlib::ClassLoader<point_cloud_transport::SubscriberPlugin> sub_loader(
    "point_cloud_transport",
    "point_cloud_transport::SubscriberPlugin");
  typedef std::map<std::string, TransportDesc> StatusMap;
  StatusMap transports;

  for (const auto & lookup_name : pub_loader.getDeclaredClasses()) {
    std::cout << "Lookup name: " << lookup_name.c_str() << std::endl;
    std::string transport_name = point_cloud_transport::erase_last_copy(lookup_name, "_pub");
    std::cout << "Transport name: " << transport_name.c_str() << std::endl;
    transports[transport_name].pub_name = lookup_name;
    transports[transport_name].package_name = pub_loader.getClassPackage(lookup_name);
    try {
      auto pub = pub_loader.createUniqueInstance(lookup_name);
      transports[transport_name].pub_status = SUCCESS;
    } catch (const pluginlib::LibraryLoadException & e) {
      transports[transport_name].pub_status = LIB_LOAD_FAILURE;
      std::cout << "LibraryLoadException: " << e.what() << std::endl;
    } catch (const pluginlib::CreateClassException & e) {
      transports[transport_name].pub_status = CREATE_FAILURE;
      std::cout << "CreateClassException: " << e.what() << std::endl;
    }
  }

  for (const auto & lookup_name : sub_loader.getDeclaredClasses()) {
    std::string transport_name = point_cloud_transport::erase_last_copy(lookup_name, "_sub");
    transports[transport_name].sub_name = lookup_name;
    transports[transport_name].package_name = sub_loader.getClassPackage(lookup_name);
    try {
      auto sub = sub_loader.createUniqueInstance(lookup_name);
      transports[transport_name].sub_status = SUCCESS;
    } catch (const pluginlib::LibraryLoadException & e) {
      transports[transport_name].sub_status = LIB_LOAD_FAILURE;
      std::cout << "LibraryLoadException: " << e.what() << std::endl;
    } catch (const pluginlib::CreateClassException & e) {
      transports[transport_name].sub_status = CREATE_FAILURE;
      std::cout << "CreateClassException: " << e.what() << std::endl;
    }
  }

  bool problem_package = false;
  std::cout << "Declared transports:" << std::endl;
  for (const StatusMap::value_type & value : transports) {
    const TransportDesc & td = value.second;
    std::cout << value.first.c_str() << std::endl;
    if ((td.pub_status == CREATE_FAILURE || td.pub_status == LIB_LOAD_FAILURE) ||
      (td.sub_status == CREATE_FAILURE || td.sub_status == LIB_LOAD_FAILURE))
    {
      std::cout << " (*): Not available. Try 'colcon build --packages-select "
                << td.package_name.c_str() << "'." << std::endl;
      problem_package = true;
    }
    std::cout << std::endl;
  }

  if (problem_package) {
    std::cout << "(*) " << std::endl;
  }

  std::cout << std::endl << "Details:" << std::endl;
  for (const auto & value : transports) {
    const TransportDesc & td = value.second;
    std::cout << "----------" << std::endl;
    std::cout << "\"" << value.first.c_str() << "\"" << std::endl;
    if (td.pub_status == CREATE_FAILURE || td.sub_status == CREATE_FAILURE) {
      std::cout <<
        "*** Plugins are built, but could not be loaded. The package may need to be rebuilt "
        "or may not be compatible with this release of point_cloud_common. ***" << std::endl;
    } else if (td.pub_status == LIB_LOAD_FAILURE || td.sub_status == LIB_LOAD_FAILURE) {
      std::cout << "*** Plugins are not built. ***" << std::endl;
    }
    std::cout << " - Provided by package: " << td.package_name.c_str() << std::endl;
    if (td.pub_status == DOES_NOT_EXIST) {
      std::cout << " - No publisher provided" << std::endl;
    } else {
      std::cout << " - Publisher: " << pub_loader.getClassDescription(td.pub_name).c_str()
                << std::endl;
    }
    if (td.sub_status == DOES_NOT_EXIST) {
      std::cout << " - No subscriber provided" << std::endl;
    } else {
      std::cout << " - Subscriber: " << sub_loader.getClassDescription(td.sub_name).c_str()
                << std::endl;
    }
  }

  return 0;
}
