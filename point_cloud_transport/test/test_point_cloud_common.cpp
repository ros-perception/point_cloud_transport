// Copyright (c) 2023 Open Source Robotics Foundation, Inc.
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
//    * Neither the name of the Willow Garage nor the names of its
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

#include <gtest/gtest.h>

#include <typeinfo>

#include "point_cloud_transport/point_cloud_common.hpp"

TEST(PointCloudCommon, erase_last_copy) {
  EXPECT_EQ("pointcloud", point_cloud_transport::erase_last_copy("pointcloud_pub", "_pub"));
  EXPECT_EQ(
    "/pointcloud_pub/pointcloud",
    point_cloud_transport::erase_last_copy("/pointcloud_pub/pointcloud_pub", "_pub"));
  EXPECT_EQ(
    "/pointcloud/pointcloud",
    point_cloud_transport::erase_last_copy("/pointcloud_pub/pointcloud", "_pub"));
}

TEST(DefaultPluginsXml, raw_pub_transport_name) {
  const std::string transport = point_cloud_transport::get_transport_name_from_manifest(
    DEFAULT_PLUGINS_XML, "point_cloud_transport/raw_pub");
  EXPECT_EQ("raw", transport);
}

TEST(DefaultPluginsXml, raw_sub_transport_name) {
  const std::string transport = point_cloud_transport::get_transport_name_from_manifest(
    DEFAULT_PLUGINS_XML, "point_cloud_transport/raw_sub");
  EXPECT_EQ("raw", transport);
}

TEST(DefaultPluginsXml, raw_pub_message_type) {
  const std::string msg_type = point_cloud_transport::get_message_type_from_manifest(
    DEFAULT_PLUGINS_XML, "point_cloud_transport/raw_pub");
  EXPECT_EQ("sensor_msgs/msg/PointCloud2", msg_type);
}

TEST(DefaultPluginsXml, raw_sub_message_type) {
  const std::string msg_type = point_cloud_transport::get_message_type_from_manifest(
    DEFAULT_PLUGINS_XML, "point_cloud_transport/raw_sub");
  EXPECT_EQ("sensor_msgs/msg/PointCloud2", msg_type);
}

TEST(DefaultPluginsXml, unknown_lookup_name_returns_empty) {
  EXPECT_EQ(
    point_cloud_transport::get_transport_name_from_manifest(
      DEFAULT_PLUGINS_XML, "point_cloud_transport/does_not_exist"),
    "");
  EXPECT_EQ(
    point_cloud_transport::get_message_type_from_manifest(
      DEFAULT_PLUGINS_XML, "point_cloud_transport/does_not_exist"),
    "");
}

TEST(DefaultPluginsXml, bad_path_returns_empty) {
  EXPECT_EQ(
    point_cloud_transport::get_transport_name_from_manifest(
      "/nonexistent/path/plugins.xml", "point_cloud_transport/raw_pub"),
    "");
  EXPECT_EQ(
    point_cloud_transport::get_message_type_from_manifest(
      "/nonexistent/path/plugins.xml", "point_cloud_transport/raw_pub"),
    "");
}

struct A
{
  virtual ~A() = default;
};

struct B : A {};

TEST(PointCloudCommon, demangle) {
  using point_cloud_transport::demangle_cpp_type_name;
  using point_cloud_transport::PluginManifestData;

  EXPECT_EQ("int", demangle_cpp_type_name(typeid(int).name()));  // NOLINT(readability/casting)
  EXPECT_EQ("point_cloud_transport::PluginManifestData",
    demangle_cpp_type_name(typeid(PluginManifestData{}).name()));
  EXPECT_EQ("A", demangle_cpp_type_name(typeid(A{}).name()));
  EXPECT_EQ("B", demangle_cpp_type_name(typeid(B{}).name()));
  A * a = new B;
  EXPECT_EQ("B", demangle_cpp_type_name(typeid(*a).name()));
  delete a;
}

// ---------------------------------------------------------------------------
// Tests that verify per-class overrides take precedence over library defaults.
// TEST_PLUGIN_MANIFEST_XML is injected as a compile definition from CMakeLists.txt.
// ---------------------------------------------------------------------------

TEST(PluginManifestXml, class_level_transport_name_overrides_library) {
  EXPECT_EQ(
    "class_transport",
    point_cloud_transport::get_transport_name_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "point_cloud_transport/class_override_pub"));
}

TEST(PluginManifestXml, class_level_message_type_overrides_library) {
  EXPECT_EQ(
    "point_cloud_interfaces/msg/CompressedPointCloud2",
    point_cloud_transport::get_message_type_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "point_cloud_transport/class_override_pub"));
}

TEST(PluginManifestXml, library_level_transport_name_used_as_fallback) {
  EXPECT_EQ(
    "lib_transport",
    point_cloud_transport::get_transport_name_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "point_cloud_transport/lib_fallback_pub"));
}

TEST(PluginManifestXml, library_level_message_type_used_as_fallback) {
  EXPECT_EQ(
    "sensor_msgs/msg/PointCloud2",
    point_cloud_transport::get_message_type_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "point_cloud_transport/lib_fallback_pub"));
}

// ---------------------------------------------------------------------------
// Tests that verify fallback to lookup name parsing.
// ---------------------------------------------------------------------------

TEST(PluginManifestXml, fallback_to_lookup_name_pub) {
  EXPECT_EQ(
    "lookup",
    point_cloud_transport::get_transport_name_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "point_cloud_transport/lookup_pub"));
}

TEST(PluginManifestXml, fallback_to_lookup_name_sub) {
  EXPECT_EQ(
    "lookup",
    point_cloud_transport::get_transport_name_from_manifest(
      TEST_PLUGIN_MANIFEST_XML, "point_cloud_transport/lookup_sub"));
}
