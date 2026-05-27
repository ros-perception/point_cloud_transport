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

#include <cstdlib>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#if defined(__GNUC__) || defined(__clang__)
#include <cxxabi.h>
#endif

#include "pluginlib/class_loader.hpp"
#include "tinyxml2.h"  // NOLINT(build/include_subdir)

#include "point_cloud_transport/point_cloud_common.hpp"

// Forward declarations needed by ClassLoader template instantiation.
#include "point_cloud_transport/publisher_plugin.hpp"
#include "point_cloud_transport/subscriber_plugin.hpp"

namespace point_cloud_transport
{

std::string erase_last_copy(const std::string & input, const std::string & search)
{
  size_t found = input.rfind(search);
  auto input_copy = input;
  if (found != std::string::npos) {
    input_copy.replace(found, search.length(), "");
  }
  return input_copy;
}

std::vector<std::string> split(
  const std::string & str, const std::string & delimiter,
  int maxSplits)
{
  // inspired by https://stackoverflow.com/a/46931770/1076564, CC-BY-SA 4.0
  // renamed some variables, added the maxSplits option
  size_t start{0};
  size_t end;
  size_t delimiterLength{delimiter.length()};
  std::string token;
  std::vector<std::string> result;

  size_t split_limit = maxSplits >= 0 ? maxSplits : std::numeric_limits<size_t>::max();

  while ((end =
    str.find(
      delimiter,
      start)) != std::string::npos && (result.size() < split_limit))
  {
    token = str.substr(start, end - start);
    start = end + delimiterLength;
    result.push_back(token);
  }

  result.push_back(str.substr(start));
  return result;
}

// from cras::string_utils
bool endsWith(const std::string & str, const std::string & suffix)
{
  return str.size() >= suffix.size() && str.compare(
    str.size() - suffix.size(),
    suffix.size(), suffix) == 0;
}


// from cras::string_utils
std::string removeSuffix(const std::string & str, const std::string & suffix, bool * hadSuffix)
{
  const auto hasSuffix = endsWith(str, suffix);
  if (hadSuffix != nullptr) {
    *hadSuffix = hasSuffix;
  }

  return hasSuffix ? str.substr(0, str.length() - suffix.length()) : str;
}

bool transportNameMatches(
  const std::string & lookup_name,
  const std::string & name, const std::string & suffix)
{
  if (lookup_name == name) {
    return true;
  }
  const std::string transport_name = removeSuffix(lookup_name, suffix);
  if (transport_name == name) {
    return true;
  }
  const auto parts = split(transport_name, "/");
  if (parts.size() == 2 && parts[1] == name) {
    return true;
  }
  return false;
}

std::string demangle_cpp_type_name(const char * mangled_name)
{
#if defined(__GNUC__) || defined(__clang__)
  int status = 0;
  char * d = abi::__cxa_demangle(mangled_name, nullptr, nullptr, &status);
  std::string result = (status == 0 && d) ? d : mangled_name;
  std::free(d);
  return result;
#elif defined(_MSC_VER)
  std::string result = mangled_name;
  if (result.size() > 6 && result.substr(0, 6) == "class ") {
    result = result.substr(6);
  }
  if (result.size() > 7 && result.substr(0, 7) == "struct ") {
    result = result.substr(7);
  }
  return result;
#else
#warning "Your platform does not support C++ type name demangling"
  return mangled_name;
#endif
}

namespace
{
/// Extract the text content of a child element named @child_name.
const char * get_child_text(
  tinyxml2::XMLElement * elem,
  const char * child_name)
{
  auto * child = elem->FirstChildElement(child_name);
  return child ? child->GetText() : nullptr;
}

/// Return the first <library> element, handling both <library> and <class_libraries> as root.
tinyxml2::XMLElement * first_library(tinyxml2::XMLDocument & doc)
{
  auto * root = doc.RootElement();
  if (!root) {
    return nullptr;
  }
  if (std::string(root->Name()) == "class_libraries") {
    return root->FirstChildElement("library");
  }
  if (std::string(root->Name()) == "library") {
    return root;
  }
  return nullptr;
}
}  // namespace

std::string get_transport_name_from_manifest(
  const std::string & manifest_path,
  const std::string & lookup_name)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(manifest_path.c_str()) != tinyxml2::XML_SUCCESS) {
    return "";
  }
  for (auto * lib = first_library(doc); lib != nullptr; lib = lib->NextSiblingElement("library")) {
    const char * lib_transport = get_child_text(lib, "transport_name");
    for (auto * cls = lib->FirstChildElement("class");
      cls != nullptr;
      cls = cls->NextSiblingElement("class"))
    {
      const char * name = cls->Attribute("name");
      if (!name || lookup_name != name) {
        continue;
      }
      const char * cls_transport = get_child_text(cls, "transport_name");
      if (cls_transport) {
        return cls_transport;
      }
      if (lib_transport) {
        return lib_transport;
      }
      if (!lookup_name.empty()) {
        const auto pos = lookup_name.rfind('/');
        const std::string short_name = (pos != std::string::npos) ?
          lookup_name.substr(pos + 1) :
          lookup_name;
        auto lookup_name_transport = erase_last_copy(short_name, "_sub");
        lookup_name_transport = erase_last_copy(lookup_name_transport, "_pub");
        return lookup_name_transport;
      }
      return "";
    }
  }
  return "";
}

std::string get_message_type_from_manifest(
  const std::string & manifest_path,
  const std::string & lookup_name)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(manifest_path.c_str()) != tinyxml2::XML_SUCCESS) {
    return "";
  }
  for (auto * lib = first_library(doc); lib != nullptr; lib = lib->NextSiblingElement("library")) {
    const char * lib_type = get_child_text(lib, "message_type");
    for (auto * cls = lib->FirstChildElement("class");
      cls != nullptr;
      cls = cls->NextSiblingElement("class"))
    {
      const char * name = cls->Attribute("name");
      if (!name || lookup_name != name) {
        continue;
      }
      const char * cls_type = get_child_text(cls, "message_type");
      if (cls_type) {
        return cls_type;
      }
      return lib_type ? lib_type : "";
    }
  }
  return "";
}

}  // namespace point_cloud_transport

// ---------------------------------------------------------------------------
// Manifest auto-discovery helpers
// ---------------------------------------------------------------------------

namespace
{

template<class BaseT>
point_cloud_transport::PluginManifestData search_manifest_for_type(
  const std::string & package,
  const std::string & base_class_type,
  const std::string & cpp_type_name)
{
  try {
    pluginlib::ClassLoader<BaseT> loader(package, base_class_type);
    for (const auto & lookup_name : loader.getDeclaredClasses()) {
      if (loader.getClassType(lookup_name) == cpp_type_name) {
        const std::string manifest_path = loader.getPluginManifestPath(lookup_name);
        return {
          point_cloud_transport::get_transport_name_from_manifest(manifest_path, lookup_name),
          point_cloud_transport::get_message_type_from_manifest(manifest_path, lookup_name),
          lookup_name
        };
      }
    }
  } catch (const std::exception &) {
    // Silently ignore: ament index unavailable, no plugins registered, etc.
  }
  return {};
}

}  // anonymous namespace

namespace point_cloud_transport
{

PluginManifestData get_pub_manifest_data_from_class_type(const std::string & cpp_type_name)
{
  return search_manifest_for_type<PublisherPlugin>(
    "point_cloud_transport", "point_cloud_transport::PublisherPlugin", cpp_type_name);
}

PluginManifestData get_sub_manifest_data_from_class_type(const std::string & cpp_type_name)
{
  return search_manifest_for_type<SubscriberPlugin>(
    "point_cloud_transport", "point_cloud_transport::SubscriberPlugin", cpp_type_name);
}

}  // namespace point_cloud_transport
