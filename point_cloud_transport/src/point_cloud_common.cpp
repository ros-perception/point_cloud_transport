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

#include <cstring>
#include <limits>

#include "point_cloud_transport/point_cloud_common.hpp"

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

}  // namespace point_cloud_transport
