// Copyright (c) 2023, John D'Angelo
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

#ifndef POINT_CLOUD_TRANSPORT__POINT_CLOUD_COMMON_HPP_
#define POINT_CLOUD_TRANSPORT__POINT_CLOUD_COMMON_HPP_

#include <string>
#include <vector>

#include "point_cloud_transport/visibility_control.hpp"

namespace point_cloud_transport
{

/// \brief Remove the last copy of the given substring from the given string.
/// \param input original string
/// \param search substring to remove the last entry of
/// \return "input" with the last entry of "search" removed
POINT_CLOUD_TRANSPORT_PUBLIC
std::string erase_last_copy(const std::string & input, const std::string & search);

/// \brief Split a string into substrings using the given delimiter.
/// \param str string to split
/// \param delimiter delimiter to split on
/// \param maxSplits limit on the max number of splits to perform (-1 for unlimited splits)
/// \return vector of string tokens
POINT_CLOUD_TRANSPORT_PUBLIC
std::vector<std::string> split(
  const std::string & str, const std::string & delimiter,
  int maxSplits = -1);

/// \brief Check if a string ends with a given suffix. (from cras::string_utils)
/// \param str string to check
/// \param suffix suffix to check for
/// \return true if "str" ends with "suffix", false otherwise
POINT_CLOUD_TRANSPORT_PUBLIC
bool endsWith(const std::string & str, const std::string & suffix);

/// \brief Remove a suffix from a string if it exists.
/// \param str string to remove suffix from
/// \param suffix suffix to remove
/// \param hadSuffix if non-null, set to true if "str" had "suffix" at the end
/// \return "str" with "suffix" removed
POINT_CLOUD_TRANSPORT_PUBLIC
std::string removeSuffix(
  const std::string & str, const std::string & suffix,
  bool * hadSuffix = nullptr);

/// \brief Check if the given transport matches the given name.
/// \param lookup_name name of the transport to check
/// \param name name to check against
/// \param suffix suffix to remove from name before checking for equality
/// \return true if "lookup_name" matches "name" with "suffix" removed, false otherwise
POINT_CLOUD_TRANSPORT_PUBLIC
bool transportNameMatches(
  const std::string & lookup_name,
  const std::string & name, const std::string & suffix);


}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__POINT_CLOUD_COMMON_HPP_
