/*
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

#ifndef POINT_CLOUD_TRANSPORT__POINT_CLOUD_COMMON_HPP_
#define POINT_CLOUD_TRANSPORT__POINT_CLOUD_COMMON_HPP_

#include <string>
#include <vector>

#include "point_cloud_transport/visibility_control.hpp"

namespace point_cloud_transport
{

/**
 * \brief Replacement for uses of boost::erase_last_copy
 */
POINT_CLOUD_TRANSPORT_PUBLIC
std::string erase_last_copy(const std::string & input, const std::string & search);

POINT_CLOUD_TRANSPORT_PUBLIC
std::vector<std::string> split(
  const std::string & str, const std::string & delimiter,
  int maxSplits=-1);

// from cras::string_utils
POINT_CLOUD_TRANSPORT_PUBLIC
bool endsWith(const std::string & str, const std::string & suffix);

// from cras::string_utils
POINT_CLOUD_TRANSPORT_PUBLIC
std::string removeSuffix(const std::string & str, const std::string & suffix, bool * hadSuffix = nullptr);

POINT_CLOUD_TRANSPORT_PUBLIC
bool transportNameMatches(const std::string &lookup_name,
                          const std::string &name, const std::string &suffix);


// taken from cras::c_api.cpp
typedef void* (*allocator_t)(std::size_t);

POINT_CLOUD_TRANSPORT_PUBLIC
char* outputString(allocator_t allocator, const char* string, std::size_t length);

POINT_CLOUD_TRANSPORT_PUBLIC
char* outputString(allocator_t allocator, const std::string& string);

}  // namespace point_cloud_transport
#endif  // POINT_CLOUD_TRANSPORT__POINT_CLOUD_COMMON_HPP_
