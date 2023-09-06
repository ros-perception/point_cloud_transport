// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POINT_CLOUD_TRANSPORT__VISIBILITY_CONTROL_HPP_
#define POINT_CLOUD_TRANSPORT__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define POINT_CLOUD_TRANSPORT_EXPORT __attribute__ ((dllexport))
    #define POINT_CLOUD_TRANSPORT_IMPORT __attribute__ ((dllimport))
  #else
    #define POINT_CLOUD_TRANSPORT_EXPORT __declspec(dllexport)
    #define POINT_CLOUD_TRANSPORT_IMPORT __declspec(dllimport)
  #endif
  #ifdef POINT_CLOUD_TRANSPORT_BUILDING_DLL
    #define POINT_CLOUD_TRANSPORT_PUBLIC POINT_CLOUD_TRANSPORT_EXPORT
  #else
    #define POINT_CLOUD_TRANSPORT_PUBLIC POINT_CLOUD_TRANSPORT_IMPORT
  #endif
  #define POINT_CLOUD_TRANSPORT_PUBLIC_TYPE POINT_CLOUD_TRANSPORT_PUBLIC
  #define POINT_CLOUD_TRANSPORT_LOCAL
#else
  #define POINT_CLOUD_TRANSPORT_EXPORT __attribute__ ((visibility("default")))
  #define POINT_CLOUD_TRANSPORT_IMPORT
  #if __GNUC__ >= 4
    #define POINT_CLOUD_TRANSPORT_PUBLIC __attribute__ ((visibility("default")))
    #define POINT_CLOUD_TRANSPORT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define POINT_CLOUD_TRANSPORT_PUBLIC
    #define POINT_CLOUD_TRANSPORT_LOCAL
  #endif
  #define POINT_CLOUD_TRANSPORT_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // POINT_CLOUD_TRANSPORT__VISIBILITY_CONTROL_HPP_
