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

#include "point_cloud_transport/qos.hpp"


namespace point_cloud_transport
{


bool detectQoSProfileFromString(const std::string& profile_name,
                                rmw_qos_profile_t& out_detected_profile)
{
    // returns unknown profile by default
    out_detected_profile = rmw_qos_profile_unknown;

    if(profile_name == "SENSOR_DATA")
    {
       out_detected_profile = rmw_qos_profile_sensor_data;
    }
    else if (profile_name == "SYSTEM_DEFAULT")
    {
       out_detected_profile = rmw_qos_profile_system_default;
    }
    else if (profile_name == "PARAMS_EVENTS")
    {
       out_detected_profile = rmw_qos_profile_parameter_events;
    }
    else if (profile_name ==  "PARAMS")
    {
       out_detected_profile = rmw_qos_profile_parameters;
    }
    else if (profile_name == "SERVICES_DEFAULT")
    {
       out_detected_profile = rmw_qos_profile_services_default;
    }
    else
    {
        return(false);
    }
    return(true);
}
};
