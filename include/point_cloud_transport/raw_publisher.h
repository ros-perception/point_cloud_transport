/*
 *
 * BSD 3-Clause License
 *
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

#ifndef POINT_CLOUD_TRANSPORT_RAW_PUBLISHER_H
#define POINT_CLOUD_TRANSPORT_RAW_PUBLISHER_H

#include "point_cloud_transport/simple_publisher_plugin.h"

namespace point_cloud_transport {

    //! RawPublisher is a simple wrapper for ros::Publisher, publishing unaltered PointCloud2 messages on the base topic.
    class RawPublisher : public SimplePublisherPlugin<sensor_msgs::PointCloud2>
    {
    public:
        virtual ~RawPublisher() {}

        virtual std::string getTransportName() const
        {
            return "raw";
        }

        // Override the default implementation because publishing the message pointer allows
        // the no-copy intraprocess optimization.
        virtual void publish(const sensor_msgs::PointCloud2ConstPtr& message) const
        {
            getPublisher().publish(message);
        }


        // Override the default implementation to not copy data to a sensor_msgs::PointCloud2 first
        virtual void publish(const sensor_msgs::PointCloud2& message, const uint8_t* data) const;


    protected:
        virtual void publish(const sensor_msgs::PointCloud2& message, const PublishFn& publish_fn) const
        {
            publish_fn(message);
        }

        virtual std::string getTopicToAdvertise(const std::string& base_topic) const
        {
            return base_topic;
        }
    };

} //namespace point_cloud_transport


#endif //POINT_CLOUD_TRANSPORT_RAW_PUBLISHER_H
