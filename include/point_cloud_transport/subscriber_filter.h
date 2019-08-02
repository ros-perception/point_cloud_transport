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

#ifndef POINT_CLOUD_TRANSPORT_SUBSCRIBER_FILTER_H
#define POINT_CLOUD_TRANSPORT_SUBSCRIBER_FILTER_H

#include <ros/ros.h>
#include <message_filters/simple_filter.h>

#include "point_cloud_transport/point_cloud_transport.h"

namespace point_cloud_transport {

/**
 * PointCloud2 subscription filter.
 *
 * This class wraps Subscriber as a "filter" compatible with the message_filters
 * package. It acts as a highest-level filter, simply passing messages from a point cloud
 * transport subscription through to the filters which have connected to it.
 *
 * When this object is destroyed it will unsubscribe from the ROS subscription.
 *
 * \section connections CONNECTIONS
 *
 * SubscriberFilter has no input connection.
 *
 * The output connection for the SubscriberFilter object is the same signature as for roscpp
 * subscription callbacks, ie.
 */
    class SubscriberFilter : public message_filters::SimpleFilter<sensor_msgs::PointCloud2>
    {
    public:
        /**
         * Constructor
         *
         * See the ros::NodeHandle::subscribe() variants for more information on the parameters
         *
         * nh The ros::NodeHandle to use to subscribe.
         * base_topic The topic to subscribe to.
         * queue_size The subscription queue size
         * transport_hints The transport hints to pass along
         */
        SubscriberFilter(PointCloudTransport& pct, const std::string& base_topic, uint32_t queue_size,
                         const TransportHints& transport_hints = TransportHints())
        {
            subscribe(pct, base_topic, queue_size, transport_hints);
        }

        /**
         * Empty constructor, use subscribe() to subscribe to a topic
         */
        SubscriberFilter()
        {
        }

        ~SubscriberFilter()
        {
            unsubscribe();
        }

        /**
         * Subscribe to a topic.
         *
         * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
         *
         * nh The ros::NodeHandle to use to subscribe.
         * base_topic The topic to subscribe to.
         * queue_size The subscription queue size
         * transport_hints The transport hints to pass along
         */
        void subscribe(PointCloudTransport& pct, const std::string& base_topic, uint32_t queue_size,
                       const TransportHints& transport_hints = TransportHints())
        {
            unsubscribe();

            sub_ = pct.subscribe(base_topic, queue_size, boost::bind(&SubscriberFilter::cb, this, _1),
                                ros::VoidPtr(), transport_hints);
        }

        /**
         * Force immediate unsubscription of this subscriber from its topic
         */
        void unsubscribe()
        {
            sub_.shutdown();
        }

        std::string getTopic() const
        {
            return sub_.getTopic();
        }

        /**
         * Returns the number of publishers this subscriber is connected to.
         */
        uint32_t getNumPublishers() const
        {
            return sub_.getNumPublishers();
        }

        /**
         * Returns the name of the transport being used.
         */
        std::string getTransport() const
        {
            return sub_.getTransport();
        }

        /**
         * Returns the internal point_cloud_transport::Subscriber object.
         */
        const Subscriber& getSubscriber() const
        {
            return sub_;
        }

    private:

        void cb(const sensor_msgs::PointCloud2ConstPtr& m)
        {
            signalMessage(m);
        }

        Subscriber sub_;
    };

} // namespace point_cloud_transport

#endif //POINT_CLOUD_TRANSPORT_SUBSCRIBER_FILTER_H
