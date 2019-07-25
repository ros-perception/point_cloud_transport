//
// Created by jakub on 7/25/19.
//

#ifndef POINT_CLOUD_TRANSPORT_SUBSCRIBER_H
#define POINT_CLOUD_TRANSPORT_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "point_cloud_transport/transport_hints.h"
#include "point_cloud_transport/exception.h"
#include "point_cloud_transport/loader_fwds.h"

namespace point_cloud_transport {

/**
 * Manages a subscription callback on a specific topic that can be interpreted
 * as a PointCloud2 topic.
 *
 * Subscriber is the client-side counterpart to Publisher. By loading the
 * appropriate plugin, it can subscribe to a base point cloud topic using any available
 * transport. The complexity of what transport is actually used is hidden from the user,
 * who sees only a normal PointCloud2 callback.
 *
 * A Subscriber should always be created through a call to PointCloudTransport::subscribe(),
 * or copied from one that was.
 * Once all copies of a specific Subscriber go out of scope, the subscription callback
 * associated with that handle will stop being called. Once all Subscriber for a given
 * topic go out of scope the topic will be unsubscribed.
 */
    class Subscriber
    {
    public:
        Subscriber() {}

        /**
         * Returns the base point cloud topic.
         *
         * The Subscriber may actually be subscribed to some transport-specific topic that
         * differs from the base topic.
         */
        std::string getTopic() const;

        /**
         * Returns the number of publishers this subscriber is connected to.
         */
        uint32_t getNumPublishers() const;

        /**
         * Returns the name of the transport being used.
         */
        std::string getTransport() const;

        /**
         * Unsubscribe the callback associated with this Subscriber.
         */
        void shutdown();

        operator void*() const;
        bool operator< (const Subscriber& rhs) const { return impl_ <  rhs.impl_; }
        bool operator!=(const Subscriber& rhs) const { return impl_ != rhs.impl_; }
        bool operator==(const Subscriber& rhs) const { return impl_ == rhs.impl_; }

    private:
        Subscriber(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                   const boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>& callback,
                   const ros::VoidPtr& tracked_object, const TransportHints& transport_hints,
                   const SubLoaderPtr& loader);

        struct Impl;
        typedef boost::shared_ptr<Impl> ImplPtr;
        typedef boost::weak_ptr<Impl> ImplWPtr;

        ImplPtr impl_;

        friend class PointCloudTransport;
    };

} //namespace point_cloud_transport

#endif //POINT_CLOUD_TRANSPORT_SUBSCRIBER_H
