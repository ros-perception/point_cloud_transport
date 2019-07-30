#ifndef POINT_CLOUD_TRANSPORT_PUBLISHER_H
#define POINT_CLOUD_TRANSPORT_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "point_cloud_transport/single_subscriber_publisher.h"
#include "point_cloud_transport/exception.h"
#include "point_cloud_transport/loader_fwds.h"

namespace point_cloud_transport {

    class Publisher {
    public:
        //! Constructor
        Publisher() {}

        //! get total number of subscribers to all advertised topics.
        uint32_t getNumSubscribers() const;

        //! get base topic of this Publisher
        std::string getTopic() const;

        //! Publish a point cloud on the topics associated with this Publisher.
        void publish(const sensor_msgs::PointCloud2& message) const;


        // ConstPtr types are typedef(s) for boost::shared_ptr
        //! Publish a point cloud on the topics associated with this Publisher.
        void publish(const sensor_msgs::PointCloud2ConstPtr& message) const;


        //! Shutdown the advertisements associated with this Publisher.
        void shutdown();

        operator void*() const;
        bool operator< (const Publisher& rhs) const { return impl_ <  rhs.impl_; }
        bool operator!=(const Publisher& rhs) const { return impl_ != rhs.impl_; }
        bool operator==(const Publisher& rhs) const { return impl_ == rhs.impl_; }

    private:
        Publisher(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                  const SubscriberStatusCallback& connect_cb,
                  const SubscriberStatusCallback& disconnect_cb,
                  const ros::VoidPtr& tracked_object, bool latch,
                  const PubLoaderPtr& loader);

        struct Impl;
        typedef boost::shared_ptr<Impl> ImplPtr;
        typedef boost::weak_ptr<Impl> ImplWPtr;


        ImplPtr impl_;

        static void weakSubscriberCb(const ImplWPtr& impl_wptr,
                                     const SingleSubscriberPublisher& plugin_pub,
                                     const SubscriberStatusCallback& user_cb);

        SubscriberStatusCallback rebindCB(const SubscriberStatusCallback& user_cb);

        friend class PointCloudTransport;
    };

} // namespace point_cloud_transport

#endif //POINT_CLOUD_TRANSPORT_PUBLISHER_H
