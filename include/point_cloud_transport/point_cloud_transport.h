//
// Created by jakub on 7/25/19.
// THIS FILE ATTEMPTS TO COPY THE PLUGIN INTERFACE OF IMAGE_TRANSPORT
//

#ifndef POINT_CLOUD_TRANSPORT_POINT_CLOUD_TRANSPORT_H
#define POINT_CLOUD_TRANSPORT_POINT_CLOUD_TRANSPORT_H


#include "image_transport/publisher.h"
#include "image_transport/subscriber.h"


namespace point_cloud_transport {

/**
* \brief Advertise and subscribe to PointCloud2 topics.
*
* PointCloudTransport is analogous to ros::NodeHandle in that it contains advertise() and
* subscribe() functions for creating advertisements and subscriptions of PointCloud2 topics.
*/

class PointCloudTransport {
public:
    //! Constructor
    explicit PointCloudTransport(const ros::NodeHandle& nh);

    //! Destructor
    ~PointCloudTransport();

    // TODO: 1)
    //! Advertise a PointCloud2 topic, simple version.
    Publisher advertise(const std::string& base_topic, uint32_t queue_size, bool latch = false);

    // TODO: 2)
    //! Advertise an PointCloud2 topic with subcriber status callbacks.
    Publisher advertise(const std::string& base_topic, uint32_t queue_size,
                        const SubscriberStatusCallback& connect_cb,
                        const SubscriberStatusCallback& disconnect_cb = SubscriberStatusCallback(),
                        const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = false);

    //! Returns the names of all transports declared in the system. Declared
    //! transports are not necessarily built or loadable.
    std::vector<std::string> getDeclaredTransports() const;

    //! Returns the names of all transports that are loadable in the system.
    std::vector<std::string> getLoadableTransports() const;

private:
    struct Impl;
    typedef boost::shared_ptr<Impl> ImplPtr;
    typedef boost::weak_ptr<Impl> ImplWPtr;

    ImplPtr impl_;
};

} // namespace point_cloud_transport

#endif //POINT_CLOUD_TRANSPORT_POINT_CLOUD_TRANSPORT_H
