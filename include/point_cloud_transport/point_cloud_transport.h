//
// Created by jakub on 7/25/19.
// THIS FILE ATTEMPTS TO COPY THE PLUGIN INTERFACE OF IMAGE_TRANSPORT
//

#ifndef POINT_CLOUD_TRANSPORT_POINT_CLOUD_TRANSPORT_H
#define POINT_CLOUD_TRANSPORT_POINT_CLOUD_TRANSPORT_H


#include "point_cloud_transport/publisher.h"
#include "point_cloud_transport/subscriber.h"


namespace point_cloud_transport {

/**
* Advertise and subscribe to PointCloud2 topics.
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




    //! Subscribe to a point cloud topic, version for arbitrary boost::function object.
    Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                         const boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>& callback,
                         const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                         const TransportHints& transport_hints = TransportHints());


    //! Subscribe to a point cloud topic, version for bare function.
    Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                         void(*fp)(const sensor_msgs::PointCloud2ConstPtr&),
                         const TransportHints& transport_hints = TransportHints())
    {
        return subscribe(base_topic, queue_size,
                         boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>(fp),
                ros::VoidPtr(), transport_hints);
    }



    //! Subscribe to a point cloud topic, version for class member function with bare pointer.

    template<class T>
    Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                         void(T::*fp)(const sensor_msgs::PointCloud2ConstPtr&), T* obj,
                         const TransportHints& transport_hints = TransportHints())
    {
        return subscribe(base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
    }


    //! Subscribe to a point cloud topic, version for class member function with shared_ptr.

    template<class T>
    Subscriber subscribe(const std::string& base_topic, uint32_t queue_size,
                         void(T::*fp)(const sensor_msgs::PointCloud2ConstPtr&),
                         const boost::shared_ptr<T>& obj,
                         const TransportHints& transport_hints = TransportHints())
    {
        return subscribe(base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
    }


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
