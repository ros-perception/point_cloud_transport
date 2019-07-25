//
// Created by jakub on 7/25/19.
//

#ifndef POINT_CLOUD_TRANSPORT_SINGLE_SUBSCRIBER_PUBLISHER_H
#define POINT_CLOUD_TRANSPORT_SINGLE_SUBSCRIBER_PUBLISHER_H

#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <sensor_msgs/PointCloud2.h>

namespace point_cloud_transport {

    //! Allows publication of a point cloud to a single subscriber. Only available inside subscriber connection callbacks.
    class SingleSubscriberPublisher : boost::noncopyable
    {
    public:
        typedef boost::function<uint32_t()> GetNumSubscribersFn;
        typedef boost::function<void(const sensor_msgs::PointCloud2&)> PublishFn;

        SingleSubscriberPublisher(const std::string& caller_id, const std::string& topic,
                                  const GetNumSubscribersFn& num_subscribers_fn,
                                  const PublishFn& publish_fn);

        std::string getSubscriberName() const;

        std::string getTopic() const;

        uint32_t getNumSubscribers() const;

        void publish(const sensor_msgs::PointCloud2& message) const;
        void publish(const sensor_msgs::PointCloud2ConstPtr& message) const;

    private:
        std::string caller_id_;
        std::string topic_;
        GetNumSubscribersFn num_subscribers_fn_;
        PublishFn publish_fn_;

        friend class Publisher; // to get publish_fn_ directly
    };

    typedef boost::function<void(const SingleSubscriberPublisher&)> SubscriberStatusCallback;

} // namespace point_cloud_transport

#endif //POINT_CLOUD_TRANSPORT_SINGLE_SUBSCRIBER_PUBLISHER_H
