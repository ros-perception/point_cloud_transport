#ifndef POINT_CLOUD_TRANSPORT_RAW_SUBSCRIBER_H
#define POINT_CLOUD_TRANSPORT_RAW_SUBSCRIBER_H

#include "point_cloud_transport/simple_subscriber_plugin.h"

namespace point_cloud_transport {

/**
 * The default SubscriberPlugin.
 *
 * RawSubscriber is a simple wrapper for ros::Subscriber which listens for PointCloud2 messages
 * and passes them through to the callback.
 */
    class RawSubscriber : public SimpleSubscriberPlugin<sensor_msgs::PointCloud2>
    {
    public:
        virtual ~RawSubscriber() {}

        virtual std::string getTransportName() const
        {
            return "raw";
        }

    protected:
        virtual void internalCallback(const sensor_msgs::PointCloud2ConstPtr& message, const Callback& user_cb)
        {
            user_cb(message);
        }

        virtual std::string getTopicToSubscribe(const std::string& base_topic) const
        {
            return base_topic;
        }
    };

} //namespace point_cloud_transport


#endif //POINT_CLOUD_TRANSPORT_RAW_SUBSCRIBER_H
