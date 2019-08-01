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
