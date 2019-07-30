#ifndef POINT_CLOUD_TRANSPORT_PUBLISHER_PLUGIN_H
#define POINT_CLOUD_TRANSPORT_PUBLISHER_PLUGIN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "point_cloud_transport/single_subscriber_publisher.h"

namespace point_cloud_transport {

    //! Base class for plugins to Publisher.
    class PublisherPlugin : boost::noncopyable
    {
    public:
        virtual ~PublisherPlugin() {}


        //!Get a string identifier for the transport provided by this plugin
        virtual std::string getTransportName() const = 0;

        //! Advertise a topic, simple version.
        void advertise(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                       bool latch = true)
        {
            advertiseImpl(nh, base_topic, queue_size, SubscriberStatusCallback(),
                          SubscriberStatusCallback(), ros::VoidPtr(), latch);
        }


        //! Advertise a topic with subscriber status callbacks.
        void advertise(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                       const SubscriberStatusCallback& connect_cb,
                       const SubscriberStatusCallback& disconnect_cb = SubscriberStatusCallback(),
                       const ros::VoidPtr& tracked_object = ros::VoidPtr(), bool latch = true)
        {
            advertiseImpl(nh, base_topic, queue_size, connect_cb, disconnect_cb, tracked_object, latch);
        }


        //! Returns the number of subscribers that are currently connected to this PublisherPlugin
        virtual uint32_t getNumSubscribers() const = 0;

        //! Returns the topic that this PublisherPlugin will publish on.
        virtual std::string getTopic() const = 0;

        //! Publish a point cloud using the transport associated with this PublisherPlugin.
        virtual void publish(const sensor_msgs::PointCloud2& message) const = 0;

        //! Publish a point cloud using the transport associated with this PublisherPlugin.
        virtual void publish(const sensor_msgs::PointCloud2ConstPtr& message) const
        {
            publish(*message);
        }


        /**
         * Publish a point cloud using the transport associated with this PublisherPlugin.
         * This version of the function can be used to optimize cases where you don't want to
         * fill a ROS message first to avoid useless copies.
         */
        virtual void publish(const sensor_msgs::PointCloud2& message, const uint8_t* data) const
        {
            sensor_msgs::PointCloud2 msg;

            msg.header = message.header;
            msg.height = message.height;
            msg.width = message.width;
            msg.fields = message.fields;
            msg.is_bigendian = message.is_bigendian;
            msg.point_step = message.point_step;
            msg.row_step = message.row_step;
            msg.is_dense = message.is_dense;

            msg.data = std::vector<uint8_t>(data, data + msg.point_step*msg.height*msg.width);

            publish(msg);
        }

        //! Shutdown any advertisements associated with this PublisherPlugin.
        virtual void shutdown() = 0;


        //! Return the lookup name of the PublisherPlugin associated with a specific transport identifier.
        static std::string getLookupName(const std::string& transport_name)
        {
            return "point_cloud_transport/" + transport_name + "_pub";
        }

    protected:

        //! Advertise a topic. Must be implemented by the subclass.
        virtual void advertiseImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                   const SubscriberStatusCallback& connect_cb,
                                   const SubscriberStatusCallback& disconnect_cb,
                                   const ros::VoidPtr& tracked_object, bool latch) = 0;
    };

} // namespace point_cloud_transport

#endif //POINT_CLOUD_TRANSPORT_PUBLISHER_PLUGIN_H
