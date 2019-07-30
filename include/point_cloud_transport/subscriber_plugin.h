#ifndef POINT_CLOUD_TRANSPORT_SUBSCRIBER_PLUGINS_H
#define POINT_CLOUD_TRANSPORT_SUBSCRIBER_PLUGINS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/noncopyable.hpp>
#include "point_cloud_transport/transport_hints.h"

namespace point_cloud_transport {

/**
 * Base class for plugins to Subscriber.
 */
    class SubscriberPlugin : boost::noncopyable
    {
    public:
        typedef boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)> Callback;

        virtual ~SubscriberPlugin() {}

        /**
         * Get a string identifier for the transport provided by
         * this plugin.
         */
        virtual std::string getTransportName() const = 0;

        /**
         * Subscribe to a point cloud topic, version for arbitrary boost::function object.
         */
        void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                       const Callback& callback, const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                       const TransportHints& transport_hints = TransportHints())
        {
            return subscribeImpl(nh, base_topic, queue_size, callback, tracked_object, transport_hints);
        }

        /**
         * Subscribe to a point cloud topic, version for bare function.
         */
        void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                       void(*fp)(const sensor_msgs::PointCloud2ConstPtr&),
                       const TransportHints& transport_hints = TransportHints())
        {
            return subscribe(nh, base_topic, queue_size,
                             boost::function<void(const sensor_msgs::PointCloud2ConstPtr&)>(fp),
                    ros::VoidPtr(), transport_hints);
        }

        /**
         * Subscribe to a point cloud topic, version for class member function with bare pointer.
         */
        template<class T>
        void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                       void(T::*fp)(const sensor_msgs::PointCloud2ConstPtr&), T* obj,
                       const TransportHints& transport_hints = TransportHints())
        {
            return subscribe(nh, base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
        }

        /**
         * Subscribe to a point cloud topic, version for class member function with shared_ptr.
         */
        template<class T>
        void subscribe(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                       void(T::*fp)(const sensor_msgs::PointCloud2ConstPtr&),
                       const boost::shared_ptr<T>& obj,
                       const TransportHints& transport_hints = TransportHints())
        {
            return subscribe(nh, base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
        }

        /**
         * Get the transport-specific communication topic.
         */
        virtual std::string getTopic() const = 0;

        /**
         * Returns the number of publishers this subscriber is connected to.
         */
        virtual uint32_t getNumPublishers() const = 0;

        /**
         * Unsubscribe the callback associated with this SubscriberPlugin.
         */
        virtual void shutdown() = 0;

        /**
         * Return the lookup name of the SubscriberPlugin associated with a specific
         * transport identifier.
         */
        static std::string getLookupName(const std::string& transport_type)
        {
            return "point_cloud_transport/" + transport_type + "_sub";
        }

    protected:
        /**
         * Subscribe to a point cloud transport topic. Must be implemented by the subclass.
         */
        virtual void subscribeImpl(ros::NodeHandle& nh, const std::string& base_topic, uint32_t queue_size,
                                   const Callback& callback, const ros::VoidPtr& tracked_object,
                                   const TransportHints& transport_hints) = 0;
    };

} //namespace point_cloud_transport

#endif //POINT_CLOUD_TRANSPORT_SUBSCRIBER_PLUGINS_H
