#ifndef POINT_CLOUD_TRANSPORT_TRANSPORT_HINTS_H
#define POINT_CLOUD_TRANSPORT_TRANSPORT_HINTS_H

#include <ros/ros.h>

namespace point_cloud_transport {

//! Stores transport settings for a point cloud topic subscription.

    class TransportHints
    {
    public:
        /**
         * Constructor.
         *
         * The default transport can be overridden by setting a certain parameter to the
         * name of the desired transport. By default this parameter is named "point_cloud_transport"
         * in the node's local namespace. For consistency across ROS applications, the
         * name of this parameter should not be changed without good reason.
         *
         */
        TransportHints(const std::string& default_transport = "raw",
                       const ros::TransportHints& ros_hints = ros::TransportHints(),
                       const ros::NodeHandle& parameter_nh = ros::NodeHandle("~"),
                       const std::string& parameter_name = "point_cloud_transport")
                : ros_hints_(ros_hints), parameter_nh_(parameter_nh)
        {
            parameter_nh_.param(parameter_name, transport_, default_transport);
        }

        const std::string& getTransport() const
        {
            return transport_;
        }

        const ros::TransportHints& getRosHints() const
        {
            return ros_hints_;
        }

        const ros::NodeHandle& getParameterNH() const
        {
            return parameter_nh_;
        }

    private:
        std::string transport_;
        ros::TransportHints ros_hints_;
        ros::NodeHandle parameter_nh_;
    };

} //namespace point_cloud_transport

#endif //POINT_CLOUD_TRANSPORT_TRANSPORT_HINTS_H
