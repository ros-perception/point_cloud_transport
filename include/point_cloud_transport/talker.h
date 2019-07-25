#ifndef POINT_CLOUD_TRANSPORT_TALKER_H
#define POINT_CLOUD_TRANSPORT_TALKER_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

// conversion to Draco point cloud
#include "PC2toDraco.h"
#include "conversion_utilities.h"

// draco library
#include <draco/compression/encode.h>

// Custom message includes. Auto-generated from msg/ directory.
#include "point_cloud_transport/CompressedPointCloud2.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>

// Auto-generated from cfg/ directory.
#include <point_cloud_transport/PointCloudConfig.h>

namespace point_cloud_transport
{
class Talker
{
    public:
    //! Constructor.
    explicit Talker(ros::NodeHandle nh);

    private:
    //! Callback function for dynamic reconfigure server.
    void configCallback(point_cloud_transport::PointCloudConfig &config, uint32_t level);

    //! Timer callback for publishing message.
    void timerCallback(const ros::TimerEvent &event);

    //! Turn on publisher.
    void start();

    //! Turn off publisher.
    void stop();

    //! ROS node handle.
    ros::NodeHandle nh_;

    // TODO: delete -> testing purposes only
    //! timer variable used to go to callback function at specified rate.
    ros::Timer timer_;


    //! message publisher.
    ros::Publisher pub_;

    //! dynamic reconfigure server.
    dynamic_reconfigure::Server<point_cloud_transport::PointCloudConfig> dr_srv_;

    //! dynamic reconfigure parameters
    int encode_speed_;
    int decode_speed_;
    int POSITION_quantization_in_bits_;
    int NORMAL_quantization_in_bits_;
    int COLOR_quantization_in_bits_;
    int TEX_COORD_quantization_in_bits_;
    int GENERIC_quantization_in_bits_;

    //! Flag for enabling/disabling node
    bool enable_;

};
}

#endif  // POINT_CLOUD_TRANSPORT_TALKER_H
