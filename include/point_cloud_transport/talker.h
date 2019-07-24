#ifndef POINT_CLOUD_TRANSPORT_TALKER_H
#define POINT_CLOUD_TRANSPORT_TALKER_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

// conversion to Draco point cloud
#include "PC2toDraco.h"

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


  //! The timer variable used to go to callback function at specified rate.
  ros::Timer timer_;


  //! Message publisher.
  ros::Publisher pub_;

  //! Dynamic reconfigure server.
  dynamic_reconfigure::Server<point_cloud_transport::PointCloudConfig> dr_srv_;

  //!
    uint8_t encode_speed_;
    uint8_t decode_speed_;

    uint8_t POSITION_quantization_in_bits_;
    uint8_t NORMAL_quantization_in_bits_;
    uint8_t COLOR_quantization_in_bits_;
    uint8_t TEX_COORD_quantization_in_bits_;
    uint8_t GENERIC_quantization_in_bits_;

  //! Flag to set whether the node should do any work at all.
  bool enable_;

};
}

#endif  // POINT_CLOUD_TRANSPORT_TALKER_H
