#ifndef POINT_CLOUD_TRANSPORT_LISTENER_H
#define POINT_CLOUD_TRANSPORT_LISTENER_H


// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>

// draco
#include "draco/compression/decode.h"
#include "draco/compression/encode.h"

// point_cloud_transport
#include "DracotoPC2.h"
// Custom message includes. Auto-generated from msg/ directory.
#include "point_cloud_transport/InfoPointCloud2.h"
#include "point_cloud_transport/CompressedPointCloud2.h"


namespace point_cloud_transport
{
class Listener
{
 public:
  //! Constructor.
  explicit Listener(ros::NodeHandle nh);

  //! Callback function for subscriber.
  void messageCallback(const CompressedPointCloud2 &msg);


 private:
  //! Subscriber to custom message.
  ros::Subscriber sub_;
};
}

#endif  // POINT_CLOUD_TRANSPORT_LISTENER_H
