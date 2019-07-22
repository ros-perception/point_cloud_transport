#ifndef POINTCLOUD_LISTENER_H
#define POINTCLOUD_LISTENER_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>

// Custom message includes. Auto-generated from msg/ directory.
#include <point_cloud_transport/PointCloudTransportData.h>

namespace point_cloud_transport
{
class Listener
{
 public:
  //! Constructor.
  explicit Listener(ros::NodeHandle nh);

  //! Callback function for subscriber.
  void messageCallback(const point_cloud_transport::PointCloudTransportData::ConstPtr &msg);
  int counter_;

 private:
  //! Subscriber to custom message.
  ros::Subscriber sub_;
};
}

#endif  // POINTCLOUD_LISTENER_H
