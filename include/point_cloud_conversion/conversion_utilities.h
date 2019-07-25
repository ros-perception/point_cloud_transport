
#ifndef POINT_CLOUD_TRANSPORT_CONVERSION_UTILITIES_H
#define POINT_CLOUD_TRANSPORT_CONVERSION_UTILITIES_H

// ros
#include <sensor_msgs/PointCloud2.h>

// point_cloud_transport
#include "point_cloud_transport/CompressedPointCloud2.h"


//! assigns header, width, ... from compressed to regular
void assign_description_of_PointCloud2(sensor_msgs::PointCloud2& target,point_cloud_transport::CompressedPointCloud2& source);

//! assigns header, width, ... from regular to compressed
void assign_description_of_PointCloud2(point_cloud_transport::CompressedPointCloud2& target, sensor_msgs::PointCloud2& source);

#endif //POINT_CLOUD_TRANSPORT_CONVERSION_UTILITIES_H
