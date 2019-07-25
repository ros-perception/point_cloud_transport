#ifndef POINT_CLOUD_TRANSPORT_DRACOTOPC2_H
#define POINT_CLOUD_TRANSPORT_DRACOTOPC2_H

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>

// draco
#include <draco/point_cloud/point_cloud.h>

// point_cloud_transport
#include "point_cloud_transport/CompressedPointCloud2.h"
#include "conversion_utilities.h"

class DracotoPC2 {
public:
    //! Constructor.
    explicit DracotoPC2(std::unique_ptr<draco::PointCloud> && pc, const point_cloud_transport::CompressedPointCloud2 & compressed_PC2);

    //! Destructor
    ~DracotoPC2();

    //! Method for converting into sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 convert();

private:
    //! Message to be converted
    std::unique_ptr<draco::PointCloud> pc_;

    //! Structure to hold information about sensor_msgs::PointCloud2
    point_cloud_transport::CompressedPointCloud2 compressed_PC2_;

};


#endif //POINT_CLOUD_TRANSPORT_DRACOTOPC2_H
