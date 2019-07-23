#ifndef POINT_CLOUD_TRANSPORT_DRACOTOPC2_H
#define POINT_CLOUD_TRANSPORT_DRACOTOPC2_H

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>

// draco
#include <draco/point_cloud/point_cloud.h>

// point_cloud_transport
#include <point_cloud_transport/InfoPointCloud2.h>

class DracotoPC2 {
public:
    //! Constructor.
    explicit DracotoPC2(std::unique_ptr<draco::PointCloud> && pc, point_cloud_transport::InfoPointCloud2 infoPC2);

    //! Destructor
    ~DracotoPC2();

    //! Method for converting into sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 convert();

private:
    //! Message to be converted
    std::unique_ptr<draco::PointCloud> pc_;

    //! Structure to hold information about sensor_msgs::PointCloud2 without storing its full data
    point_cloud_transport::InfoPointCloud2 infoPC2_;

};


#endif //POINT_CLOUD_TRANSPORT_DRACOTOPC2_H
