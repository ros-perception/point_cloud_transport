#ifndef POINT_CLOUD_TRANSPORT_REPUBTRY_H
#define POINT_CLOUD_TRANSPORT_REPUBTRY_H

// ros
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

// point_cloud_transport
#include "PC2toDraco.h"
#include "DracotoPC2.h"
#include <point_cloud_transport/InfoPointCloud2.h>

// draco
#include "draco/compression/decode.h"
#include "draco/compression/encode.h"


class Repubtry {
public:

        //! Constructor
        explicit Repubtry(ros::NodeHandle nh);

        //! Destructor
        ~Repubtry();

         //! public publisher of converted PointCloud2
        ros::Publisher pub_;

private:
        //! private subscriber to source of PointCloud2 messages
        ros::Subscriber sub_;

        //! handle of caller node
        ros::NodeHandle nh_;

};

//! subscriber callback
void MessageReceived(const sensor_msgs::PointCloud2 & msg);

#endif //POINT_CLOUD_TRANSPORT_REPUBTRY_H
