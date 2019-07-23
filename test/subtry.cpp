// ros
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

// point_cloud_transport
#include "Repubtry.h"



int main(int argc, char**argv){
    // initialize the ROS system
    ros::init(argc, argv, "main_test_node");
    ros::NodeHandle nh;

    Repubtry repub_object(nh);

    // decompressed and republished PointCloud2
    extern sensor_msgs::PointCloud2 PC2_;

    while(ros::ok()) {
        repub_object.pub_.publish(PC2_);

        // let ROS take over
        ros::spinOnce();
    }
    return 0;
}
