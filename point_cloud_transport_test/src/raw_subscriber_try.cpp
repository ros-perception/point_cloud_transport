#include <ros/ros.h>
#include <point_cloud_transport/point_cloud_transport.h>
#include <sensor_msgs/PointCloud2.h>

void Callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
        std::cout << "Message received, number of points is: " << msg->width*msg->height << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_subscriber");
    ros::NodeHandle nh;

    point_cloud_transport::PointCloudTransport pct(nh);
    point_cloud_transport::Subscriber sub = pct.subscribe("pct/point_cloud", 1, Callback);
    ros::spin();
}

