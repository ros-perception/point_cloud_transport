
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <point_cloud_transport/point_cloud_transport.h>

//
#include <rosbag/bag.h>
#include <rosbag/view.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_publisher");
    ros::NodeHandle nh;

    point_cloud_transport::PointCloudTransport pct(nh);
    point_cloud_transport::Publisher pub = pct.advertise("pct/point_cloud", 1);

    rosbag::Bag bag;
    bag.open("/media/jakub/WDel\ NTFS/Downloads/husky_2019-07-04-09-49-59.bag", rosbag::bagmode::Read);

    ros::Rate loop_rate(5);

    for(rosbag::MessageInstance const m: rosbag::View(bag))
    {
        sensor_msgs::PointCloud2::ConstPtr i = m.instantiate<sensor_msgs::PointCloud2>();
        if (i != nullptr)
        {
            pub.publish(i);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
