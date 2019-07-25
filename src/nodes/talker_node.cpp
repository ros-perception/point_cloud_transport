#include <point_cloud_transport/talker.h>


namespace point_cloud_transport {
    sensor_msgs::PointCloud2 to_be_sent_PC2;
}

void MessageReceived(const sensor_msgs::PointCloud2 & msg);

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  point_cloud_transport::Talker node(nh);

  ros::Subscriber sub = nh.subscribe("dynamic_point_cloud", 100, &MessageReceived);

  while(ros::ok()) {



      // Let ROS handle all callbacks.
      ros::spinOnce();


  }
  return 0;
}  // end main()

void MessageReceived(const sensor_msgs::PointCloud2 & msg)
{
    point_cloud_transport::to_be_sent_PC2 = msg;
}
