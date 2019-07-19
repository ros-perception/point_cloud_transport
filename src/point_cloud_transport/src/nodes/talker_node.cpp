#include <point_cloud_transport/talker.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  point_cloud_transport::Talker node(nh);

  // Let ROS handle all callbacks.
  ros::spin();

  return 0;
}  // end main()
