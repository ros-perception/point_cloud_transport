#include <point_cloud_transport/listener.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  point_cloud_transport::Listener node(nh);

  while(nh.ok())
  {
      // Let ROS handle all callbacks.
      ros::spinOnce();
  }


  return 0;
}  // end main()
