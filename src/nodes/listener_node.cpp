#include <point_cloud_transport/listener.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "listener");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  point_cloud_transport::Listener node(nh);

  extern sensor_msgs::PointCloud2 global_decoded_PC2;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/decompressed_point_cloud", 100);



  while(nh.ok())
  {

      pub.publish(global_decoded_PC2);

      // Let ROS handle all callbacks.
      ros::spinOnce();
  }


  return 0;
}  // end main()
