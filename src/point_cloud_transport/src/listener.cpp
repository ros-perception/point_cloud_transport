#include <listener.h>

namespace point_cloud_transport
{
Listener::Listener(ros::NodeHandle nh)
{
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh.subscribe("test_topic", 10, &Listener::messageCallback, this);
  counter_ = 0;
}

void Listener::messageCallback(const point_cloud_transport::PointCloudTransportData::ConstPtr &msg)
{
  // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // Typically, they should be DEBUG.
  ROS_INFO("\nString: %s,\nNumber: %d", msg->string_a.c_str(), msg->number_a);

  std::cout <<this->counter_++ << " " << msg->number_a;
}
}
