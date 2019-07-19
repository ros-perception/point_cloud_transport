#include <talker.h>


namespace point_cloud_transport
{
    Talker::Talker(ros::NodeHandle nh) : nh_(nh), teststring_("string"), testint_(0)
    {
        // Set up a dynamic reconfigure server.
        // Do this before parameter server, else some of the parameter server values can be overwritten.
        dynamic_reconfigure::Server<point_cloud_transport::PointCloudConfig>::CallbackType cb;
        cb = boost::bind(&Talker::configCallback, this, _1, _2);
        dr_srv_.setCallback(cb);

        // Declare variables that can be modified by launch file or command line.
        double rate = 1.0;

        // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
        // of the node can be run simultaneously while using different parameters.
        ros::NodeHandle pnh("~");

        pnh.param("enable", enable_, enable_);
        pnh.param("teststring", teststring_, teststring_);
        pnh.param("testint", testint_, testint_);

        // Create a publisher and name the topic.
  if (enable_)
  {
    start();
  }

  // Create timer.
  timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &Talker::timerCallback, this);
}

void Talker::start()
{
  pub_ = nh_.advertise<point_cloud_transport::PointCloudTransportData>("test_topic", 100);
}

void Talker::stop()
{
  pub_.shutdown();
}

void Talker::timerCallback(const ros::TimerEvent &event __attribute__((unused)))
{
  if (!enable_)
  {
    return;
  }

  point_cloud_transport::PointCloudTransportData msg;
  msg.string_a = teststring_;
  msg.number_a = testint_;

  pub_.publish(msg);
}

void Talker::configCallback(point_cloud_transport::PointCloudConfig &config, uint32_t level __attribute__((unused)))
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  teststring_ = config.teststring;
  testint_ = config.testint;

  // Check if we are changing enabled state.
  if (enable_ != config.enable)
  {
    if (config.enable)
    {
      start();
    }
    else
    {
      stop();
    }
  }
  enable_ = config.enable;

}
}
