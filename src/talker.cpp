#include <talker.h>


namespace point_cloud_transport
{
    Talker::Talker(ros::NodeHandle nh) : nh_(nh), decode_speed_(0), encode_speed_(0)
    {
        // Set up a dynamic reconfigure server.
        // Do this before parameter server, else some of the parameter server values can be overwritten.
        dynamic_reconfigure::Server<point_cloud_transport::PointCloudConfig>::CallbackType cb;
        cb = boost::bind(&Talker::configCallback, this, _1, _2);
        dr_srv_.setCallback(cb);


        // Declare variables that can be modified by launch file or command line.
        double rate = 10;


        // Initialize node parameters from launch file or command line. Use a private node handle so that multiple instances
        // of the node can be run simultaneously while using different parameters.
        ros::NodeHandle pnh("~");

        pnh.param("enable", enable_, enable_);
        pnh.param("encode_speed", encode_speed_, encode_speed_);
        pnh.param("decode_speed", decode_speed_, decode_speed_);
        pnh.param("quantization_POSITION",POSITION_quantization_in_bits_, POSITION_quantization_in_bits_);
        pnh.param("quantization_NORMAL",NORMAL_quantization_in_bits_, NORMAL_quantization_in_bits_);
        pnh.param("quantization_COLOR",COLOR_quantization_in_bits_, COLOR_quantization_in_bits_);
        pnh.param("quantization_TEX_COORD",TEX_COORD_quantization_in_bits_, TEX_COORD_quantization_in_bits_);
        pnh.param("quantization_GENERIC",GENERIC_quantization_in_bits_, GENERIC_quantization_in_bits_);

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
  pub_ = nh_.advertise<point_cloud_transport::CompressedPointCloud2>("compressed_PC2", 1);
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

    extern sensor_msgs::PointCloud2 msg;

    PC2toDraco converter(msg);

    std::unique_ptr<draco::PointCloud> pc = converter.convert();

    draco::EncoderBuffer encode_buffer;

    draco::Encoder encoder;

    //! set quantization of attribute types
    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, POSITION_quantization_in_bits_);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::NORMAL, NORMAL_quantization_in_bits_);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, COLOR_quantization_in_bits_);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::TEX_COORD, TEX_COORD_quantization_in_bits_);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, GENERIC_quantization_in_bits_);


    encoder.SetSpeedOptions(encode_speed_, decode_speed_);

    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);

    encoder.SetTrackEncodedProperties(false);

    encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);

    point_cloud_transport::CompressedPointCloud2 compressed_PC2;

    pub_.publish(compressed_PC2);
}


void Talker::configCallback(point_cloud_transport::PointCloudConfig &config, uint32_t level __attribute__((unused)))
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
  encode_speed_ = config.encode_speed;
  decode_speed_ = config.decode_speed;

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
