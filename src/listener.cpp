#include <listener.h>
#include "debug_msg.h"

// TODO: make sure all data is passed by reference without making copies if possible
// TODO: rewrite in style of image_transport

//!
sensor_msgs::PointCloud2 global_decoded_PC2;

namespace point_cloud_transport
{
Listener::Listener(ros::NodeHandle nh)
{
  // Create a subscriber.
  // Name the topic, message queue, callback function with class name, and object containing callback function.
  sub_ = nh.subscribe("/compressed_PC2", 1, &Listener::messageCallback, this);
}

void Listener::messageCallback(const CompressedPointCloud2 &msg)
{

    // get size of buffer with compressed data in Bytes
    uint32_t compressed_data_size = msg.compressed_data.size();

    // if buffer is empty, message is "point-less", return
    if (compressed_data_size==0)
    {
     return ;
    }

    //! --- decoding - START ---

    draco::DecoderBuffer decode_buffer;

    std::vector <unsigned char> vec_data = (msg.compressed_data);

    // Sets the buffer's internal data. Note that no copy of the input data is
    // made so the data owner needs to keep the data valid and unchanged for
    // runtime of the decoder.
    decode_buffer.Init(reinterpret_cast<const char *>(&vec_data[0]), compressed_data_size);

    // create decoder object
    draco::Decoder decoder;

    // decode buffer into draco point cloud
    std::unique_ptr<draco::PointCloud> decoded_pc =
            decoder.DecodePointCloudFromBuffer(&decode_buffer).value();

    //! --- decoding - END ---

    //! --- Draco to PC2 - START ---

    // create and initiate converter object
    DracotoPC2 converter_b(std::move(decoded_pc), msg);
    // convert draco point cloud to sensor_msgs::PointCloud2
    global_decoded_PC2 = converter_b.convert();

    //! --- Draco to PC2 - END ---

}
}
