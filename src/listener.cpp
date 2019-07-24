#include <listener.h>
#include "debug_msg.h"

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

    uint32_t compressed_data_size = msg.compressed_data.size();

    //!
    if (compressed_data_size==0)
    {
     return ;
    }

    //DBGVAR(std::cout, compressed_data_size);

    std::vector <unsigned char> vec_data = (msg.compressed_data);

    //DBGVAR(std::cout, compressed_data_size);


    //! decoding

    draco::DecoderBuffer decode_buffer;

    //DBGVAR(std::cout, compressed_data_size);


    // Sets the buffer's internal data. Note that no copy of the input data is
    // made so the data owner needs to keep the data valid and unchanged for
    // runtime of the decoder.
    decode_buffer.Init(reinterpret_cast<const char *>(&vec_data[0]), compressed_data_size);

    //DBGVAR(std::cout, compressed_data_size);


    draco::Decoder decoder;

    //DBGVAR(std::cout, compressed_data_size);


    std::unique_ptr<draco::PointCloud> decoded_pc =
            decoder.DecodePointCloudFromBuffer(&decode_buffer).value();

    //DBGVAR(std::cout, decode_buffer.decoded_size());
    //DBGVAR(std::cout, decode_buffer.remaining_size());



    //! Draco to PC2

    InfoPointCloud2 infoPC2;

    infoPC2.header = msg.header;
    infoPC2.height = msg.height;
    infoPC2.width = msg.width;
    infoPC2.fields = msg.fields;
    infoPC2.is_bigendian = msg.is_bigendian;
    infoPC2.point_step = msg.point_step;
    infoPC2.row_step = msg.row_step;
    infoPC2.is_dense = msg.is_dense;

    //DBGVAR(std::cout, compressed_data_size);


    DracotoPC2 converter_b(std::move(decoded_pc),(infoPC2));

    //DBGVAR(std::cout, compressed_data_size);

    global_decoded_PC2 = converter_b.convert();

    //DBGVAR(std::cout, compressed_data_size);

}
}
