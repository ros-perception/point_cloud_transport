#include "Repubtry.h"

//! used to share PC2 from callback function, global object was the simplest solution
sensor_msgs::PointCloud2  PC2_;
point_cloud_transport::CompressedPointCloud2 compressed_PC2_;

//! Constructor
Repubtry::Repubtry(ros::NodeHandle nh)
{
nh_=nh;
pub_ = nh.advertise<sensor_msgs::PointCloud2>("/new_point_cloud", 1);
pub2_ = nh.advertise<point_cloud_transport::CompressedPointCloud2>("/compressed_PC2",100);
sub_ = nh.subscribe("/dynamic_point_cloud", 100, &MessageReceived);
}

//! Destructor
Repubtry::~Repubtry(){}

//! callback
void MessageReceived(const sensor_msgs::PointCloud2 & msg)
{

    sensor_msgs::PointCloud2 cloud_msg = msg;

    std::cout << cloud_msg.header.frame_id << std::endl;

    //! PC2 to Draco

    PC2toDraco converter(cloud_msg);
    std::unique_ptr<draco::PointCloud> pc = converter.convert();

    //! encoding

    draco::EncoderBuffer encode_buffer;

    draco::Encoder encoder;

    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, 8);
    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 14);


    int encode_speed = 10, decode_speed = 10;
    encoder.SetSpeedOptions(encode_speed, decode_speed);
    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    encoder.SetTrackEncodedProperties(false);
    encoder.EncodePointCloudToBuffer(*pc, &encode_buffer);


/*
    //! decoding

    draco::DecoderBuffer decode_buffer;

    decode_buffer.Init(encode_buffer.data(), encode_buffer.size());

    draco::Decoder decoder;

    std::unique_ptr<draco::PointCloud> decoded_pc =
            decoder.DecodePointCloudFromBuffer(&decode_buffer).value();


    //! Draco to PC2

    point_cloud_transport::InfoPointCloud2 infoPC2;

    infoPC2.header = cloud_msg.header;
    infoPC2.height = cloud_msg.height;
    infoPC2.width = cloud_msg.width;
    infoPC2.fields = cloud_msg.fields;
    infoPC2.is_bigendian = cloud_msg.is_bigendian;
    infoPC2.point_step = cloud_msg.point_step;
    infoPC2.row_step = cloud_msg.row_step;
    infoPC2.is_dense = cloud_msg.is_dense;

    sensor_msgs::PointCloud2 new_PC2;

    DracotoPC2 converter_b(std::move(decoded_pc), infoPC2);

    new_PC2 = converter_b.convert();

    PC2_=(new_PC2);

    */

    //! set compressed point cloud to advertise

    point_cloud_transport::CompressedPointCloud2 new_compressed_PC2;

    std::vector<unsigned char> vec_data(encode_buffer.data(), encode_buffer.data()+encode_buffer.size());

    new_compressed_PC2.compressed_data = std::move(vec_data);

    new_compressed_PC2.header = cloud_msg.header;
    new_compressed_PC2.height = cloud_msg.height;
    new_compressed_PC2.width = cloud_msg.width;
    new_compressed_PC2.fields = cloud_msg.fields;
    new_compressed_PC2.is_bigendian = cloud_msg.is_bigendian;
    new_compressed_PC2.point_step = cloud_msg.point_step;
    new_compressed_PC2.row_step = cloud_msg.row_step;
    new_compressed_PC2.is_dense = cloud_msg.is_dense;

    compressed_PC2_=new_compressed_PC2;

}
