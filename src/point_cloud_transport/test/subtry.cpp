// this file was originally created for purpose of learning to use ROS
// currently it serves for testing google Draco library functions
// and user defined classes

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <iomanip> // for std::setprecision and std::fixed
#include "PC2toDraco.h"
#include "DracotoPC2.h"
#include "draco/compression/decode.h"
#include "draco/compression/encode.h"


// a callback function executed each time a new message arrives
void PC2MessageReceived(const sensor_msgs::PointCloud2 & msg){
    ROS_INFO_STREAM(std::setprecision(2) << std::fixed);
    //std::cout << "Received PC2, size: " << msg.width*msg.height << std::endl;


    sensor_msgs::PointCloud2 cloud_msg = msg;

    const char *PointFieldDataTypes[] =
            {
                    (char*)"Undefined",
                    (char*)"INT8",
                    (char*)"UINT8",
                    (char*)"INT16",
                    (char*)"UINT16",
                    (char*)"INT32",
                    (char*)"UINT32",
                    (char*)"FLOAT32",
                    (char*)"FLOAT64"
            };

    const char *BoolDataTypes[] =
            {
                    (char*)"False",
                    (char*)"True"
            };


    std::cout << "Header: " << cloud_msg.header << "\nHeight: " << cloud_msg.height << "\nWidth: " << cloud_msg.width <<
              "\nFields[0]: " << cloud_msg.fields[0].name << "\nBigendian: " << BoolDataTypes[cloud_msg.is_bigendian]
              << "\nPoint step: " << cloud_msg.point_step <<
              "\nRow step: " << cloud_msg.row_step << "\nData[0]: " << cloud_msg.data[0] << "\nIs dense: "
              << BoolDataTypes[cloud_msg.is_dense] << std::endl;

    std::cout << "\n\nFields:" << std::endl;
    for (size_t i = 4; i--;) {
        std::cout << cloud_msg.fields[i].name << " " << cloud_msg.fields[i].offset << " "
                  << PointFieldDataTypes[cloud_msg.fields[i].datatype] << " " << cloud_msg.fields[i].count << std::endl;
    }


    PC2toDraco converter(cloud_msg);

    std::unique_ptr<draco::PointCloud> pc = converter.convert();

    //! encoding

    draco::EncoderBuffer encode_buffer;

    draco::Encoder encoder;

    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);

    encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, 8);

    encoder.SetAttributeQuantization(draco::GeometryAttribute::GENERIC, 14);


    int encode_speed = 0, decode_speed = 0;
    encoder.SetSpeedOptions(encode_speed, decode_speed);
    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);
    encoder.SetTrackEncodedProperties(false);

    std::cout<< "Encoding succesful: " << encoder.EncodePointCloudToBuffer(*pc, &encode_buffer).ok() << "\nNumber of encoded points: " << encoder.num_encoded_points() << "\nSize of encoded buffer: "<< encode_buffer.size() <<"\n" <<std::endl;

    //! decoding

    draco::DecoderBuffer decode_buffer;

    decode_buffer.Init(encode_buffer.data(), encode_buffer.size());

    draco::Decoder decoder;

    std::unique_ptr<draco::PointCloud> decoded_pc =
            decoder.DecodePointCloudFromBuffer(&decode_buffer).value();

    int32_t number_of_attributes = decoded_pc->num_attributes();

    //std::cout  << sizeof(pc) << " " << sizeof(decoded_pc) << std::endl;

    //int n=memcmp (&pc, &decoded_pc, sizeof(pc));


    std::cout << "Sizes: " << pc->num_points() << " " << decoded_pc->num_points() << "     " << decoded_pc->num_attributes() << std::endl;


    //!

    int32_t number_of_decoded_attributes = decoded_pc->num_attributes();
    std::vector<int> att_ids;

    const draco::PointAttribute* attribute0=decoded_pc->GetAttributeByUniqueId(0);

    std::cout << " att type: " << attribute0->attribute_type() << " att data type: " << attribute0->data_type() << " num of components: " << int8_t(attribute0->num_components()) << " byte stride: " << attribute0->byte_stride() << " offset: "<< attribute0->byte_offset() << std::endl;

    int num_of_decoded_points = decoded_pc->num_points();
    float float_arr[num_of_decoded_points];
    for (int i=0; i<num_of_decoded_points; i++)
    {
        attribute0->GetMappedValue(draco::PointIndex (i), &float_arr[i]);

        //std::cout << float_arr[i] << std::endl;
    }

    //! DracotoPC2 testing

    sensor_msgs::PointCloud2 new_PC2;

    DracotoPC2 converter_b( std::move(decoded_pc), &cloud_msg.fields[0], cloud_msg.point_step);

    new_PC2 = converter_b.convert();

    new_PC2.header=msg.header;
    new_PC2.fields=msg.fields;
    new_PC2.is_bigendian=msg.is_bigendian;
    new_PC2.is_dense=msg.is_dense;
    //new_PC2.header=msg.header;


    std::cout << " FULL PROCESS FINISHED" << std::endl;
}

int main(int argc, char**argv){
    // initialize the ROS system
    ros::init(argc, argv, "subscribe_to_pose");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(
            "/new_point_cloud", 1000); // 1000 is size of queue


    // create a subscriber object
    ros::Subscriber sub = nh.subscribe("/dynamic_point_cloud", 1000, &PC2MessageReceived);

    // let ROS take over
    ros::spin();

    return 0;
}
