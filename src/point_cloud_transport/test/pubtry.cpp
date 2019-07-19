// this file was originally created for purpose of learning to use ROS
// currently it serves for testing google Draco library functions
// and user defined classes


#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h> // for rand() and RAND_MAX
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
//#include "draco/point_cloud/point_cloud_builder.h"
#include "PC2toDraco.h"
#include "draco/compression/decode.h"
#include "draco/compression/encode.h"

int main(int argc, char** argv) {
    // initialize the ROS system and become a node
    ros::init(argc, argv, "publish_velocity");
    ros::NodeHandle nh;

    // create a publisher object
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>(
            "turtle1/cmd_vel", 1000); // 1000 is size of queue

    // seed random number generator
    srand(time(0));


    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.height = 1;
    cloud_msg.width = 4;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::UINT32,
                                  "z", 1, sensor_msgs::PointField::INT32,
                                  "rgb", 1, sensor_msgs::PointField::UINT8);


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

    std::cout << "\nFields size: " << sizeof(cloud_msg.fields) << std::endl;


    for (sensor_msgs::PointField i : cloud_msg.fields) {
        std::cout << i.name << ' ';
    }

    std::cout << std::endl;

    PC2toDraco converter(cloud_msg);

    std::unique_ptr<draco::PointCloud> pc = converter.convert();
    std::cout << pc->num_attributes() << std::endl ;


    draco::EncoderBuffer encode_buffer;

    draco::Encoder encoder;

    encoder.SetTrackEncodedProperties(1);

    encoder.SetAttributeQuantization(draco::GeometryAttribute::POSITION, 14);

    encoder.SetAttributeQuantization(draco::GeometryAttribute::COLOR, 8);


    //  0 = slowest speed, but the best compression.
    // 10 = fastest, but the worst compression.
    // -1 = undefined.
    int encode_speed = 0, decode_speed = 0;
    encoder.SetSpeedOptions(encode_speed, decode_speed);

    encoder.SetEncodingMethod(draco::POINT_CLOUD_KD_TREE_ENCODING);

    std::cout<< "LOOOK HERE: " << encoder.EncodePointCloudToBuffer(*pc, &encode_buffer).ok() << "\n and here: " << encoder.num_encoded_points() << encode_buffer.size()<<std::endl;

    draco::DecoderBuffer decode_buffer;

    decode_buffer.Init(encode_buffer.data(), encode_buffer.size());

    draco::Decoder decoder;

    std::unique_ptr<draco::PointCloud> decoded_pc =
            decoder.DecodePointCloudFromBuffer(&decode_buffer).value();


    std::cout << "\n \nDecoded: " << decoded_pc->NumNamedAttributes(draco::GeometryAttribute::POSITION) << std::endl;



    // loop at 2Hz until node is shut down
    ros::Rate rate(2);
    while (ros::ok()) {
        // create and fill the message, empty fields are defaulted to 0
        geometry_msgs::Twist msg;
        msg.linear.x = double(rand()) / double(RAND_MAX);
        msg.angular.z = 2 * double(rand()) / double(RAND_MAX) - 1;

        // publish the message
        pub.publish(msg);

        // send a message to rosout with the details
        ROS_INFO_STREAM("Sending random velocity command:"
                                << " linear=" << msg.linear.x << " angular=" << msg.angular.z);

        // wait until it is time for another iteration
        rate.sleep();
    }
}
