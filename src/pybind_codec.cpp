
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rcl/types.h>
#include <rcl/allocator.h>
#include <rclcpp/serialization.hpp>

#include <point_cloud_transport/point_cloud_codec.hpp>

namespace py = pybind11;
using namespace point_cloud_transport;

// Utilities for handling the transfer of information to/from python
namespace point_cloud_transport
{
    void stringToSerializedMsg(const std::string &buffer, rclcpp::SerializedMessage &serial_msg)
    {
        rcl_serialized_message_t raw_serialized_msg = rmw_get_zero_initialized_serialized_message();

        // Allocate memory for the serialized message
        raw_serialized_msg.buffer_capacity = buffer.size();
        raw_serialized_msg.buffer_length = buffer.size();
        raw_serialized_msg.buffer = static_cast<uint8_t *>(malloc(raw_serialized_msg.buffer_capacity));

        if (!raw_serialized_msg.buffer)
        {
            // Handle memory allocation error
            // For simplicity, we just throw an exception here
            throw std::runtime_error("Failed to allocate memory for serialized message.");
        }

        // Copy the string data into the serialized message buffer
        memcpy(raw_serialized_msg.buffer, buffer.c_str(), raw_serialized_msg.buffer_length);

        serial_msg = rclcpp::SerializedMessage(raw_serialized_msg);
    }

    void serializedMsgToString(const rclcpp::SerializedMessage &serial_msg, std::string &buffer)
    {
        buffer = std::string(reinterpret_cast<const char *>(serial_msg.get_rcl_serialized_message().buffer), serial_msg.get_rcl_serialized_message().buffer_length);
    }

    void stringToPointCloud2(const std::string &buffer, sensor_msgs::msg::PointCloud2 &cloud)
    {
        // serialize the pointcloud2 msg and use stringToSerialziedMsg
        auto serialized_msg_ptr = std::make_shared<rclcpp::SerializedMessage>();
        stringToSerializedMsg(buffer, *serialized_msg_ptr);

        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> deserializer;
        deserializer.deserialize_message(serialized_msg_ptr.get(), &(cloud));
    }

    void pointCloud2ToString(const sensor_msgs::msg::PointCloud2 &cloud, std::string &buffer)
    {
        // serialize the pointcloud2 msg and use stringToSerialziedMsg
        auto serialized_msg_ptr = std::make_shared<rclcpp::SerializedMessage>();
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
        serializer.serialize_message(&(cloud), serialized_msg_ptr.get());
        serializedMsgToString(*serialized_msg_ptr, buffer);
    }
}

PYBIND11_MAKE_OPAQUE(std::vector<std::string>);

PYBIND11_MODULE(_codec, m)
{

    py::bind_vector<std::vector<std::string>>(m, "VectorString");

    py::class_<PointCloudCodec>(m, "PointCloudCodec")
        .def(pybind11::init())
        .def("getLoadableTransports", &PointCloudCodec::getLoadableTransports)
        .def("getTopicsToPublish", &PointCloudCodec::getTopicsToPublish)
        .def("getTopicToSubscribe", [](PointCloudCodec &codec, const std::string &baseTopic, const std::string &transport)
             {
            std::string topic;
            std::string name;
            std::string dataType;
            codec.getTopicToSubscribe(baseTopic, transport, topic, name, dataType);
            return std::make_tuple(transport, topic, name, dataType); })
        .def("encode", [](PointCloudCodec &codec, const std::string &transport_name, const std::string &pointcloud_buffer)
             {
            sensor_msgs::msg::PointCloud2 msg;
            stringToPointCloud2(pointcloud_buffer, msg);

            rclcpp::SerializedMessage serialized_msg;
            codec.encode(transport_name, msg, serialized_msg);

            std::string serialized_buffer;
            serializedMsgToString(serialized_msg, serialized_buffer);
            return serialized_buffer; }
            )
        .def("decode", [](PointCloudCodec &codec, const std::string &transport_name, const std::string &serialized_buffer)
             {
            rclcpp::SerializedMessage serialized_msg;
            stringToSerializedMsg(serialized_buffer, serialized_msg);

            sensor_msgs::msg::PointCloud2 msg;
            codec.decode(transport_name, serialized_msg, msg);

            std::string buffer;
            pointCloud2ToString(msg, buffer);
            return buffer; }
        )
}
