#include <point_cloud_transport/raw_publisher.h>
#include <ros/static_assert.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudTransportPointCloud
{
public:
    sensor_msgs::PointCloud2 PC2_; //!< ROS header
    const uint8_t* data_;           //!< Image data for use with OpenCV

    /**
     Empty constructor.
     */
    PointCloudTransportPointCloud() {}

    /**
     Constructor.
     */
    PointCloudTransportPointCloud(const sensor_msgs::PointCloud2& point_cloud, const uint8_t* data)
            : PC2_(point_cloud), data_(data)
    {
    }
};

/// @cond DOXYGEN_IGNORE
namespace ros {

    namespace message_traits {

        template<> struct MD5Sum<PointCloudTransportPointCloud>
        {
            static const char* value() { return MD5Sum<sensor_msgs::PointCloud2>::value(); }
            static const char* value(const PointCloudTransportPointCloud&) { return value(); }

            static const uint64_t static_value1 = MD5Sum<sensor_msgs::PointCloud2>::static_value1;
            static const uint64_t static_value2 = MD5Sum<sensor_msgs::PointCloud2>::static_value2;

            // If the definition of sensor_msgs/PointCloud2 changes, we'll get a compile error here.
            //ROS_STATIC_ASSERT(MD5Sum<sensor_msgs::PointCloud2>::static_value1 == 0x060021388200f6f0ULL);
            //ROS_STATIC_ASSERT(MD5Sum<sensor_msgs::PointCloud2>::static_value2 == 0xf447d0fcd9c64743ULL);
        };

        template<> struct DataType<PointCloudTransportPointCloud>
        {
            static const char* value() { return DataType<sensor_msgs::PointCloud2>::value(); }
            static const char* value(const PointCloudTransportPointCloud&) { return value(); }
        };

        template<> struct Definition<PointCloudTransportPointCloud>
        {
            static const char* value() { return Definition<sensor_msgs::PointCloud2>::value(); }
            static const char* value(const PointCloudTransportPointCloud&) { return value(); }
        };

        template<> struct HasHeader<PointCloudTransportPointCloud> : TrueType {};

    } // namespace ros::message_traits

    namespace serialization {

        template<> struct Serializer<PointCloudTransportPointCloud>
        {

            template<typename Stream>
            inline static void write(Stream& stream, const PointCloudTransportPointCloud& m)
            {
                stream.next(m.PC2_.header);
                stream.next((uint32_t)m.PC2_.height); // height
                stream.next((uint32_t)m.PC2_.width); // width
                stream.next(m.PC2_.fields);
                bool is_bigendian = false;
                stream.next(is_bigendian);
                stream.next((uint32_t)m.PC2_.point_step);
                stream.next((uint32_t)m.PC2_.row_step);
                size_t data_size = m.PC2_.point_step*m.PC2_.height*m.PC2_.width;
                stream.next((uint32_t)data_size);
                if (data_size > 0)
                    memcpy(stream.advance(data_size), m.data_, data_size);
                bool is_dense = false;
            }

            inline static uint32_t serializedLength(const PointCloudTransportPointCloud& m)
            {
                size_t data_size = m.PC2_.point_step*m.PC2_.height*m.PC2_.width;
                // TODO: make sure serialization is working properly under all circumstances
                return serializationLength(m.PC2_.header) + serializationLength(m.PC2_.fields) + 18 + data_size;
            }
        };

    } // namespace ros::serialization

} // namespace ros


namespace point_cloud_transport {

    void RawPublisher::publish(const sensor_msgs::PointCloud2& message, const uint8_t* data) const
    {
        getPublisher().publish(PointCloudTransportPointCloud(message, data));
    }

} //namespace point_cloud_transport

