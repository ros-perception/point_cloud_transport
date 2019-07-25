//
// Created by jakub on 7/25/19.
//

#include <image_transport/raw_publisher.h>
#include <sensor_msgs/PointCloud2.h>



namespace point_cloud_transport {

    void RawPublisher::publish(const sensor_msgs::PointCloud2& message) const
    {
        getPublisher().publish((message);
    }

} // namespace point_cloud_transport

