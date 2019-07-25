//
// Created by jakub on 7/25/19.
//

#include <pluginlib/class_list_macros.h>
#include "point_cloud_transport/raw_publisher.h"
#include "point_cloud_transport/raw_subscriber.h"

PLUGINLIB_EXPORT_CLASS( point_cloud_transport::RawPublisher, point_cloud_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS( point_cloud_transport::RawSubscriber, point_cloud_transport::SubscriberPlugin)
