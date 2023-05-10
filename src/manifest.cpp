// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak .. 2009

#include <pluginlib/class_list_macros.h>

#include <point_cloud_transport/publisher_plugin.h>
#include <point_cloud_transport/raw_publisher.h>
#include <point_cloud_transport/raw_subscriber.h>
#include <point_cloud_transport/subscriber_plugin.h>

PLUGINLIB_EXPORT_CLASS(point_cloud_transport::RawPublisher, point_cloud_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(point_cloud_transport::RawSubscriber, point_cloud_transport::SubscriberPlugin)
