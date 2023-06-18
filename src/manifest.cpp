// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague .. 2019, paplhjak .. 2009

#include <pluginlib/class_list_macros.hpp>

#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/raw_publisher.hpp>
#include <point_cloud_transport/raw_subscriber.hpp>
#include <point_cloud_transport/subscriber_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(point_cloud_transport::RawPublisher, point_cloud_transport::PublisherPlugin)
PLUGINLIB_EXPORT_CLASS(point_cloud_transport::RawSubscriber, point_cloud_transport::SubscriberPlugin)
