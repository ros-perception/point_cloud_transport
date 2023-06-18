#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief A node(let) for republishing clouds from one transport to other transports.
 * \author Martin Pecka
 */

#include <memory>

#include <point_cloud_transport/point_cloud_transport.hpp>
#include <point_cloud_transport/publisher.hpp>
#include <point_cloud_transport/publisher_plugin.hpp>
#include <point_cloud_transport/subscriber.hpp>


namespace point_cloud_transport
{

//! A Node (component) for republishing clouds from one transport to other transports
//!
//! Usage: republish in_transport in:=<in_base_topic> [out_transport] out:=<out_base_topic>
//!
//! Parameters:
//! - `~in_queue_size` (int, default 10): Input queue size.
//! - `~out_queue_size` (int, default `in_queue_size`): Output queue size.
class RepublishComponent
{
protected:
  void onInit() override;

  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct;
  point_cloud_transport::Subscriber sub;
  std::shared_ptr<point_cloud_transport::Publisher> pub;
  std::shared_ptr<point_cloud_transport::PublisherPlugin> pubPlugin;
};

}
