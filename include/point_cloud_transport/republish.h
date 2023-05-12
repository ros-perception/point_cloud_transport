#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief A node(let) for republishing clouds from one transport to other transports.
 * \author Martin Pecka
 */

#include <memory>

#include <boost/shared_ptr.hpp>

#include <cras_cpp_common/nodelet_utils.hpp>

#include <point_cloud_transport/point_cloud_transport.h>
#include <point_cloud_transport/publisher.h>
#include <point_cloud_transport/publisher_plugin.h>
#include <point_cloud_transport/subscriber.h>


namespace point_cloud_transport
{

//! A node(let) for republishing clouds from one transport to other transports
//!
//! Usage: republish in_transport in:=<in_base_topic> [out_transport] out:=<out_base_topic>
//!
//! Parameters:
//! - `~in_queue_size` (int, default 10): Input queue size.
//! - `~out_queue_size` (int, default `in_queue_size`): Output queue size.
class RepublishNodelet : public cras::Nodelet
{
protected:
  void onInit() override;

  std::unique_ptr<point_cloud_transport::PointCloudTransport> pct;
  point_cloud_transport::Subscriber sub;
  boost::shared_ptr<point_cloud_transport::Publisher> pub;
  boost::shared_ptr<point_cloud_transport::PublisherPlugin> pubPlugin;
};

}
