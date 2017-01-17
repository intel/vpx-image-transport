// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef VPX_PUBLISHER_H
#define VPX_PUBLISHER_H

#include <dynamic_reconfigure/server.h>
#include <image_transport/simple_publisher_plugin.h>
#include <std_msgs/Header.h>
#include <vector>
#include <vpx_image_transport/Packet.h>
#include <vpx_image_transport/VPXPublisherConfig.h>
#include "stream_muxer.h"

namespace vpx_image_transport {

class VPXPublisher : public image_transport::SimplePublisherPlugin<vpx_image_transport::Packet>,
                     public vpx_streamer::StreamMuxerDelegate
{
public:
  VPXPublisher();
  ~VPXPublisher();

  virtual std::string getTransportName() const { return "vpx"; }

protected:
  virtual void advertiseImpl(ros::NodeHandle &nh, const std::string& base_topic,
      uint32_t queue_size,
      const image_transport::SubscriberStatusCallback& user_connect_cb,
      const image_transport::SubscriberStatusCallback& user_disconnect_cb,
      const ros::VoidPtr& tracked_object,
      bool latch);

  virtual void publish(const sensor_msgs::Image& message,
                       const PublishFn& publish_fn) const;
  virtual void connectCallback(const ros::SingleSubscriberPublisher& pub);
  virtual void disconnectCallback(const ros::SingleSubscriberPublisher &pub);

  // StreamMuxerDelegate implementation
  virtual void onChunkReady(std::vector<uint8_t>& buffer);

private:
  // Dynamic reconfigure support
  typedef vpx_image_transport::VPXPublisherConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;

  mutable uint64_t package_sequence_;
  mutable vpx_streamer::StreamMuxer stream_muxer_;
  mutable const PublishFn* publish_function_;
  void configCallback(Config& config, uint32_t level);
};

} //namespace vpx_image_transport


#endif // VPX_PUBLISHER_H
