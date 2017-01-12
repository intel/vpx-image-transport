// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef VPX_PUBLISHER_H
#define VPX_PUBLISHER_H

#include <dynamic_reconfigure/server.h>
#include <image_transport/simple_publisher_plugin.h>
#include <memory>
#include <opencv2/imgproc/imgproc.hpp>
#include <std_msgs/Header.h>
#include <vpx_image_transport/Packet.h>
#include <vpx_image_transport/VPXPublisherConfig.h>
#include <vpx/vpx_encoder.h>
#include <webm_live_muxer.h>
#include "encoder.h"

namespace vpx_image_transport {

class YamiEncoder;

class VPXPublisher : public image_transport::SimplePublisherPlugin<vpx_image_transport::Packet>,
                     public EncoderDelegate
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

private:
  // Dynamic reconfigure support
  typedef vpx_image_transport::VPXPublisherConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  mutable vpx_codec_ctx_t* codec_context_;
  mutable vpx_codec_enc_cfg_t* encoder_config_;
  mutable uint64_t frame_count_;
  mutable uint64_t package_sequence_;
  mutable unsigned int keyframe_forced_interval_;
  mutable webm_tools::WebMLiveMuxer* muxer_;

  mutable boost::shared_ptr<YamiEncoder> yami_encoder_;

  void configCallback(Config& config, uint32_t level);
  void sendChunkIfReady(const PublishFn &publish_fn) const;
  bool createVPXEncoder(int frame_width, int frame_height) const;
  void encodeWithVPX(const cv::Mat& mat) const;

  // EncoderDelegate implementation.
  virtual void onWriteFrame(uint8_t* buffer, uint64_t size,
                            uint64_t timeStamp, bool isKeyFrame);
};

} //namespace vpx_image_transport


#endif // VPX_PUBLISHER_H
