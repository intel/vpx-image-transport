// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_publisher.h"

#include <cv_bridge/cv_bridge.h>
#include "yami_encoder.h"
#include "software_encoder.h"

namespace vpx_image_transport {

  using namespace webm_tools;
  using namespace mkvmuxer;

VPXPublisher::VPXPublisher()
  : package_sequence_(0), muxer_(NULL), yami_encoder_(new YamiEncoder(this)),
    software_encoder_(new SoftwareEncoder(this)) {
}

VPXPublisher::~VPXPublisher() {
  muxer_->Finalize();
  delete muxer_;
}

void VPXPublisher::advertiseImpl(ros::NodeHandle &nh,
    const std::string& base_topic,
    uint32_t queue_size,
    const image_transport::SubscriberStatusCallback& user_connect_cb,
    const image_transport::SubscriberStatusCallback& user_disconnect_cb,
    const ros::VoidPtr& tracked_object,
    bool latch) {
  typedef image_transport::SimplePublisherPlugin<vpx_image_transport::Packet> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb,
    user_disconnect_cb, tracked_object, latch);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f =
      boost::bind(&VPXPublisher::configCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

void VPXPublisher::configCallback(Config& config, uint32_t level) {
  EncoderConfig conf;

  conf.target_bitrate = config.target_bitrate;
  conf.keyframe_forced_interval = config.keyframe_forced_interval;

  yami_encoder_->configure(conf);
  software_encoder_->configure(conf);
}

void VPXPublisher::publish(const sensor_msgs::Image& message,
                           const PublishFn& publish_fn) const {
  // conversion necessary
  std::string encoding;
  if (sensor_msgs::image_encodings::isColor(message.encoding)
      || sensor_msgs::image_encodings::isMono(message.encoding)) {
    encoding = sensor_msgs::image_encodings::BGR8;
  } else if (message.encoding == std::string("16UC1")) {
    encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  } else if (message.encoding == std::string("8UC1")) {
    encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  } else {
    ROS_ERROR("VPX publisher is not able to handle encoding type:%s", message.encoding.c_str());
    return;
  }

  cv_bridge::CvImageConstPtr cv_image_ptr;
  try {
    cv_image_ptr = cv_bridge::toCvCopy(message, encoding);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: '%s'", e.what());
    return;
  } catch (cv::Exception& e) {
    ROS_ERROR("OpenCV exception: '%s'", e.what());
    return;
  }
  if (cv_image_ptr == 0) {
    ROS_ERROR("Unable to convert from '%s' to '%s'", message.encoding.c_str(), encoding.c_str());
    return;
  }

  int frame_width = message.width, frame_height = message.height;
  cv::Mat bgr;
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    cv::Mat gray8;
    cv_image_ptr->image.convertTo(gray8, CV_8UC1);
    cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
  } else if (encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
    cv::Mat gray8 = cv_image_ptr->image;
    cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
  } else {
    bgr = cv_image_ptr->image;
  }

  if (!yami_encoder_->initialized() && !software_encoder_->initialized()) {
    if (yami_encoder_->createEncoder(message.width, message.height)) {
      ROS_INFO("Hardware accelerated encoder enabled.");
    } else if (!software_encoder_->createEncoder(message.width, message.height)){
      ROS_WARN("Failed to create encoder, will retry.");
      return;
    }
  }

  if (!muxer_->initialized()) {
    muxer_->Init();
    muxer_->AddVideoTrack(frame_width, frame_height);
  }

  if (yami_encoder_->initialized()) {
    yami_encoder_->encode(bgr);
  } else if (software_encoder_->initialized()) {
    software_encoder_->encode(bgr);
  }

  sendChunkIfReady(publish_fn);
}

void VPXPublisher::sendChunkIfReady(const PublishFn &publish_fn) const {
  webm_tools::int32 chunk_length = 0;
  if (!muxer_->ChunkReady(&chunk_length)) {
    return;
  }

  vpx_image_transport::Packet packet;
  packet.data.resize(chunk_length);
  int ret = muxer_->ReadChunk(chunk_length, &packet.data[0]);
  if (WebMLiveMuxer::kSuccess != ret) {
    ROS_ERROR("Failed to read chunk with error code: %d", ret);
    return;
  }
  packet.header.seq = ++package_sequence_;
  packet.header.stamp = ros::Time::now();
  publish_fn(packet);
}

void VPXPublisher::connectCallback(const ros::SingleSubscriberPublisher& pub) {
  if (muxer_) {
    delete muxer_;
  }
  muxer_ = new webm_tools::WebMLiveMuxer();

  yami_encoder_->connect();
  software_encoder_->connect();
}

void VPXPublisher::disconnectCallback(const ros::SingleSubscriberPublisher &pub) {
  int ret = muxer_->Finalize();
  if (ret != WebMLiveMuxer::kSuccess) {
    ROS_ERROR("Failed to finalize live muxer with error code: %d", ret);
    return;
  }
  int chunk_length = 0;
  if (!muxer_->ChunkReady(&chunk_length)) {
    ROS_ERROR("Failed to get chunk after finalized called.");
  }

  yami_encoder_->disconnect();
  software_encoder_->disconnect();
}

void VPXPublisher::onWriteFrame(uint8_t* buffer, uint64_t size,
                                uint64_t timeStamp, bool isKeyFrame) {
  muxer_->WriteVideoFrame(buffer, size, timeStamp, isKeyFrame);
}

} // vpx_image_transport
