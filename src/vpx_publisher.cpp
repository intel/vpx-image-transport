// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_publisher.h"

#include <cv_bridge/cv_bridge.h>
#include <utility>

namespace vpx_image_transport {

using namespace vpx_streamer;

VPXPublisher::VPXPublisher()
  : package_sequence_(0), stream_muxer_(this), publish_function_(NULL) {
}

VPXPublisher::~VPXPublisher() {
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

  conf.keyframe_forced_interval = config.keyframe_forced_interval;
  conf.target_framerate = config.target_framerate;
  conf.quality = config.quality;

  stream_muxer_.configure(conf);
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

  publish_function_ = &publish_fn;
  stream_muxer_.encodeImage(bgr, frame_width, frame_height);
  publish_function_ = NULL;
}

void VPXPublisher::onChunkReady(std::vector<uint8_t>& buffer) {
  if (!publish_function_) {
    return;
  }

  vpx_image_transport::Packet packet;
  packet.data = boost::move(buffer);
  packet.header.seq = ++package_sequence_;
  packet.header.stamp = ros::Time::now();
  (*publish_function_)(packet);
}

void VPXPublisher::connectCallback(const ros::SingleSubscriberPublisher& pub) {
  stream_muxer_.connect();
}

void VPXPublisher::disconnectCallback(const ros::SingleSubscriberPublisher &pub) {
  stream_muxer_.disconnect();
}

} // vpx_image_transport
