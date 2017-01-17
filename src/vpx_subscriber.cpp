// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_subscriber.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace vpx_image_transport {

using namespace vpx_streamer;

VPXSubscriber::VPXSubscriber()
  : stream_parser_(this), image_index_(0), user_callback_(NULL) {
}

VPXSubscriber::~VPXSubscriber() {
}

void VPXSubscriber::internalCallback(
    const typename vpx_image_transport::Packet::ConstPtr& message,
    const Callback& user_cb) {

  user_callback_ = &user_cb;
  stream_parser_.decodeStream(message->data);
  user_callback_ = NULL;
}

void VPXSubscriber::onImageDecoded(const cv::Mat& bgr) {
  if (!user_callback_) {
    return;
  }

  std_msgs::Header header;
  header.seq = ++image_index_;
  header.stamp = ros::Time::now();
  cv_bridge::CvImage cv_img(header, "bgr8", bgr);
  (*user_callback_)(cv_img.toImageMsg());
}

} // namespace vpx_image_transport
