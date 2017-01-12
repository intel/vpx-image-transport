// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_publisher.h"

#include <cv_bridge/cv_bridge.h>
#include <vpx/vp8cx.h>
#include "yami_encoder.h"

namespace vpx_image_transport {

  using namespace webm_tools;
  using namespace mkvmuxer;

VPXPublisher::VPXPublisher()
  : codec_context_(NULL), encoder_config_(NULL), frame_count_(0),
    package_sequence_(0), keyframe_forced_interval_(4), muxer_(NULL),
    yami_encoder_(new YamiEncoder(this)) {
}

VPXPublisher::~VPXPublisher() {
  muxer_->Finalize();
  delete muxer_;

  if (codec_context_) {
    if (VPX_CODEC_OK != vpx_codec_destroy(codec_context_)) {
      ROS_ERROR("Failed to destroy VPX encoder context.");
    }
    delete codec_context_;
  }

  if (encoder_config_) {
    delete encoder_config_;
  }
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
  vpx_codec_err_t ret;
  if (!encoder_config_) {
    encoder_config_ = new vpx_codec_enc_cfg();
    ret = vpx_codec_enc_config_default(vpx_codec_vp8_cx(), encoder_config_, 0);
    if (ret) {
      ROS_ERROR("Failed to get default encoder configuration. Error No.: %d", ret);
    }
  }

  encoder_config_->g_threads = config.threads;
  encoder_config_->rc_end_usage = static_cast<vpx_rc_mode>(config.end_usage);
  encoder_config_->rc_target_bitrate = config.target_bitrate;

  // Keyframe configurations
  keyframe_forced_interval_ = config.keyframe_forced_interval;
  encoder_config_->kf_mode = static_cast<vpx_kf_mode>(config.keyframe_mode);
  encoder_config_->kf_min_dist = config.keyframe_min_interval;
  encoder_config_->kf_max_dist = config.keyframe_max_interval;

  if (codec_context_) {
    ret = vpx_codec_enc_config_set(codec_context_, encoder_config_);
    if (ret) {
      ROS_ERROR("Failed to update codec configuration. Error No.:%d", ret);
    }
  }
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

  if (!yami_encoder_->initialized() && !codec_context_) {
    if (yami_encoder_->createEncoder(message.width, message.height)) {
      ROS_INFO("Hardware accelerated encoder enabled.");
    } else if (!createVPXEncoder(message.width, message.height)){
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
  } else if (codec_context_) {
    encodeWithVPX(bgr);
  }

  sendChunkIfReady(publish_fn);
}

bool VPXPublisher::createVPXEncoder(int frame_width, int frame_height) const {
  assert(!codec_context_);

  vpx_codec_err_t ret;
  if (!encoder_config_) {
    encoder_config_ = new vpx_codec_enc_cfg();
    ret = vpx_codec_enc_config_default(vpx_codec_vp8_cx(), encoder_config_, 0);
    if (ret) {
      ROS_ERROR("Failed to get default encoder configuration. Error No.: %d", ret);
      return false;
    }
  }

  encoder_config_->g_w = frame_width;
  encoder_config_->g_h = frame_height;

  codec_context_ = new vpx_codec_ctx_t();
  ret = vpx_codec_enc_init(codec_context_, vpx_codec_vp8_cx(), encoder_config_, 0);
  if (ret) {
    ROS_ERROR("Failed to initialize VPX encoder. Error No.:%d", ret);
    return false;
  }
  return true;
}

void VPXPublisher::encodeWithVPX(const cv::Mat& mat) const {
  assert(codec_context_);

  // Convert image to i420 color space used by vpx
  cv::Mat i420;
  cv::cvtColor(mat, i420, cv::COLOR_BGR2YUV_I420);

  vpx_image_t image;

  if (!vpx_img_wrap(&image, VPX_IMG_FMT_I420, encoder_config_->g_w,
                    encoder_config_->g_h, 1, i420.data)) {
    ROS_ERROR("Failed to wrap cv::Mat into vpx image.");
    return;
  }

  int flags = 0;
  if (frame_count_ % keyframe_forced_interval_ == 0)
    flags |= VPX_EFLAG_FORCE_KF;

  vpx_codec_iter_t iter = NULL;
  const vpx_codec_cx_pkt_t *pkt = NULL;

  const vpx_codec_err_t ret = vpx_codec_encode(codec_context_, &image,
     ros::Time::now().toNSec(), 1, flags, VPX_DL_REALTIME);
  if (ret != VPX_CODEC_OK) {
    return;
  }

  while ((pkt = vpx_codec_get_cx_data(codec_context_, &iter)) != NULL) {
    if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
      bool keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;
      muxer_->WriteVideoFrame(reinterpret_cast<uint8*>(pkt->data.frame.buf), pkt->data.frame.sz, ++frame_count_, keyframe);
    } else {
      ROS_INFO("pkt->kind: %d", pkt->kind);
    }
  }
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
  frame_count_ = 0;

  yami_encoder_->connect();
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
}

void VPXPublisher::onWriteFrame(uint8_t* buffer, uint64_t size,
                                uint64_t timeStamp, bool isKeyFrame) {
  muxer_->WriteVideoFrame(buffer, size, timeStamp, isKeyFrame);
}

} // vpx_image_transport
