// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "software_encoder.h"

#include <vpx/vp8cx.h>
#include "stream_logger.h"

namespace vpx_streamer {

SoftwareEncoder::SoftwareEncoder(EncoderDelegate* delegate)
    : Encoder(delegate), frame_count_(0), keyframe_forced_interval_(4), quality_(10) {
}

SoftwareEncoder::~SoftwareEncoder() {
  if (codec_context_) {
    if (VPX_CODEC_OK != vpx_codec_destroy(codec_context_.get())) {
      STREAM_LOG_ERROR("Failed to destroy VPX encoder context.");
    }
  }
}

void SoftwareEncoder::encode(const cv::Mat& mat, bool isBgr) {
  assert(codec_context_);

  // Convert image to i420 color space used by vpx
  cv::Mat i420;
  if (isBgr)
    cv::cvtColor(mat, i420, cv::COLOR_BGR2YUV_I420);
  else
    cv::cvtColor(mat, i420, cv::COLOR_RGB2YUV_I420);

  vpx_image_t image;

  if (!vpx_img_wrap(&image, VPX_IMG_FMT_I420, encoder_config_->g_w,
                    encoder_config_->g_h, 1, i420.data)) {
    STREAM_LOG_ERROR("Failed to wrap cv::Mat into vpx image.");
    return;
  }

  int flags = 0;
  if (frame_count_ % keyframe_forced_interval_ == 0)
    flags |= VPX_EFLAG_FORCE_KF;

  vpx_codec_iter_t iter = NULL;
  const vpx_codec_cx_pkt_t *pkt = NULL;

  const vpx_codec_err_t ret = vpx_codec_encode(codec_context_.get(), &image,
     0, 1, flags, VPX_DL_REALTIME);
  if (ret != VPX_CODEC_OK) {
    return;
  }

  while ((pkt = vpx_codec_get_cx_data(codec_context_.get(), &iter)) != NULL) {
    if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
      bool keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;
      std::chrono::high_resolution_clock::time_point time = std::chrono::high_resolution_clock::now();
      auto elapsed = time - start_time_;
      uint64_t passed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
      frame_count_++;
      delegate_->onWriteFrame(reinterpret_cast<uint8_t*>(pkt->data.frame.buf), pkt->data.frame.sz, passed_time, keyframe);
    } else {
      STREAM_LOG_INFO("pkt->kind: %d", pkt->kind);
    }
  }
}

bool SoftwareEncoder::initialize(int frameWidth, int frameHeight) {
  assert(!codec_context_);

  vpx_codec_err_t ret;
  encoder_config_->g_w = frameWidth;
  encoder_config_->g_h = frameHeight;
  codec_context_.reset(new vpx_codec_ctx_t());
  ret = vpx_codec_enc_init(codec_context_.get(), vpx_codec_vp8_cx(), encoder_config_.get(), 0);
  if (ret) {
    STREAM_LOG_ERROR("Failed to initialize VPX encoder. Error No.:%d", ret);
    return false;
  }
  ret = vpx_codec_control_(codec_context_.get(), VP8E_SET_CQ_LEVEL, quality_);
  return true;
}

bool SoftwareEncoder::initialized() {
  return codec_context_ != NULL;
}

void SoftwareEncoder::configure(const EncoderConfig& config) {
  vpx_codec_err_t ret;
  quality_ = FigureCQLevel(config.quality);
  if (!encoder_config_) {
    encoder_config_.reset(new vpx_codec_enc_cfg());
    ret = vpx_codec_enc_config_default(vpx_codec_vp8_cx(), encoder_config_.get(), 0);
    if (ret) {
      STREAM_LOG_ERROR("Failed to get default encoder configuration. Error No.: %d", ret);
    }
  }
  encoder_config_->rc_end_usage = VPX_Q;
  // encoder_config_->rc_target_bitrate = config.target_bitrate;
  encoder_config_->g_timebase.num = 1;
  encoder_config_->g_timebase.den = config.target_framerate;
  encoder_config_->rc_min_quantizer = quality_;
  encoder_config_->rc_max_quantizer = quality_;
  // Keyframe configurations
  keyframe_forced_interval_ = config.keyframe_forced_interval;

  if (codec_context_) {
    ret = vpx_codec_enc_config_set(codec_context_.get(), encoder_config_.get());
    if (ret) {
      STREAM_LOG_ERROR("Failed to update codec configuration. Error No.:%d", ret);
    }
    ret = vpx_codec_control_(codec_context_.get(), VP8E_SET_CQ_LEVEL, quality_);
    if (ret) {
      STREAM_LOG_ERROR("Failed to set CQ_LEVEL. Error No.:%d", ret);
    }
  }
}

void SoftwareEncoder::connect() {
  frame_count_ = 0;
  start_time_ = std::chrono::high_resolution_clock::now();
}

int SoftwareEncoder::FigureCQLevel(int quality) {
  // For software encoder, the ineternal quality range is [0,63], and default is 10.
  // the input is from 1 to 100.
  int result = 10;
  if (quality < 1 || quality > 100)
    return result;
  result = 63-round((quality-1)*63/99);
  return result;
}

} // namespace vpx_streamer
