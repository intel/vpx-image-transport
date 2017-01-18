// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "software_encoder.h"

#include <vpx/vp8cx.h>
#include "stream_logger.h"

namespace vpx_streamer {

SoftwareEncoder::SoftwareEncoder(EncoderDelegate* delegate)
  : Encoder(delegate), codec_context_(NULL), encoder_config_(NULL), frame_count_(0),
    keyframe_forced_interval_(4) {
}

SoftwareEncoder::~SoftwareEncoder() {
  if (codec_context_) {
    if (VPX_CODEC_OK != vpx_codec_destroy(codec_context_)) {
      STREAM_LOG_ERROR("Failed to destroy VPX encoder context.");
    }
    delete codec_context_;
  }

  if (encoder_config_) {
    delete encoder_config_;
  }
}

void SoftwareEncoder::encode(const cv::Mat& mat) {
  assert(codec_context_);

  // Convert image to i420 color space used by vpx
  cv::Mat i420;
  cv::cvtColor(mat, i420, cv::COLOR_BGR2YUV_I420);

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

  const vpx_codec_err_t ret = vpx_codec_encode(codec_context_, &image,
     0, 1, flags, VPX_DL_REALTIME);
  if (ret != VPX_CODEC_OK) {
    return;
  }

  while ((pkt = vpx_codec_get_cx_data(codec_context_, &iter)) != NULL) {
    if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
      bool keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;
      delegate_->onWriteFrame(reinterpret_cast<uint8_t*>(pkt->data.frame.buf), pkt->data.frame.sz, ++frame_count_, keyframe);
    } else {
      STREAM_LOG_INFO("pkt->kind: %d", pkt->kind);
    }
  }
}

bool SoftwareEncoder::initialize(int frameWidth, int frameHeight) {
  assert(!codec_context_);

  vpx_codec_err_t ret;
  if (!encoder_config_) {
    encoder_config_ = new vpx_codec_enc_cfg();
    ret = vpx_codec_enc_config_default(vpx_codec_vp8_cx(), encoder_config_, 0);
    if (ret) {
      STREAM_LOG_ERROR("Failed to get default encoder configuration. Error No.: %d", ret);
      return false;
    }
  }

  encoder_config_->g_w = frameWidth;
  encoder_config_->g_h = frameHeight;

  codec_context_ = new vpx_codec_ctx_t();
  ret = vpx_codec_enc_init(codec_context_, vpx_codec_vp8_cx(), encoder_config_, 0);
  if (ret) {
    STREAM_LOG_ERROR("Failed to initialize VPX encoder. Error No.:%d", ret);
    return false;
  }
  return true;
}

bool SoftwareEncoder::initialized() {
  return codec_context_ != NULL;
}

void SoftwareEncoder::configure(const EncoderConfig& config) {
  vpx_codec_err_t ret;
  if (!encoder_config_) {
    encoder_config_ = new vpx_codec_enc_cfg();
    ret = vpx_codec_enc_config_default(vpx_codec_vp8_cx(), encoder_config_, 0);
    if (ret) {
      STREAM_LOG_ERROR("Failed to get default encoder configuration. Error No.: %d", ret);
    }
  }

  encoder_config_->rc_target_bitrate = config.target_bitrate;
  // Keyframe configurations
  keyframe_forced_interval_ = config.keyframe_forced_interval;

  if (codec_context_) {
    ret = vpx_codec_enc_config_set(codec_context_, encoder_config_);
    if (ret) {
      STREAM_LOG_ERROR("Failed to update codec configuration. Error No.:%d", ret);
    }
  }
}

void SoftwareEncoder::connect() {
  frame_count_ = 0;
}

} // namespace vpx_streamer
