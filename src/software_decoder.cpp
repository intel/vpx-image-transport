// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "software_decoder.h"

#include <memory>
#include <ros/ros.h>
#include <vpx/vp8dx.h>

namespace vpx_image_transport {

SoftwareDecoder::SoftwareDecoder(DecoderDelegate* delegate)
  : Decoder(delegate), codec_context_(NULL), decoder_config_(NULL)  {
}

SoftwareDecoder::~SoftwareDecoder() {
  if (VPX_CODEC_OK != vpx_codec_destroy(codec_context_)) {
    ROS_ERROR("Failed to destroy VPX decoder context.");
  }
  delete codec_context_;
  delete decoder_config_;
}

bool SoftwareDecoder::createDecoder(int frameWidth, int frameHeight) {
  codec_context_ = new vpx_codec_ctx_t();
  decoder_config_ = new vpx_codec_dec_cfg();
  decoder_config_->threads = 1;
  decoder_config_->w = static_cast<unsigned int>(frameWidth);
  decoder_config_->h = static_cast<unsigned int>(frameHeight);

  long long ret = vpx_codec_dec_init(codec_context_, vpx_codec_vp8_dx(), decoder_config_, 0);
  if (ret) {
    ROS_ERROR("Failed to initialize VPX context. Error No.:%lld", ret);
    return false;
  }
  return true;
}

void SoftwareDecoder::decode(uint8_t* buffer, uint64_t size) {
  assert(codec_context_);
  const vpx_codec_err_t err = vpx_codec_decode(codec_context_, buffer, size, NULL, 0);
  if (err != VPX_CODEC_OK) {
    ROS_ERROR("Failed to decode frame, VPX error code:%d", err);
    return;
  }
  vpx_codec_iter_t iter = NULL;
  vpx_image_t* image = NULL;
  while ((image = vpx_codec_get_frame(codec_context_, &iter)) != NULL) {
    assert(VPX_IMG_FMT_I420 == image->fmt);

    cv::Mat i420;
    i420.create(cv::Size(image->d_w, image->d_h * 3 / 2), CV_8U);
    unsigned char* dest = i420.data;
    for (int plane = 0; plane < 3; ++plane) {
      const unsigned char *buf = image->planes[plane];
      const int stride = image->stride[plane];
      const int w = (plane ? (image->d_w + 1) >> 1 : image->d_w)
                     * ((image->fmt & VPX_IMG_FMT_HIGHBITDEPTH) ? 2 : 1);
      const int h = plane ? (image->d_h + 1) >> 1 : image->d_h;

      for (int y = 0; y < h; ++y) {
        memcpy(dest, buf, w);
        buf += stride;
        dest += w;
      }
    }
    vpx_img_free(image);

    cv::Mat bgr;
    cv::cvtColor(i420, bgr, CV_YUV2BGR_I420);

    delegate_->onFrameDecoded(bgr);
  }
}

bool SoftwareDecoder::initialized() {
  return codec_context_ != NULL;
}

} // namespace vpx_image_transport
