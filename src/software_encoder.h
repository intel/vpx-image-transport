// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SOFTWARE_ENCODER_H
#define SOFTWARE_ENCODER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <vpx/vpx_encoder.h>
#include "encoder.h"

namespace vpx_image_transport {

class SoftwareEncoder : public Encoder {
public:
  SoftwareEncoder(EncoderDelegate* delegate);
  virtual ~SoftwareEncoder();

  virtual bool createEncoder(int frameWidth, int frameHeight);
  virtual void encode(const cv::Mat& mat);
  virtual void configure(const EncoderConfig& config);
  virtual void connect();

  bool initialized();

private:
  vpx_codec_ctx_t* codec_context_;
  vpx_codec_enc_cfg_t* encoder_config_;
  uint64_t frame_count_;
  unsigned int keyframe_forced_interval_;
};

} // namespace vpx_image_transport

#endif // SOFTWARE_ENCODER_H
