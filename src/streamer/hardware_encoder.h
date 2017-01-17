// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef YAMI_ENCODER_H
#define YAMI_ENCODER_H

#include <cv_bridge/cv_bridge.h>
#include <va/va.h>
#include <VideoEncoderInterface.h>
#include "encoder.h"

namespace vpx_streamer {

class HardwareEncoder : public Encoder {
public:
  HardwareEncoder(EncoderDelegate* delegate, NativeDisplay* display);
  virtual ~HardwareEncoder();

  virtual bool initialize(int frameWidth, int frameHeight);
  virtual bool initialized();
  virtual void encode(const cv::Mat& mat);
  virtual void connect();
  virtual void disconnect();

private:
  void fillVideoFrame(VideoFrameRawData* frame, const cv::Mat& mat,
                      int frame_width, int frame_height);

private:
  YamiMediaCodec::IVideoEncoder* encoder_;
  SharedPtr<NativeDisplay> native_display_;
  uint32_t max_output_buf_size_;
  uint32_t keyframe_forced_interval_;
  uint64_t frame_count_;
};

} // namespace vpx_streamer

#endif // YAMI_ENCODER_H
