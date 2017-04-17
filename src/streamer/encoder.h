// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ENCODER_H
#define ENCODER_H

#include <opencv2/imgproc/imgproc.hpp>

namespace vpx_streamer {

class EncoderDelegate {
public:
  virtual void onWriteFrame(uint8_t* buffer, uint64_t size,
                            uint64_t timeStamp, bool isKeyFrame) = 0;
};

struct EncoderConfig {
  int target_bitrate;
  int keyframe_forced_interval;
};

class Encoder {
public:
  virtual ~Encoder() {};

  virtual bool initialize(int frameWidth, int frameHeight) = 0;
  virtual bool initialized() = 0;
  virtual void encode(const cv::Mat& mat, bool isBgr) = 0;
  virtual void configure(const EncoderConfig& config) {};
  virtual void connect() {};
  virtual void disconnect() {};

protected:
  Encoder(EncoderDelegate* delegate) : delegate_(delegate) {}

  EncoderDelegate *delegate_;
};

} // namespace vpx_streamer
#endif // ENCODER_H
