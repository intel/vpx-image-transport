// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef DECODER_H
#define DECODER_H

#include <opencv2/imgproc/imgproc.hpp>

namespace vpx_streamer {

class DecoderDelegate {
public:
  virtual void onFrameDecoded(const cv::Mat& bgr) = 0;
};

struct DecoderConfig {
};

class Decoder {
public:
  virtual ~Decoder() = 0;

  virtual bool initialize(int frameWidth, int frameHeight) = 0;
  virtual bool initialized() = 0;
  virtual void decode(uint8_t* buffer, uint64_t size) = 0;
  virtual void configure(const DecoderConfig& config) {};

protected:
  Decoder(DecoderDelegate* delegate) : delegate_(delegate) {};

  DecoderDelegate* delegate_;
};

} // namespace vpx_streamer

#endif // DECODER_H
