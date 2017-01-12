// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ENCODER_H
#define ENCODER_H

#include <opencv2/imgproc/imgproc.hpp>

namespace vpx_image_transport {

class EncoderDelegate {
public:
  virtual void onWriteFrame(uint8_t* buffer, uint64_t size,
                            uint64_t timeStamp, bool isKeyFrame) = 0;
};

class Encoder {
public:
  virtual ~Encoder() = 0;

  virtual void encode(const cv::Mat& mat) = 0;
  virtual bool createEncoder(int frameWidth, int frameHeight) = 0;
  virtual void connect() {};
  virtual void disconnect() {};

protected:
  Encoder(EncoderDelegate* delegate) : delegate_(delegate) {}

  EncoderDelegate *delegate_;
};

} // namespace vpx_image_transport
#endif // ENCODER_H
