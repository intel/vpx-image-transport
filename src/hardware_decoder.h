// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef HARDWARE_DECODER_H
#define HARDWARE_DECODER_H

#include <va/va.h>
#include <VideoDecoderInterface.h>
#include "decoder.h"

namespace vpx_image_transport {

class HardwareDecoder : public Decoder {
public:
  HardwareDecoder(DecoderDelegate* delegate, VADisplay vaDisplay, NativeDisplay* nativeDisplay);
  virtual ~HardwareDecoder();

  virtual bool createDecoder(int frameWidth, int frameHeight);
  virtual void decode(uint8_t* buffer, uint64_t size);
  virtual bool initialized();

private:
  YamiMediaCodec::IVideoDecoder* yami_decoder_;
  VADisplay va_display_;
  SharedPtr<NativeDisplay> native_display_;
};

} // namespace vpx_image_transport

#endif // HARDWARE_DECODER_H
