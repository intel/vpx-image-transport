// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef CODEC_FACTORY_H
#define CODEC_FACTORY_H

#include <va/va.h>
#include "decoder.h"
#include "encoder.h"

namespace vpx_image_transport {

class CodecFactory {
public:
  enum CreationMethod {
    DEFAULT = 0,
    SOFTWARE_ONLY
  };
  CodecFactory();
  ~CodecFactory();
  Encoder* createEncoder(EncoderDelegate* delegate, CreationMethod method=DEFAULT);
  Decoder* createDecoder(DecoderDelegate* delegate, CreationMethod method=DEFAULT);

private:
  bool initDisplay();
  bool isHardwareAcceleratedEncoderSupported();
  bool isHardwareAcceleratedDecoderSupported();

  VADisplay va_display_;
};

} // namespace vpx_image_transport

#endif // CODEC_FACTORY_H
