// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef ENCODER_FACTORY_H
#define ENCODER_FACTORY_H

#include <va/va.h>
#include "encoder.h"

namespace vpx_image_transport {

class EncoderFactory {
public:
  enum CreationMethod {
    DEFAULT = 0,
    SOFTWARE_ONLY
  };
  EncoderFactory();
  ~EncoderFactory();
  Encoder* createEncoder(EncoderDelegate* delegate, CreationMethod method=DEFAULT);

private:
  bool initDisplay();
  bool isHardwareAccelerationSupported();

  VADisplay va_display_;
};

} // namespace vpx_image_transport

#endif // ENCODER_FACTORY_H
