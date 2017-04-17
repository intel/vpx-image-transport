// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_encoder.h"

VPXEncoder::VPXEncoder() {
  VPXEncoderDelegate::GetInstance();
}

VPXEncoder::VPXEncoder(const VPXEncoder& rhs) {
  // TODO(widl-nan): copy from rhs if you want this behavior
  // Or mark ctor = delete in vpx_encoder.h
}

VPXEncoder::~VPXEncoder() {
  // TODO(widl-nan): copy from rhs if you want this behavior
  // Or mark ctor = delete in vpx_encoder.h  
}

VPXEncoder& VPXEncoder::operator = (const VPXEncoder& rhs) {
  if (&rhs != this) {
    // TODO(widl-nan): copy members from rhs
  }
  return *this;
}
