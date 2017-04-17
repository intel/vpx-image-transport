// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef _VPX_ENCODER_H_
#define _VPX_ENCODER_H_

#include <node.h>
#include <v8.h>

#include <string>

#include "gen/encoder_options.h"
#include "gen/generator_helper.h"
#include "utils.h"
#include "vpx_encoder_delegate.h"

class VPXEncoder {
 public:
  VPXEncoder();

  VPXEncoder(const VPXEncoder& rhs);

  ~VPXEncoder();

  VPXEncoder& operator = (const VPXEncoder& rhs);

 public:
  v8::Handle<v8::Promise> config(const EncoderOptions& options) {
    return VPXEncoderDelegate::GetInstance()->Config(options);
  }

  v8::Handle<v8::Promise> encode(
      const ArrayBuffer& rawData, const int32_t& width, const int32_t& height) {
    return VPXEncoderDelegate::GetInstance()->Encode(rawData, width, height);
  }

  void SetJavaScriptThis(v8::Local<v8::Object> obj) {
    VPXEncoderDelegate::GetInstance()->SetJavaScriptThis(obj);
  }
};

#endif  // _VPX_ENCODER_H_
