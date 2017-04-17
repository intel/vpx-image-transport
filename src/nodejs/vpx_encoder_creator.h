// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef _VPX_ENCODER_CREATOR_H_
#define _VPX_ENCODER_CREATOR_H_

#include "gen/encoder_options.h"
#include "gen/promise-helper.h"

class VPXEncoderCreator : public PromiseWorkQueueHelper {
 public:
  VPXEncoderCreator();
  ~VPXEncoderCreator() {}
  virtual bool DoWork();
  virtual void OnWorkDone();
  virtual v8::Local<v8::Value> GetResolved();
  virtual v8::Local<v8::Value> GetRejected();
  static void DestroyInstance();
  void SetEncoderOptions(
       const DictionaryEncoderOptions& dic_encoder_options);
 private:
  DictionaryEncoderOptions dic_encoder_options_;
  bool have_encoder_options_;
  static Nan::Persistent<v8::Object>* encoder_object_;
  std::string fail_reason_;
};

NAN_METHOD(CreateEncoder);

#endif  // _VPX_ENCODER_CREATOR_H_
