// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gen/nan__vpx_encoder.h"
#include "utils.h"
#include "vpx_encoder.h"
#include "vpx_encoder_creator.h"

Nan::Persistent<v8::Object>* VPXEncoderCreator::encoder_object_ = nullptr;

static bool IsPlaceholder(v8::Local<v8::Value> value) {
  return value->IsUndefined() || value->IsNull();
}

NAN_METHOD(CreateEncoder) {
  DictionaryEncoderOptions dic_encoder_options;
  bool encoder_options_present = false;
  bool wrong_arguments = false;

  // Quite lame way to process arguments, but it works
  int32_t len = info.Length();
  for (int32_t i=0; i < len; i++) {
    if (!info[i]->IsObject() && !IsPlaceholder(info[i])) {
      wrong_arguments = true;
      break;
    }
  }

  if (wrong_arguments) {
    info.GetReturnValue().Set(EncoderUtils::CreateRejectedPromise(
        "Invalid CreateEncoder parameters"));
    return;
  }

  if (len >= 1 && info[0]->IsObject())
    encoder_options_present = true;

  // Check the option value type
  std::string error_string;
  if (encoder_options_present) {
    EncoderOptions encoder_options(info[0]->ToObject());
    dic_encoder_options.ImportFrom(encoder_options);

    if (!encoder_options.CheckType(&error_string)) {
      info.GetReturnValue().Set(
          EncoderUtils::CreateRejectedPromise(error_string));
      return;
    }
  }

  VPXEncoderCreator* creator = new VPXEncoderCreator();
  if (encoder_options_present)
    creator->SetEncoderOptions(dic_encoder_options);
  creator->CreatePromiseAndSetFunctionReturnValue(info);
}

VPXEncoderCreator::VPXEncoderCreator()
    : have_encoder_options_(false) {
}

void VPXEncoderCreator::SetEncoderOptions(
      const DictionaryEncoderOptions& dic_encoder_options) {
  dic_encoder_options_ = dic_encoder_options;
  have_encoder_options_ = true;
}

bool VPXEncoderCreator::DoWork() {
  bool success = true;
  return success;
}

void VPXEncoderCreator::OnWorkDone() {
  PromiseWorkQueueHelper::OnWorkDone();
  v8::Isolate::GetCurrent()->RunMicrotasks();
}

v8::Local<v8::Value> VPXEncoderCreator::GetResolved() {
  if(!encoder_object_) {
    encoder_object_ = new Nan::Persistent<v8::Object>();
    encoder_object_->Reset(NanVPXEncoder::NewInstance());
  }
  // if options is provided, set it, set options directly.
  if (have_encoder_options_) {
    VPXEncoderDelegate::GetInstance()->ConfigStreamer(dic_encoder_options_);
  }

  return Nan::New(*encoder_object_);
}

v8::Local<v8::Value> VPXEncoderCreator::GetRejected() {
  return Nan::New(fail_reason_).ToLocalChecked();
}

void VPXEncoderCreator::DestroyInstance() {
  if(encoder_object_) {
    encoder_object_->Reset();
  }
  delete encoder_object_;
  encoder_object_ = nullptr;
}
