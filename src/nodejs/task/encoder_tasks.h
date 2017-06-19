// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be
// found in the LICENSE file.

#ifndef ENCODER_TASKS_H_
#define ENCODER_TASKS_H_

#include <v8.h>

#include <memory>
#include <string>
#include <vector>

#include "gen/promise-helper.h"
#include "task/async_task.h"
#include "vpx_encoder_delegate.h"

class VPXPayload : public AsyncTaskPayload {
 public:
  explicit VPXPayload(VPXEncoderDelegate* delegate) : delegate_(delegate) {};
  virtual ~VPXPayload() {};
  VPXEncoderDelegate* delegate_;
};

class ConfigTaskPayload : public VPXPayload {
 public:
  ConfigTaskPayload(const EncoderOptions& options,
                    VPXEncoderDelegate* delegate) :
      VPXPayload(delegate),
      dic_options_(options) {}
  virtual ~ConfigTaskPayload() {}
  DictionaryEncoderOptions dic_options_;
};

class ConfigTask : public PromiseTask {
 public:
  ConfigTask() : reject_reason_("Failed") {}
  virtual ~ConfigTask() {}
  virtual void WorkerThreadExecute();
  virtual ConfigTaskPayload* GetPayload();
  virtual v8_value_t GetRejected();

 protected:
  // The reject_reason_ shall be set to meaningful value.
  std::string reject_reason_;
};

class EncodeTaskPayload : public VPXPayload {
 public:
  EncodeTaskPayload(const ArrayBuffer& raw_data,
                    const int32_t& width,
                    const int32_t& height,
                    VPXEncoderDelegate* delegate) :
      VPXPayload(delegate),
      width_(width),
      height_(height),
      result_data_(nullptr),
      result_data_len_(0) {
        if (raw_data.size) {
          raw_data_.data = static_cast<char*>(malloc(raw_data.size));
          memcpy(raw_data_.data, raw_data.data, raw_data.size);
          raw_data_.size = raw_data.size;
        }
      }
  virtual ~EncodeTaskPayload() {
    if (raw_data_.data)
      free(raw_data_.data);
  }
  ArrayBuffer raw_data_;
  int32_t width_;
  int32_t height_;
  uint8_t* result_data_;
  uint32_t result_data_len_;
};

class EncodeTask : public EventEmitterTask {
 public:
  EncodeTask() : reject_reason_("Failed") {}
  virtual ~EncodeTask() {}
  virtual EncodeTaskPayload* GetPayload();
  virtual void WorkerThreadExecute();
  virtual bool ShouldPopEvent(size_t event_index);
  virtual v8_value_t PopEventData(size_t event_index);
  virtual std::string GetEventName(size_t event_index) const;
  virtual size_t GetEventCount();
  v8_value_t GetRejected();

 protected:
   std::string reject_reason_;
};

#endif  // ENCODER_TASKS_H_
