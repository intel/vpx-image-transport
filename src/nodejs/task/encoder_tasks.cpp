// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be
// found in the LICENSE file.

#include "task/encoder_tasks.h"
#include "gen/array_helper.h"
#include "utils.h"
#include "vpx_encoder_delegate.h"

void ConfigTask::WorkerThreadExecute() {
  ConfigTaskPayload* payload = GetPayload();
  payload->delegate_->ConfigStreamer(payload->dic_options_);
  task_state = AsyncTask::Successful;
}

ConfigTaskPayload* ConfigTask::GetPayload() {
  return reinterpret_cast<ConfigTaskPayload*>(AsyncTask::GetPayload());
}

v8_value_t ConfigTask::GetRejected() {
  return EncoderUtils::CreateRejectedPromise(reject_reason_);
}

EncodeTaskPayload* EncodeTask::GetPayload() {
  return reinterpret_cast<EncodeTaskPayload*>(AsyncTask::GetPayload());
}

void EncodeTask::WorkerThreadExecute() {
  EncodeTaskPayload* payload = GetPayload();
  if (payload->delegate_->EncodeImage(payload->raw_data_,
                                      payload->width_,
                                      payload->height_,
                                      &reject_reason_)) {
    task_state = AsyncTask::Successful;
  }  else {
    task_state = AsyncTask::Failed;
  }
}

bool EncodeTask::ShouldPopEvent(size_t event_index) {
  EncodeTaskPayload* payload = GetPayload();  
  return payload->delegate_->ShouldPopEvent(event_index);
}

v8_value_t EncodeTask::PopEventData(size_t event_index) {
  EncodeTaskPayload* payload = GetPayload();
  return payload->delegate_->PopEventData(event_index);
}

std::string EncodeTask::GetEventName(size_t event_index) const {
  EncodeTaskPayload* payload = const_cast<EncodeTask*>(this)->GetPayload();
  return payload->delegate_->GetEventName(event_index);
}

size_t EncodeTask::GetEventCount() {
  EncodeTaskPayload* payload = GetPayload();
  return payload->delegate_->GetEventCount();
}

v8_value_t EncodeTask::GetRejected() {
  return EncoderUtils::CreateRejectedPromise(reject_reason_);
}
