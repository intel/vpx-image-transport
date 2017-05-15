// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_encoder_delegate.h"

#include "task/encoder_tasks.h"
#include "utils.h"

const std::string VPXEncoderDelegate::kEventName = "video_data";
VPXEncoderDelegate* VPXEncoderDelegate::self_ = nullptr;

VPXEncoderDelegate::VPXEncoderDelegate() :
    runner_(new AsyncTaskRunner()),
    stream_muxer_(this, false),
    connected_(false) {}

VPXEncoderDelegate::~VPXEncoderDelegate() {
  if (runner_)
    delete runner_;
  js_this_.Reset();
  DisconnectStreamMuxer();
}

v8::Handle<v8::Promise> VPXEncoderDelegate::Config(
    const EncoderOptions& options) {
  std::string error;
  if(!options.CheckType(&error)) {
    return EncoderUtils::CreateRejectedPromise(error);
  }
  ConfigTaskPayload* payload = new ConfigTaskPayload(options, this);
  ConfigTask* task = new ConfigTask();
  return runner_->PostPromiseTask(task, payload, "ConfigTask");
}

v8::Handle<v8::Promise> VPXEncoderDelegate::Encode(
    const ArrayBuffer& raw_data, const int32_t& width, const int32_t& height) {
  EncodeTaskPayload* payload = new EncodeTaskPayload(
      raw_data, width, height, this);
  EncodeTask* task = new EncodeTask();
  return runner_->PostEventEmitterTask(
      task, payload, Nan::New(js_this_), "EncodeTask");
}

void VPXEncoderDelegate::onChunkReadyRawData(uint8_t* buffer, int32_t length) {
  std::lock_guard<std::mutex> guard(data_lock_);
  encoded_data_segs_.push_back(std::pair<uint8_t*, uint32_t>(buffer, length));
}

void VPXEncoderDelegate::ConfigStreamer(
    const DictionaryEncoderOptions& options) {
  vpx_streamer::EncoderConfig config;
  config.target_bitrate = options.member_bitRate;
  config.keyframe_forced_interval = options.member_keyFrameForcedInterval;
  config.target_framerate = options.member_frameRate;
  DEBUG_INFO("ConfigStreamer, bitrate/interval/fps:", config.target_bitrate, '/', config.keyframe_forced_interval, '/', config.target_framerate);
  stream_muxer_.configure(config);
}

bool VPXEncoderDelegate::EncodeImage(const ArrayBuffer& raw_data,
                                     const int32_t width,
                                     const int32_t height,
                                     std::string* error) {
  // check to connect streamMuxer
  ConnectStreamMuxer();
  cv::Mat input(height, width, CV_8UC3, raw_data.data);
  stream_muxer_.encodeImage(input, width, height, false);
  return true;
}

bool VPXEncoderDelegate::ShouldPopEvent(size_t event_index) {
  std::lock_guard<std::mutex> guard(data_lock_);
  return !encoded_data_segs_.empty();
}

v8_value_t VPXEncoderDelegate::PopEventData(size_t event_index) {
  std::lock_guard<std::mutex> guard(data_lock_);
  v8::Local<v8::Object> value;
  uint32_t len = 0;
  uint8_t* data = MergeEncodedDataSegs(&len);
  if (data) {
    Nan::NewBuffer((char*)data, len).ToLocal(&value);
    return value;
  } else
    return Nan::Undefined();
}
VPXEncoderDelegate* VPXEncoderDelegate::GetInstance() {
  if (!self_)
    self_ = new VPXEncoderDelegate();
  return self_;
}

void VPXEncoderDelegate::ConnectStreamMuxer() {
  if (!connected_) {
    connected_ = true;    
    stream_muxer_.connect();
  }
}

void VPXEncoderDelegate::DisconnectStreamMuxer() {
  if (connected_) {
    stream_muxer_.disconnect();
    connected_ = false;
  }
}

uint8_t* VPXEncoderDelegate::MergeEncodedDataSegs(uint32_t* total_length) {
  uint32_t total_len = 0;
  for (auto& seg: encoded_data_segs_) {
    total_len += seg.second;
  }
  if (!total_len)
    return nullptr;
  uint8_t* buf = (uint8_t*)malloc(total_len);
  if (!buf)
    return nullptr;
  uint32_t index = 0;
  for (auto& seg: encoded_data_segs_) {
    memcpy(buf+index, seg.first, seg.second);
    index += seg.second;
    free(seg.first);
  }
  encoded_data_segs_.clear();
  *total_length = total_len;
  return buf;
}