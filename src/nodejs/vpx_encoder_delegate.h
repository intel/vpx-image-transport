// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef _VPX_ENCODER_DELEGATE_H_
#define _VPX_ENCODER_DELEGATE_H_

#include <node.h>
#include <v8.h>

#include <mutex>
#include <string>
#include <vector>

#include "gen/array_helper.h"
#include "gen/encoder_options.h"
#include "stream_muxer.h"
#include "task/async_task_runner.h"

class VPXEncoderDelegate : public vpx_streamer::StreamMuxerDelegate {
 public:
  ~VPXEncoderDelegate();

 public:
  // called by VPXEncoder
  v8::Handle<v8::Promise> Config(const EncoderOptions& options);
  v8::Handle<v8::Promise> Encode(
      const ArrayBuffer& raw_data, const int32_t& width, const int32_t& height);
  void SetJavaScriptThis(v8::Local<v8::Object> obj) {
    js_this_.Reset(obj);
  }

  // called in worker thread
  void ConfigStreamer(const DictionaryEncoderOptions& options);
  bool EncodeImage(const ArrayBuffer& raw_data,
                   const int32_t width,
                   const int32_t height,
                   std::string* error);

  // StreamMuxerDelegate implementation
  virtual void onChunkReady(std::vector<uint8_t>& buffer) {}
  virtual void onChunkReadyRawData(uint8_t* buffer, int32_t length);

  size_t GetEventCount() { return 1; };
  bool ShouldPopEvent(size_t event_index);
  std::string GetEventName(size_t event_index) const { return kEventName; };
  v8_value_t PopEventData(size_t event_index);

  static const std::string kEventName;
  static VPXEncoderDelegate* GetInstance();

 private:
  VPXEncoderDelegate();
  void ConnectStreamMuxer();
  void DisconnectStreamMuxer();
  // merge all of the data segments in encoded_data_segs_ into a single buffer.
  uint8_t* MergeEncodedDataSegs(uint32_t* total_length);  

  AsyncTaskRunner* runner_;
  Nan::Persistent<v8::Object> js_this_;
  std::vector<std::pair<uint8_t*, uint32_t>> encoded_data_segs_;
  vpx_streamer::StreamMuxer stream_muxer_;
  static VPXEncoderDelegate* self_;
  std::mutex data_lock_;
  bool connected_;
};

#endif  // _VPX_ENCODER_DELEGATE_H_
