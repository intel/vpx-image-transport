// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "hardware_encoder.h"

#include <va/va_x11.h>
#include <VideoEncoderHost.h>
#include "stream_logger.h"

namespace vpx_streamer {

HardwareEncoder::HardwareEncoder(EncoderDelegate* delegate, NativeDisplay* display)
  : Encoder(delegate), native_display_(display), max_output_buf_size_(0),
    keyframe_forced_interval_(4), frame_count_(0) {
}

HardwareEncoder::~HardwareEncoder() {
  releaseVideoEncoder(encoder_.get());
}

void HardwareEncoder::fillVideoFrame(VideoFrameRawData* frame,
                                     const cv::Mat& mat,
                                     int frame_width,
                                     int frame_height,
                                     int64_t time_stamp) {
  assert(frame);

  // working solution for yami.
  frame->fourcc = VA_FOURCC('I', '4', '2', '0');
  frame->width = frame_width;
  frame->height = frame_height;
  frame->handle = reinterpret_cast<intptr_t>(mat.data);
  frame->size = mat.total();
  frame->memoryType = VIDEO_DATA_MEMORY_TYPE_RAW_POINTER;
  frame->timeStamp = time_stamp;

  uint32_t offset = 0;
  for (int i = 0; i < 3; ++i) {
    int w = i ? (frame_width + 1) >> 1 : frame_width;
    int h = i ? (frame_height + 1) >> 1 : frame_height;
    frame->pitch[i] = w;
    frame->offset[i] = offset;
    offset += w * h;
  }
}

void HardwareEncoder::encode(const cv::Mat& mat, bool isBgr) {
  assert(encoder_);

  cv::Mat input;
  if (isBgr)
    cv::cvtColor(mat, input, cv::COLOR_BGR2YUV_I420);
  else
    cv::cvtColor(mat, input, cv::COLOR_RGB2YUV_I420);
  std::chrono::high_resolution_clock::time_point time =
      std::chrono::high_resolution_clock::now();
  auto elapsed = time - start_time_;
  int64_t passed_time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();

  VideoFrameRawData input_buffer;
  memset(&input_buffer, 0, sizeof(input_buffer));
  fillVideoFrame(
      &input_buffer, input, mat.size().width, mat.size().height, passed_time);

  YamiStatus status = encoder_->encode(&input_buffer);
  if (status != ENCODE_SUCCESS) {
    STREAM_LOG_WARN("Failed to encode input buffer.");
    return;
  }

  VideoEncOutputBuffer output_buffer;
  output_buffer.data = static_cast<uint8_t*>(malloc(max_output_buf_size_));
  output_buffer.bufferSize = max_output_buf_size_;
  output_buffer.format = OUTPUT_EVERYTHING;

  do {
    status = encoder_->getOutput(&output_buffer);
    if (status == ENCODE_SUCCESS) {
      bool keyframe = (4 & output_buffer.flag) != 0;
      delegate_->onWriteFrame(output_buffer.data, output_buffer.dataSize,
                              output_buffer.timeStamp, keyframe);
    }
  } while (status != ENCODE_BUFFER_NO_MORE);
  free(output_buffer.data);
}

bool HardwareEncoder::initialize(int frameWidth, int frameHeight) {
  assert(encoder_ == NULL);

  encoder_.reset(createVideoEncoder(YAMI_MIME_VP8));
  if (encoder_ == NULL) {
    STREAM_LOG_ERROR("Failed to create VP8 yami encoder.");
    return false;
  }

  encoder_->setNativeDisplay(native_display_.get());
  VideoParamsCommon params;
  params.size = sizeof(VideoParamsCommon);
  YamiStatus s = encoder_->getParameters(VideoParamsTypeCommon, &params);
  if (s != YAMI_SUCCESS) {
    STREAM_LOG_ERROR("Failed to get parameters before set.");
    return false;
  }
  params.resolution.width = frameWidth;
  params.resolution.height = frameHeight;
  params.intraPeriod = keyframe_forced_interval_;
  params.rcMode = RATE_CONTROL_CQP;
  params.rcParams.bitRate = 1024*1024*8;
  s = encoder_->setParameters(VideoParamsTypeCommon, &params);
  if (s != YAMI_SUCCESS) {
    STREAM_LOG_ERROR("Failed to set parameters for yami encoder, status code:%d", s);
    return false;
  }

  s = encoder_->start();
  if (s != YAMI_SUCCESS) {
    STREAM_LOG_ERROR("Failed to start yami encoder, status code:%d", s);
    return false;
  }
  encoder_->getMaxOutSize(&max_output_buf_size_);
  return true;
}

bool HardwareEncoder::initialized() {
  return encoder_ != NULL;
}
void HardwareEncoder::connect() {
  if (encoder_) {
    YamiStatus s = encoder_->start();
    if (s != YAMI_SUCCESS) {
      STREAM_LOG_ERROR("Failed to start yami encoder when connect, status code:%d", s);
      return;
    }
  }
  frame_count_ = 0;
  start_time_ = std::chrono::high_resolution_clock::now();
}

void HardwareEncoder::disconnect() {
  if (encoder_) {
    encoder_->flush();
    YamiStatus s = encoder_->stop();
    if (s != YAMI_SUCCESS) {
      STREAM_LOG_ERROR("Failed to start yami encoder when disconnect, status code:%d", s);
      return;
    }
  }
}

} // namespace vpx_streamer
