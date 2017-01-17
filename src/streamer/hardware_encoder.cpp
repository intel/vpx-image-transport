// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "hardware_encoder.h"

#include <ros/ros.h>
#include <va/va_x11.h>
#include <VideoEncoderHost.h>

namespace vpx_streamer {

HardwareEncoder::HardwareEncoder(EncoderDelegate* delegate, NativeDisplay* display)
  : Encoder(delegate), encoder_(NULL), native_display_(display), max_output_buf_size_(0),
    keyframe_forced_interval_(4), frame_count_(0) {
}

HardwareEncoder::~HardwareEncoder() {
  if (encoder_) {
    releaseVideoEncoder(encoder_);
    delete encoder_;
  }
}

void HardwareEncoder::fillVideoFrame(VideoFrameRawData* frame, const cv::Mat& mat,
                                  int frame_width, int frame_height) {
  assert(frame);

  // working solution for yami.
  frame->fourcc = VA_FOURCC('I', '4', '2', '0');
  frame->width = frame_width;
  frame->height = frame_height;
  frame->handle = reinterpret_cast<intptr_t>(mat.data);
  frame->size = mat.total();
  frame->memoryType = VIDEO_DATA_MEMORY_TYPE_RAW_POINTER;
  frame->timeStamp = ++frame_count_;

  uint32_t offset = 0;
  for (int i = 0; i < 3; ++i) {
    int w = i ? (frame_width + 1) >> 1 : frame_width;
    int h = i ? (frame_height + 1) >> 1 : frame_height;
    frame->pitch[i] = w;
    frame->offset[i] = offset;
    offset += w * h;
  }
}

void HardwareEncoder::encode(const cv::Mat& mat) {
  assert(encoder_);

  cv::Mat input;
  cv::cvtColor(mat, input, cv::COLOR_BGR2YUV_I420);

  VideoFrameRawData input_buffer;
  memset(&input_buffer, 0, sizeof(input_buffer));
  fillVideoFrame(&input_buffer, input, mat.size().width, mat.size().height);

  YamiStatus status = encoder_->encode(&input_buffer);
  if (status != ENCODE_SUCCESS) {
    ROS_WARN("Failed to encode input buffer.");
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
  assert(!encoder_);

  YamiMediaCodec::IVideoEncoder* encoder = createVideoEncoder(YAMI_MIME_VP8);
  if (!encoder) {
    ROS_ERROR("Failed to create VP8 yami encoder.");
    return false;
  }

  encoder->setNativeDisplay(native_display_.get());
  VideoParamsCommon params;
  params.size = sizeof(VideoParamsCommon);
  YamiStatus s = encoder->getParameters(VideoParamsTypeCommon, &params);
  if (s != YAMI_SUCCESS) {
    ROS_ERROR("Failed to get parameters before set.");
    return false;
  }
  params.resolution.width = frameWidth;
  params.resolution.height = frameHeight;
  params.intraPeriod = keyframe_forced_interval_;
  params.rcMode = RATE_CONTROL_CQP;
  params.rcParams.bitRate = 5000;
  s = encoder->setParameters(VideoParamsTypeCommon, &params);
  if (s != YAMI_SUCCESS) {
    ROS_ERROR("Failed to set parameters for yami encoder, status code:%d", s);
    return false;
  }

  s = encoder->start();
  if (s != YAMI_SUCCESS) {
    ROS_ERROR("Failed to start yami encoder, status code:%d", s);
    return false;
  }
  encoder->getMaxOutSize(&max_output_buf_size_);
  encoder_ = encoder;
  return true;
}

bool HardwareEncoder::initialized() {
  return encoder_ != NULL;
}
void HardwareEncoder::connect() {
  if (encoder_) {
    YamiStatus s = encoder_->start();
    if (s != YAMI_SUCCESS) {
      ROS_ERROR("Failed to start yami encoder when connect, status code:%d", s);
      return;
    }
  }
  frame_count_ = 0;
}

void HardwareEncoder::disconnect() {
  if (encoder_) {
    encoder_->flush();
    YamiStatus s = encoder_->stop();
    if (s != YAMI_SUCCESS) {
      ROS_ERROR("Failed to start yami encoder when disconnect, status code:%d", s);
      return;
    }
  }
}

} // namespace vpx_streamer
