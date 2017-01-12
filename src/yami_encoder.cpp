// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "yami_encoder.h"

#include <ros/ros.h>
#include <va/va_x11.h>
#include <VideoEncoderHost.h>

namespace vpx_image_transport {

YamiEncoder::YamiEncoder(EncoderDelegate* delegate)
  : Encoder(delegate), encoder_(NULL), va_display_(0), max_output_buf_size_(0),
    keyframe_forced_interval_(4), frame_count_(0) {
}

YamiEncoder::~YamiEncoder() {
  if (encoder_) {
    releaseVideoEncoder(encoder_);
    delete encoder_;
  }
}

bool YamiEncoder::initDisplay() {
  Display* display = XOpenDisplay(NULL);
  if (!display) {
    ROS_ERROR("Failed to open X display.");
    return false;
  }
  va_display_ = vaGetDisplay(display);
  int major, minor;
  VAStatus status = vaInitialize(va_display_, &major, &minor);
  if (status != VA_STATUS_SUCCESS) {
    ROS_ERROR("Failed to init va, with status code:%d", status);
    return false;
  }
  native_display_.reset(new NativeDisplay);
  native_display_->type = NATIVE_DISPLAY_VA;
  native_display_->handle = (intptr_t)va_display_;

  return true;
}

bool YamiEncoder::isHardwareAccelerationSupported() {
  if (!va_display_ && !initDisplay()) {
    return false;
  }

  VAEntrypoint* entry_points = new VAEntrypoint[vaMaxNumEntrypoints(va_display_)];
  int num_of_entry_points = 0;
  VAStatus s = vaQueryConfigEntrypoints(va_display_, VAProfileVP8Version0_3, entry_points, &num_of_entry_points);
  if (s != VA_STATUS_SUCCESS) {
    ROS_ERROR("Failed to query VA config entry points.");
    return false;
  }
  bool vp8_encoder_supported = false;
  for (int i = 0; i < num_of_entry_points; ++i) {
    if (entry_points[i] == VAEntrypointEncSlice) {
      vp8_encoder_supported = true;
      break;
    }
  }
  delete [] entry_points;
  if (!vp8_encoder_supported) {
    return false;
  }

  std::vector<std::string> codecs = getVideoEncoderMimeTypes();
  std::vector<std::string>::iterator finder =
    std::find(codecs.begin(), codecs.end(), YAMI_MIME_VP8);
  return finder != codecs.end() ? true : false;
}

void YamiEncoder::fillVideoFrame(VideoFrameRawData* frame, const cv::Mat& mat,
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

void YamiEncoder::encode(const cv::Mat& mat) {
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

bool YamiEncoder::createEncoder(int frameWidth, int frameHeight) {
  assert(!encoder_);

  if (!isHardwareAccelerationSupported()) {
    ROS_WARN("Hardware accelerated encoding is not supported on your platform.");
    return false;
  }

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

bool YamiEncoder::initialized() {
  return encoder_ != NULL;
}
void YamiEncoder::connect() {
  if (encoder_) {
    YamiStatus s = encoder_->start();
    if (s != YAMI_SUCCESS) {
      ROS_ERROR("Failed to start yami encoder when connect, status code:%d", s);
      return;
    }
  }
  frame_count_ = 0;
}

void YamiEncoder::disconnect() {
  if (encoder_) {
    encoder_->flush();
    YamiStatus s = encoder_->stop();
    if (s != YAMI_SUCCESS) {
      ROS_ERROR("Failed to start yami encoder when disconnect, status code:%d", s);
      return;
    }
  }
}

} // namespace vpx_image_transport
