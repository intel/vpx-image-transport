// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "codec_factory.h"

#include <ros/ros.h>
#include <va/va_x11.h>
#include <VideoEncoderHost.h>
#include <VideoDecoderHost.h>
#include "hardware_decoder.h"
#include "hardware_encoder.h"
#include "software_decoder.h"
#include "software_encoder.h"

namespace vpx_streamer {

CodecFactory::CodecFactory() : va_display_(NULL) {
}

CodecFactory::~CodecFactory() {
}

Encoder* CodecFactory::createEncoder(EncoderDelegate* delegate, CreationMethod method) {
  Encoder* encoder = NULL;
  if (method != SOFTWARE_ONLY && isHardwareAcceleratedEncoderSupported()) {
    NativeDisplay* display = new NativeDisplay;
    display->type = NATIVE_DISPLAY_VA;
    display->handle = (intptr_t)va_display_;
    encoder = new HardwareEncoder(delegate, display);
  } else {
    encoder = new SoftwareEncoder(delegate);
  }
  return encoder;
}

Decoder* CodecFactory::createDecoder(DecoderDelegate* delegate, CreationMethod method) {
  Decoder* decoder = NULL;
  if (method != SOFTWARE_ONLY && isHardwareAcceleratedDecoderSupported()) {
    NativeDisplay* display = new NativeDisplay;
    display->type = NATIVE_DISPLAY_VA;
    display->handle = (intptr_t)va_display_;
    decoder = new HardwareDecoder(delegate, va_display_, display);
  } else {
    decoder = new SoftwareDecoder(delegate);
  }
  return decoder;
}

bool CodecFactory::initDisplay() {
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
  return true;
}

bool CodecFactory::isHardwareAcceleratedEncoderSupported() {
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

bool CodecFactory::isHardwareAcceleratedDecoderSupported() {
  if (!va_display_ && !initDisplay()) {
    return false;
  }

  VAEntrypoint* entry_points = new VAEntrypoint[vaMaxNumEntrypoints(va_display_)];
  int num_of_entry_points = 0;
  VAStatus s = vaQueryConfigEntrypoints(va_display_, VAProfileVP8Version0_3, entry_points, &num_of_entry_points);
  if (s != VA_STATUS_SUCCESS) {
    ROS_ERROR("Failed to query config entry points.");
    return false;
  }
  bool vp8_decoder_supported = false;
  for (int i = 0; i < num_of_entry_points; ++i) {
    if (entry_points[i] == VAEntrypointVLD) {
      vp8_decoder_supported = true;
      break;
    }
  }
  delete [] entry_points;
  if (!vp8_decoder_supported) {
    return false;
  }

  // Try to create yami codec first, if fails create vpx context instead.
  std::vector<std::string> codecs = getVideoDecoderMimeTypes();
  std::vector<std::string>::iterator finder =
    std::find(codecs.begin(), codecs.end(), YAMI_MIME_VP8);
  return finder != codecs.end() ? true : false;
}

} // namespace vpx_streamer
