// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "encoder_factory.h"

#include <ros/ros.h>
#include <va/va_x11.h>
#include <VideoEncoderHost.h>
#include "software_encoder.h"
#include "yami_encoder.h"

namespace vpx_image_transport {

EncoderFactory::EncoderFactory() : va_display_(NULL) {
}

EncoderFactory::~EncoderFactory() {
}

Encoder* EncoderFactory::createEncoder(EncoderDelegate* delegate, CreationMethod method) {
  Encoder* encoder = NULL;
  if (method != SOFTWARE_ONLY && isHardwareAccelerationSupported()) {
    NativeDisplay* display = new NativeDisplay;
    display->type = NATIVE_DISPLAY_VA;
    display->handle = (intptr_t)va_display_;
    encoder = new YamiEncoder(delegate, display);
  } else {
    encoder = new SoftwareEncoder(delegate);
  }
  return encoder;
}

bool EncoderFactory::initDisplay() {
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

bool EncoderFactory::isHardwareAccelerationSupported() {
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

} // namespace vpx_image_transport
