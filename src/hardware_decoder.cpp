// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "hardware_decoder.h"

#include <ros/ros.h>
#include <va/va_x11.h>
#include <VideoDecoderHost.h>

namespace vpx_image_transport {

HardwareDecoder::HardwareDecoder(DecoderDelegate* delegate, VADisplay vaDisplay,
                                 NativeDisplay* nativeDisplay)
  : Decoder(delegate), yami_decoder_(NULL), va_display_(vaDisplay),
    native_display_(nativeDisplay) {
}

HardwareDecoder::~HardwareDecoder() {
}

bool HardwareDecoder::createDecoder(int frameWidth, int frameHeight) {
  yami_decoder_ = createVideoDecoder(YAMI_MIME_VP8);
  if (!yami_decoder_) {
    ROS_ERROR("Failed to create yami decoder with mime type: %s.", YAMI_MIME_VP8);
    return false;
  }

  yami_decoder_->setNativeDisplay(native_display_.get());

  VideoConfigBuffer config_buffer;
  memset(&config_buffer, 0, sizeof(VideoConfigBuffer));
  config_buffer.profile = VAProfileVP8Version0_3;
  config_buffer.fourcc = YAMI_FOURCC_NV12;
  config_buffer.width = frameWidth;
  config_buffer.height = frameHeight;
  Decode_Status status = yami_decoder_->start(&config_buffer);
  assert(status == DECODE_SUCCESS);
  return true;
}

void HardwareDecoder::decode(uint8_t* buffer, uint64_t size) {
  assert(yami_decoder_);

  VideoDecodeBuffer input_buffer;
  input_buffer.data = buffer;
  input_buffer.size = size;

  Decode_Status status = yami_decoder_->decode(&input_buffer);
  if (status == DECODE_SUCCESS) {
    while (true) {
      SharedPtr<VideoFrame> frame = yami_decoder_->getOutput();
      if (!frame) {
        break;
      }
      VAImage image;
      VAStatus s = vaDeriveImage(va_display_, frame->surface, &image);
      if (s != VA_STATUS_SUCCESS) {
        ROS_ERROR("Failed to derive VA image. status code:%d", s);
        return;
      }

      void* image_buf = NULL;
      s = vaMapBuffer(va_display_, image.buf, &image_buf);
      if (s != VA_STATUS_SUCCESS) {
        ROS_ERROR("Fail to map buffer from VAImage, status code:%d", s);
        return;
      }

      cv::Mat nv12;
      nv12.create(cv::Size(image.width, image.height * 3 / 2), CV_8U);
      unsigned char* dest = nv12.data;
      for (int plane = 0; plane < image.num_planes; ++plane) {
        const unsigned char *buf = (const unsigned char*)image_buf + image.offsets[plane];
        const int stride = image.pitches[plane];
        const int w = image.width;
        const int h = plane ? (image.height + 1) >> 1 : image.height;

        for (int y = 0; y < h; ++y) {
          memcpy(dest, buf, w);
          buf += stride;
          dest += w;
        }
      }

      cv::Mat bgr;
      cv::cvtColor(nv12, bgr, CV_YUV2BGR_NV12);
      delegate_->onFrameDecoded(bgr);

      vaUnmapBuffer(va_display_, image.buf);
      vaDestroyImage(va_display_, image.image_id);
    }
  } else if (status == DECODE_FORMAT_CHANGE) {
    ROS_INFO("Decode format change.");
  } else {
    ROS_WARN("Unknown decode status code:%d", status);
  }
}

bool HardwareDecoder::initialized() {
  return yami_decoder_ != NULL;
}

} // namespace vpx_image_transport
