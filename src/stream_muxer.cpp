// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "stream_muxer.h"

#include <ros/ros.h>

namespace vpx_image_transport {

using namespace webm_tools;
using namespace mkvmuxer;

StreamMuxer::StreamMuxer(StreamMuxerDelegate* delegate)
  : delegate_(delegate), muxer_(NULL), encoder_(codec_factory_.createEncoder(this)) {
}

StreamMuxer::~StreamMuxer() {
  if (muxer_) {
    muxer_->Finalize();
    delete muxer_;
  }
}

void StreamMuxer::configure(const EncoderConfig& config) {
  if (encoder_) {
    encoder_->configure(config);
  }
}

void StreamMuxer::connect() {
  encoder_->connect();
  if (muxer_) {
    delete muxer_;
  }
  muxer_ = new webm_tools::WebMLiveMuxer();
}

void StreamMuxer::disconnect() {
  int ret = muxer_->Finalize();
  if (ret != WebMLiveMuxer::kSuccess) {
    ROS_ERROR("Failed to finalize live muxer with error code: %d", ret);
    return;
  }
  int chunk_length = 0;
  if (!muxer_->ChunkReady(&chunk_length)) {
    ROS_ERROR("Failed to get chunk after finalized called.");
  }

  encoder_->disconnect();
}

void StreamMuxer::encodeImage(const cv::Mat& bgr, int frameWidth, int frameHeight) {
  if (!muxer_) {
    ROS_WARN("Failed to create muxer, will wait for the connection.");
    return;
  }
  if (!encoder_->initialized()) {
    if (!encoder_->createEncoder(frameWidth, frameHeight)) {
      ROS_WARN("Failed to create encoder, will retry.");
      return;
    }
  }

  if (!muxer_->initialized()) {
    muxer_->Init();
    muxer_->AddVideoTrack(frameWidth, frameHeight);
  }

  if (encoder_->initialized()) {
    encoder_->encode(bgr);
  }

  webm_tools::int32 chunk_length = 0;
  if (!muxer_->ChunkReady(&chunk_length)) {
    return;
  }

  std::vector<uint8_t> buffer = std::vector<uint8_t>(chunk_length);
  uint8_t* data = buffer.data();
  int ret = muxer_->ReadChunk(chunk_length, data);
  if (WebMLiveMuxer::kSuccess != ret) {
    ROS_ERROR("Failed to read chunk with error code: %d", ret);
    return;
  }
  delegate_->onChunkReady(buffer);
}

void StreamMuxer::onWriteFrame(uint8_t* buffer, uint64_t size,
                               uint64_t timeStamp, bool isKeyFrame) {
  if (!muxer_) {
    ROS_WARN("Failed to create muxer, will wait for the connection.");
    return;
  }
  muxer_->WriteVideoFrame(buffer, size, timeStamp, isKeyFrame);
}

} // namespace vpx_image_transport
