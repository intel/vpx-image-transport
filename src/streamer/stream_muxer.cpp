// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "stream_muxer.h"

#include <boost/scoped_ptr.hpp>
#include <webm_live_muxer.h>
#include "stream_logger.h"

namespace vpx_streamer {

using namespace webm_tools;
using namespace mkvmuxer;

StreamMuxer::StreamMuxer(StreamMuxerDelegate* delegate, bool use_vector)
    : wrap_chunk_in_vector_(use_vector),
      delegate_(delegate),
      encoder_(codec_factory_.createEncoder(this)) {
}

StreamMuxer::~StreamMuxer() {
  if (muxer_) {
    muxer_->Finalize();
  }
}

void StreamMuxer::configure(const EncoderConfig& config) {
  if (encoder_) {
    encoder_->configure(config);
  }
}

void StreamMuxer::connect() {
  encoder_->connect();
  muxer_.reset(new webm_tools::WebMLiveMuxer());
}

void StreamMuxer::disconnect() {
  int ret = muxer_->Finalize();
  if (ret != WebMLiveMuxer::kSuccess) {
    STREAM_LOG_ERROR("Failed to finalize live muxer with error code: %d", ret);
    return;
  }
  int chunk_length = 0;
  if (!muxer_->ChunkReady(&chunk_length)) {
    STREAM_LOG_ERROR("Failed to get chunk after finalized called.");
  }

  encoder_->disconnect();
}

void StreamMuxer::encodeImage(
    const cv::Mat& bgr, int frameWidth, int frameHeight, bool isBgr) {
  if (!muxer_) {
    STREAM_LOG_WARN("Failed to create muxer, will wait for the connection.");
    return;
  }
  if (!encoder_->initialized()) {
    if (!encoder_->initialize(frameWidth, frameHeight)) {
      STREAM_LOG_WARN("Failed to create encoder, will retry.");
      return;
    }
  }

  if (!muxer_->initialized()) {
    muxer_->Init();
    muxer_->AddVideoTrack(frameWidth, frameHeight);
  }

  if (encoder_->initialized()) {
    encoder_->encode(bgr, isBgr);
  }

  webm_tools::int32 chunk_length = 0;
  if (!muxer_->ChunkReady(&chunk_length)) {
    return;
  }
  uint8_t* data = nullptr;
  boost::scoped_ptr<std::vector<uint8_t>> buffer;
  if (wrap_chunk_in_vector_) {
    buffer.reset(new std::vector<uint8_t>(chunk_length));
    data = buffer->data();
  } else {
    data = static_cast<uint8_t*>(malloc(chunk_length));
  }
  int ret = muxer_->ReadChunk(chunk_length, data);
  if (WebMLiveMuxer::kSuccess != ret) {
    STREAM_LOG_ERROR("Failed to read chunk with error code: %d", ret);
    return;
  }
  if (wrap_chunk_in_vector_)
    delegate_->onChunkReady(*buffer);
  else
    delegate_->onChunkReadyRawData(data, chunk_length);
}

void StreamMuxer::onWriteFrame(uint8_t* buffer, uint64_t size,
                               uint64_t timeStamp, bool isKeyFrame) {
  if (!muxer_) {
    STREAM_LOG_WARN("Failed to create muxer, will wait for the connection.");
    return;
  }
  muxer_->WriteVideoFrame(buffer, size, timeStamp, isKeyFrame);
}

} // namespace vpx_streamer
