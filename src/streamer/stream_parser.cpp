// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "stream_parser.h"

#include <algorithm>
#include <mkvparser/mkvparser.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <webm_file.h>
#include "stream_logger.h"

namespace vpx_streamer {

using namespace mkvparser;

const uint64_t RESERVED_BUFFER_SIZE = 2 * 1024 * 1024; // 2MB

class StreamBufferManager {
public:
  StreamBufferManager(StreamParser* parser);
  ~StreamBufferManager();
  bool load(const std::vector<uint8_t>& payload);

private:
  StreamParser *parser_;
  int bytes_read_;
};

StreamBufferManager::StreamBufferManager(StreamParser *parser)
  : parser_(parser), bytes_read_(-1) {
}

StreamBufferManager::~StreamBufferManager() {
  if (bytes_read_ == -1) {
    STREAM_LOG_WARN("More data required, waiting for the next chunk...");
    return;
  }
  parser_->buffer_.erase(parser_->buffer_.begin(),
                         parser_->buffer_.begin() + bytes_read_);
  parser_->bytes_consumed_ += bytes_read_;
}

bool StreamBufferManager::load(const std::vector<uint8_t>& payload) {
  parser_->buffer_.insert(parser_->buffer_.end(), payload.begin(), payload.end());

  int status =
    parser_->webm_file_->ParseNextChunk(parser_->buffer_.data(),
                                        parser_->buffer_.size(), &bytes_read_);

  switch(status) {
    case webm_tools::WebMFile::kInvalidWebM:
      STREAM_LOG_ERROR("Invalid WebM.");
      return false;
    case webm_tools::WebMFile::kParsingError:
      STREAM_LOG_ERROR("Parsing error.");
      return false;
    case webm_tools::WebMFile::kParsingHeader:
      STREAM_LOG_INFO("Parsing header, bytes read:%d", bytes_read_);
      return false;
    case webm_tools::WebMFile::kParsingClusters:
      break;
    case webm_tools::WebMFile::kParsingFinalElements:
      STREAM_LOG_INFO("Parsing final elements, bytes read:%d", bytes_read_);
      break;
    case webm_tools::WebMFile::kParsingDone:
      STREAM_LOG_INFO("Parsing done, bytes read:%d", bytes_read_);
      break;
    default:
      STREAM_LOG_INFO("Unknown status:%d, bytes read:%d", status, bytes_read_);
  }
  return true;
}

StreamParser::StreamParser(StreamParserDelegate* delegate)
  : delegate_(delegate), bytes_consumed_(0), track_(NULL), current_block_(NULL) {
  buffer_.reserve(RESERVED_BUFFER_SIZE);
  decoder_.reset(codec_factory_.createDecoder(this));
}

StreamParser::~StreamParser() {
}

void StreamParser::decodeStream(const std::vector<uint8_t>& buffer) {
  if (!webm_file_) {
    webm_file_.reset(new webm_tools::WebMFile());
  }

  StreamBufferManager manager(this);
  if (!manager.load(buffer)) {
    return;
  }

  if (!track_) {
    const Tracks* tracks = webm_file_->GetSegment()->GetTracks();
    if (!tracks) {
      STREAM_LOG_WARN("Failed to get tracks, will retry.");
      return;
    }

    assert(tracks->GetTracksCount() == 1);
    track_ = dynamic_cast<const VideoTrack*>(tracks->GetTrackByIndex(0));
    if (!track_) {
      STREAM_LOG_WARN("Failed to get first track in tracks, will retry.");
      return;
    }
  }

  const BlockEntry* current = NULL;
  while ((current = retriveBlockEntry(current_block_)) != NULL) {
    processBlockEntry(current);
    current_block_ = current;
  }
}

const BlockEntry* StreamParser::retriveBlockEntry(const BlockEntry* current) {
  const BlockEntry* entry = NULL;
  if (!current) {
    track_->GetFirst(entry);
  } else {
    track_->GetNext(current, entry);
  }
  return entry;
}

void StreamParser::processBlockEntry(const BlockEntry* entry) {
  assert(entry->GetKind() == BlockEntry::kBlockSimple);
  const Block* block = entry->GetBlock();
  if (!block) {
    STREAM_LOG_ERROR("Failed to get block from block entry");
    return;
  }

  for (int i = 0; i < block->GetFrameCount(); ++i) {
    long long int position = block->GetFrame(i).pos;
    long int length = block->GetFrame(i).len;
    uint8_t* frame_buffer = static_cast<uint8_t*>(buffer_.data()
                            + (position - bytes_consumed_) * sizeof(uint8_t));

    if (!decoder_->initialized()) {
      if (!decoder_->initialize(track_->GetWidth(), track_->GetHeight())) {
        STREAM_LOG_WARN("Failed to create decoder, will retry.");
        return;
      }
    }
    decoder_->decode(frame_buffer, length);
  }
}

void StreamParser::onFrameDecoded(const cv::Mat& bgr) {
  delegate_->onImageDecoded(bgr);
}

} // namespace vpx_streamer
