// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_subscriber.h"

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace vpx_image_transport {

using namespace mkvparser;

const uint64_t RESERVED_BUFFER_SIZE = 2 * 1024 * 1024; // 2MB

VPXBufferManager::VPXBufferManager(VPXSubscriber *subscriber)
  : subscriber_(subscriber), bytes_read_(-1) {
}

VPXBufferManager::~VPXBufferManager() {
  if (bytes_read_ == -1) {
    ROS_WARN("More data required, waiting for the next chunk...");
    return;
  }
  subscriber_->buffer_.erase(subscriber_->buffer_.begin(),
                             subscriber_->buffer_.begin() + bytes_read_);
  subscriber_->bytes_consumed_ += bytes_read_;
}

bool VPXBufferManager::load( const typename vpx_image_transport::Packet::ConstPtr& message) {
  subscriber_->buffer_.insert(subscriber_->buffer_.end(),
                              message->data.begin(), message->data.end());

  int status =
    subscriber_->webm_file_->ParseNextChunk(subscriber_->buffer_.data(),
                                            subscriber_->buffer_.size(), &bytes_read_);

  switch(status) {
    case webm_tools::WebMFile::kInvalidWebM:
      ROS_ERROR("Invalid WebM.");
      return false;
    case webm_tools::WebMFile::kParsingError:
      ROS_ERROR("Parsing error.");
      return false;
    case webm_tools::WebMFile::kParsingHeader:
      ROS_INFO("Parsing header, bytes read:%d", bytes_read_);
      return false;
    case webm_tools::WebMFile::kParsingClusters:
      break;
    case webm_tools::WebMFile::kParsingFinalElements:
      ROS_INFO("Parsing final elements, bytes read:%d", bytes_read_);
      break;
    case webm_tools::WebMFile::kParsingDone:
      ROS_INFO("Parsing done, bytes read:%d", bytes_read_);
      break;
    default:
      ROS_INFO("Unknown status:%d, bytes read:%d", status, bytes_read_);
  }
  return true;
}

VPXSubscriber::VPXSubscriber()
  : webm_file_(NULL), bytes_consumed_(0), track_(NULL), current_block_(NULL),
  image_index_(0), user_callback_(NULL) {
  buffer_.reserve(RESERVED_BUFFER_SIZE);
  decoder_.reset(codec_factory_.createDecoder(this));
}

VPXSubscriber::~VPXSubscriber() {
  if (webm_file_)
    delete webm_file_;
}

void VPXSubscriber::internalCallback(
    const typename vpx_image_transport::Packet::ConstPtr& message,
    const Callback& user_cb) {

  if (!webm_file_) {
    webm_file_ = new webm_tools::WebMFile();
  }

  VPXBufferManager manager(this);
  if (!manager.load(message)) {
    return;
  }

  if (!track_) {
    const Tracks* tracks = webm_file_->GetSegment()->GetTracks();
    if (!tracks) {
      ROS_WARN("Failed to get tracks, will retry.");
      return;
    }

    assert(tracks->GetTracksCount() == 1);
    const Track* track = tracks->GetTrackByIndex(0);
    if (!track) {
      ROS_WARN("Failed to get first track in tracks, will retry.");
      return;
    }
    assert(track->GetType() == Track::kVideo);
    track_ = dynamic_cast<const VideoTrack*>(track);
  }

  const BlockEntry* current = NULL;
  while ((current = retriveBlockEntry(current_block_)) != NULL) {
    processBlockEntry(current, user_cb);
    current_block_ = current;
  }
}

const BlockEntry* VPXSubscriber::retriveBlockEntry(const BlockEntry* current) {
  const BlockEntry* entry = NULL;
  if (!current) {
    track_->GetFirst(entry);
  } else {
    track_->GetNext(current, entry);
  }
  return entry;
}

void VPXSubscriber::processBlockEntry(const BlockEntry* entry, const Callback& user_cb) {
  assert(entry->GetKind() == BlockEntry::kBlockSimple);
  const Block* block = entry->GetBlock();
  if (!block) {
    ROS_ERROR("Failed to get block from block entry");
    return;
  }

  for (int i = 0; i < block->GetFrameCount(); ++i) {
    long long int position = block->GetFrame(i).pos;
    long int length = block->GetFrame(i).len;
    uint8_t* frame_buffer = static_cast<uint8_t*>(buffer_.data()
                            + (position - bytes_consumed_) * sizeof(uint8_t));
    decodeFrame(frame_buffer, length, user_cb);
  }
}

void VPXSubscriber::decodeFrame(uint8_t* buffer, long int size, const Callback& user_cb) {
  if (!decoder_->initialized()) {
    if (decoder_->createDecoder(track_->GetWidth(), track_->GetHeight())) {
      ROS_INFO("Hardware accelerated decoder enabled.");
    } else {
      ROS_WARN("Failed to create decoder, will retry.");
      return;
    }
  }
  user_callback_ = &user_cb;
  decoder_->decode(buffer, size);
  user_callback_ = NULL;
}

void VPXSubscriber::onFrameDecoded(const cv::Mat& bgr) {
  if (!user_callback_) {
    return;
  }

  std_msgs::Header header;
  header.seq = ++image_index_;
  header.stamp = ros::Time::now();
  cv_bridge::CvImage cv_img(header, "bgr8", bgr);
  (*user_callback_)(cv_img.toImageMsg());
}

} // namespace vpx_image_transport
