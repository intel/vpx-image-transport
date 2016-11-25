// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_subscriber.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vpx/vp8dx.h>

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

  webm_tools::WebMFile::Status status =
    subscriber_->webm_file_->ParseNextChunk(subscriber_->buffer_.data(),
                                            subscriber_->buffer_.size(), &bytes_read_);

  switch(status) {
    case webm_tools::WebMFile::kInvalidWebM:
      ROS_ERROR("Invalid WebM");
      return false;
    case webm_tools::WebMFile::kParsingError:
      ROS_ERROR("Parsing error");
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
  codec_context_(NULL), decoder_config_(NULL), image_index_(0) {
  buffer_.reserve(RESERVED_BUFFER_SIZE);
}

VPXSubscriber::~VPXSubscriber() {
  if (VPX_CODEC_OK != vpx_codec_destroy(codec_context_)) {
    ROS_ERROR("Failed to destroy VPX decoder context.");
  }
  if (webm_file_)
    delete webm_file_;

  delete codec_context_;
  delete decoder_config_;
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

  long long ret = 0;
  if (!codec_context_) {
    codec_context_ = new vpx_codec_ctx_t();
    decoder_config_ = new vpx_codec_dec_cfg();
    decoder_config_->threads = 1;
    decoder_config_->w = static_cast<unsigned int>(track_->GetWidth());
    decoder_config_->h = static_cast<unsigned int>(track_->GetHeight());

    ret = vpx_codec_dec_init(codec_context_, vpx_codec_vp8_dx(), decoder_config_, 0);
    if (ret) {
      ROS_ERROR("Failed to initialize VPX context. Error No.:%lld", ret);
      return;
    }
  }

  for (int i = 0; i < block->GetFrameCount(); ++i) {
    long long int position = block->GetFrame(i).pos;
    long int length = block->GetFrame(i).len;
    uint8_t* frame_buffer = static_cast<uint8_t*>(buffer_.data()
                            + (position - bytes_consumed_) * sizeof(uint8_t));

    const vpx_codec_err_t ret = vpx_codec_decode(codec_context_, frame_buffer, length, NULL, 0);
    if (ret != VPX_CODEC_OK) {
      ROS_ERROR("Failed to decode frame:%d, VPX error code:%d", i, ret);
      return;
    }
    decodeImage(user_cb);
  }
}

void VPXSubscriber::decodeImage(const Callback& user_cb) {
  vpx_codec_iter_t iter = NULL;
  vpx_image_t* image = NULL;
  while ((image = vpx_codec_get_frame(codec_context_, &iter)) != NULL) {
    assert(VPX_IMG_FMT_I420 == image->fmt);

    cv::Mat i420;
    i420.create(cv::Size(image->d_w, image->d_h * 3 / 2), CV_8U);
    unsigned char* dest = i420.data;
    for (int plane = 0; plane < 3; ++plane) {
      const unsigned char *buf = image->planes[plane];
      const int stride = image->stride[plane];
      const int w = (plane ? (image->d_w + 1) >> 1 : image->d_w)
                     * ((image->fmt & VPX_IMG_FMT_HIGHBITDEPTH) ? 2 : 1);
      const int h = plane ? (image->d_h + 1) >> 1 : image->d_h;

      for (int y = 0; y < h; ++y) {
        memcpy(dest, buf, w);
        buf += stride;
        dest += w;
      }
    }
    vpx_img_free(image);

    cv::Mat bgr;
    cv::cvtColor(i420, bgr, CV_YUV2BGR_I420);

    std_msgs::Header header;
    header.seq = ++image_index_;
    header.stamp = ros::Time::now();
    cv_bridge::CvImage cv_img(header, "bgr8", bgr);
    user_cb(cv_img.toImageMsg());
  }
}

} // namespace vpx_image_transport
