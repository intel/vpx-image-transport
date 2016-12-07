// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_subscriber.h"

#include <algorithm>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <va/va_x11.h>
#include <vpx/vp8dx.h>
#include <VideoDecoderHost.h>

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
  codec_context_(NULL), decoder_config_(NULL), yami_decoder_(NULL), va_display_(0),
  image_index_(0) {
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

  for (int i = 0; i < block->GetFrameCount(); ++i) {
    long long int position = block->GetFrame(i).pos;
    long int length = block->GetFrame(i).len;
    uint8_t* frame_buffer = static_cast<uint8_t*>(buffer_.data()
                            + (position - bytes_consumed_) * sizeof(uint8_t));
    decodeFrame(frame_buffer, length, user_cb);
  }
}

void VPXSubscriber::decodeFrame(uint8_t* buffer, long int size, const Callback& user_cb) {
  if (!yami_decoder_ && !codec_context_) {
    if (createYamiDecoder()) {
      ROS_INFO("Hardware accelerated decoder enabled.");
    } else if (!createVPXDecoder()){
      ROS_WARN("Failed to create decoder, will retry.");
      return;
    }
  }

  if (yami_decoder_) {
    decodeFrameWithYami(buffer, size, user_cb);
  } else if (codec_context_) {
    decodeFrameWithVPX(buffer, size, user_cb);
  }
}

bool VPXSubscriber::isHardwareAccelerationSupported() {
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

bool VPXSubscriber::initDisplay() {
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

bool VPXSubscriber::createYamiDecoder() {
  if (!isHardwareAccelerationSupported()) {
    ROS_WARN("Hardware decoder is not supported on current platform.");
    return false;
  }
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
  config_buffer.width = track_->GetWidth();
  config_buffer.height = track_->GetHeight();
  Decode_Status status = yami_decoder_->start(&config_buffer);
  assert(status == DECODE_SUCCESS);
  return true;
}

bool VPXSubscriber::createVPXDecoder() {
  codec_context_ = new vpx_codec_ctx_t();
  decoder_config_ = new vpx_codec_dec_cfg();
  decoder_config_->threads = 1;
  decoder_config_->w = static_cast<unsigned int>(track_->GetWidth());
  decoder_config_->h = static_cast<unsigned int>(track_->GetHeight());

  long long ret = vpx_codec_dec_init(codec_context_, vpx_codec_vp8_dx(), decoder_config_, 0);
  if (ret) {
    ROS_ERROR("Failed to initialize VPX context. Error No.:%lld", ret);
    return false;
  }
  return true;
}

void VPXSubscriber::decodeFrameWithYami(uint8_t* buffer, long int size, const Callback& user_cb) {
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

      std_msgs::Header header;
      header.seq = ++image_index_;
      header.stamp = ros::Time::now();
      cv_bridge::CvImage cv_img(header, "bgr8", bgr);
      user_cb(cv_img.toImageMsg());

      vaUnmapBuffer(va_display_, image.buf);
      vaDestroyImage(va_display_, image.image_id);
    }
  } else if (status == DECODE_FORMAT_CHANGE) {
    ROS_INFO("Decode format change.");
  } else {
    ROS_WARN("Unknown decode status code:%d", status);
  }
}

void VPXSubscriber::decodeFrameWithVPX(uint8_t* buffer, long int size, const Callback& user_cb) {
  assert(codec_context_);
  const vpx_codec_err_t err = vpx_codec_decode(codec_context_, buffer, size, NULL, 0);
  if (err != VPX_CODEC_OK) {
    ROS_ERROR("Failed to decode frame, VPX error code:%d", err);
    return;
  }
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
