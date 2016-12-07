// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_publisher.h"

#include <cv_bridge/cv_bridge.h>
#include <va/va_x11.h>
#include <vpx/vp8cx.h>
#include <VideoEncoderHost.h>

namespace vpx_image_transport {

  using namespace webm_tools;
  using namespace mkvmuxer;

VPXPublisher::VPXPublisher()
  : codec_context_(NULL), encoder_config_(NULL), frame_count_(0),
    package_sequence_(0), keyframe_forced_interval_(4), muxer_(NULL),
    yami_encoder_(NULL), va_display_(0), yami_max_output_buf_size_(0) {
}

VPXPublisher::~VPXPublisher() {
  muxer_->Finalize();
  delete muxer_;

  if (yami_encoder_) {
    releaseVideoEncoder(yami_encoder_);
    delete yami_encoder_;
  }

  if (codec_context_) {
    if (VPX_CODEC_OK != vpx_codec_destroy(codec_context_)) {
      ROS_ERROR("Failed to destroy VPX encoder context.");
    }
    delete codec_context_;
  }

  if (encoder_config_) {
    delete encoder_config_;
  }
}

void VPXPublisher::advertiseImpl(ros::NodeHandle &nh,
    const std::string& base_topic,
    uint32_t queue_size,
    const image_transport::SubscriberStatusCallback& user_connect_cb,
    const image_transport::SubscriberStatusCallback& user_disconnect_cb,
    const ros::VoidPtr& tracked_object,
    bool latch) {
  typedef image_transport::SimplePublisherPlugin<vpx_image_transport::Packet> Base;
  Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb,
    user_disconnect_cb, tracked_object, latch);

  // Set up reconfigure server for this topic
  reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
  ReconfigureServer::CallbackType f =
      boost::bind(&VPXPublisher::configCallback, this, _1, _2);
  reconfigure_server_->setCallback(f);
}

void VPXPublisher::configCallback(Config& config, uint32_t level) {
  vpx_codec_err_t ret;
  if (!encoder_config_) {
    encoder_config_ = new vpx_codec_enc_cfg();
    ret = vpx_codec_enc_config_default(vpx_codec_vp8_cx(), encoder_config_, 0);
    if (ret) {
      ROS_ERROR("Failed to get default encoder configuration. Error No.: %d", ret);
    }
  }

  encoder_config_->g_threads = config.threads;
  encoder_config_->rc_end_usage = static_cast<vpx_rc_mode>(config.end_usage);
  encoder_config_->rc_target_bitrate = config.target_bitrate;

  // Keyframe configurations
  keyframe_forced_interval_ = config.keyframe_forced_interval;
  encoder_config_->kf_mode = static_cast<vpx_kf_mode>(config.keyframe_mode);
  encoder_config_->kf_min_dist = config.keyframe_min_interval;
  encoder_config_->kf_max_dist = config.keyframe_max_interval;

  if (codec_context_) {
    ret = vpx_codec_enc_config_set(codec_context_, encoder_config_);
    if (ret) {
      ROS_ERROR("Failed to update codec configuration. Error No.:%d", ret);
    }
  }
}

void VPXPublisher::publish(const sensor_msgs::Image& message,
                           const PublishFn& publish_fn) const {
  // conversion necessary
  std::string encoding;
  if (sensor_msgs::image_encodings::isColor(message.encoding)
      || sensor_msgs::image_encodings::isMono(message.encoding)) {
    encoding = sensor_msgs::image_encodings::BGR8;
  } else if (message.encoding == std::string("16UC1")) {
    encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  } else if (message.encoding == std::string("8UC1")) {
    encoding = sensor_msgs::image_encodings::TYPE_8UC1;
  } else {
    ROS_ERROR("VPX publisher is not able to handle encoding type:%s", message.encoding.c_str());
    return;
  }

  cv_bridge::CvImageConstPtr cv_image_ptr;
  try {
    cv_image_ptr = cv_bridge::toCvCopy(message, encoding);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: '%s'", e.what());
    return;
  } catch (cv::Exception& e) {
    ROS_ERROR("OpenCV exception: '%s'", e.what());
    return;
  }
  if (cv_image_ptr == 0) {
    ROS_ERROR("Unable to convert from '%s' to '%s'", message.encoding.c_str(), encoding.c_str());
    return;
  }

  int frame_width = message.width, frame_height = message.height;
  cv::Mat bgr;
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    cv::Mat gray8;
    cv_image_ptr->image.convertTo(gray8, CV_8UC1);
    cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
  } else if (encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
    cv::Mat gray8 = cv_image_ptr->image;
    cv::cvtColor(gray8, bgr, cv::COLOR_GRAY2BGR);
  } else {
    bgr = cv_image_ptr->image;
  }

  if (!yami_encoder_ && !codec_context_) {
    if (createYamiEncoder(message.width, message.height)) {
      ROS_INFO("Hardware accelerated encoder enabled.");
    } else if (!createVPXEncoder(message.width, message.height)){
      ROS_WARN("Failed to create encoder, will retry.");
      return;
    }
  }

  if (!muxer_->initialized()) {
    muxer_->Init();
    muxer_->AddVideoTrack(frame_width, frame_height);
  }

  if (yami_encoder_) {
    encodeWithYami(bgr);
  } else if (codec_context_) {
    encodeWithVPX(bgr);
  }

  sendChunkIfReady(publish_fn);
}

bool VPXPublisher::createYamiEncoder(int frame_width, int frame_height) const {
  assert(!yami_encoder_);

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
  params.resolution.width = frame_width;
  params.resolution.height = frame_height;
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
  encoder->getMaxOutSize(&yami_max_output_buf_size_);
  yami_encoder_ = encoder;
  return true;
}

bool VPXPublisher::createVPXEncoder(int frame_width, int frame_height) const {
  assert(!codec_context_);

  vpx_codec_err_t ret;
  if (!encoder_config_) {
    encoder_config_ = new vpx_codec_enc_cfg();
    ret = vpx_codec_enc_config_default(vpx_codec_vp8_cx(), encoder_config_, 0);
    if (ret) {
      ROS_ERROR("Failed to get default encoder configuration. Error No.: %d", ret);
      return false;
    }
  }

  encoder_config_->g_w = frame_width;
  encoder_config_->g_h = frame_height;

  codec_context_ = new vpx_codec_ctx_t();
  ret = vpx_codec_enc_init(codec_context_, vpx_codec_vp8_cx(), encoder_config_, 0);
  if (ret) {
    ROS_ERROR("Failed to initialize VPX encoder. Error No.:%d", ret);
    return false;
  }
  return true;
}

void VPXPublisher::fillVideoFrame(VideoFrameRawData* frame, const cv::Mat& mat,
                                  int frame_width, int frame_height) const {
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

void VPXPublisher::encodeWithYami(const cv::Mat& mat) const {
  assert(yami_encoder_);

  cv::Mat input;
  cv::cvtColor(mat, input, cv::COLOR_BGR2YUV_I420);

  VideoFrameRawData input_buffer;
  memset(&input_buffer, 0, sizeof(input_buffer));
  fillVideoFrame(&input_buffer, input, mat.size().width, mat.size().height);

  YamiStatus status = yami_encoder_->encode(&input_buffer);
  if (status != ENCODE_SUCCESS) {
    ROS_WARN("Failed to encode input buffer.");
    return;
  }

  VideoEncOutputBuffer output_buffer;
  output_buffer.data = static_cast<uint8_t*>(malloc(yami_max_output_buf_size_));
  output_buffer.bufferSize = yami_max_output_buf_size_;
  output_buffer.format = OUTPUT_EVERYTHING;

  do {
    status = yami_encoder_->getOutput(&output_buffer);
    if (status == ENCODE_SUCCESS) {
      bool keyframe = (4 & output_buffer.flag) != 0;
      muxer_->WriteVideoFrame(output_buffer.data, output_buffer.dataSize, output_buffer.timeStamp, keyframe);
    }
  } while (status != ENCODE_BUFFER_NO_MORE);
  free(output_buffer.data);
}

void VPXPublisher::encodeWithVPX(const cv::Mat& mat) const {
  assert(codec_context_);

  // Convert image to i420 color space used by vpx
  cv::Mat i420;
  cv::cvtColor(mat, i420, cv::COLOR_BGR2YUV_I420);

  vpx_image_t image;

  if (!vpx_img_wrap(&image, VPX_IMG_FMT_I420, encoder_config_->g_w,
                    encoder_config_->g_h, 1, i420.data)) {
    ROS_ERROR("Failed to wrap cv::Mat into vpx image.");
    return;
  }

  int flags = 0;
  if (frame_count_ % keyframe_forced_interval_ == 0)
    flags |= VPX_EFLAG_FORCE_KF;

  vpx_codec_iter_t iter = NULL;
  const vpx_codec_cx_pkt_t *pkt = NULL;

  const vpx_codec_err_t ret = vpx_codec_encode(codec_context_, &image,
     ros::Time::now().toNSec(), 1, flags, VPX_DL_REALTIME);
  if (ret != VPX_CODEC_OK) {
    return;
  }

  while ((pkt = vpx_codec_get_cx_data(codec_context_, &iter)) != NULL) {
    if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
      bool keyframe = (pkt->data.frame.flags & VPX_FRAME_IS_KEY) != 0;
      muxer_->WriteVideoFrame(reinterpret_cast<uint8*>(pkt->data.frame.buf), pkt->data.frame.sz, ++frame_count_, keyframe);
    } else {
      ROS_INFO("pkt->kind: %d", pkt->kind);
    }
  }
}

void VPXPublisher::sendChunkIfReady(const PublishFn &publish_fn) const {
  webm_tools::int32 chunk_length = 0;
  if (!muxer_->ChunkReady(&chunk_length)) {
    return;
  }

  vpx_image_transport::Packet packet;
  packet.data.resize(chunk_length);
  int ret = muxer_->ReadChunk(chunk_length, &packet.data[0]);
  if (WebMLiveMuxer::kSuccess != ret) {
    ROS_ERROR("Failed to read chunk with error code: %d", ret);
    return;
  }
  packet.header.seq = ++package_sequence_;
  packet.header.stamp = ros::Time::now();
  publish_fn(packet);
}

void VPXPublisher::connectCallback(const ros::SingleSubscriberPublisher& pub) {
  if (muxer_) {
    delete muxer_;
  }
  muxer_ = new webm_tools::WebMLiveMuxer();
  frame_count_ = 0;

  if (yami_encoder_) {
    YamiStatus s = yami_encoder_->start();
    if (s != YAMI_SUCCESS) {
      ROS_ERROR("Failed to start yami encoder when connect, status code:%d", s);
      return;
    }
  }
}

void VPXPublisher::disconnectCallback(const ros::SingleSubscriberPublisher &pub) {
  int ret = muxer_->Finalize();
  if (ret != WebMLiveMuxer::kSuccess) {
    ROS_ERROR("Failed to finalize live muxer with error code: %d", ret);
    return;
  }
  int chunk_length = 0;
  if (!muxer_->ChunkReady(&chunk_length)) {
    ROS_ERROR("Failed to get chunk after finalized called.");
  }

  if (yami_encoder_) {
    yami_encoder_->flush();
    YamiStatus s = yami_encoder_->stop();
    if (s != YAMI_SUCCESS) {
      ROS_ERROR("Failed to start yami encoder when disconnect, status code:%d", s);
      return;
    }
  }
}

bool VPXPublisher::initDisplay() const {
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

bool VPXPublisher::isHardwareAccelerationSupported() const {
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

} // vpx_image_transport
