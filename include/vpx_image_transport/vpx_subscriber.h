// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef VPX_SUBSCRIBER_H
#define VPX_SUBSCRIBER_H

#include <image_transport/simple_subscriber_plugin.h>
#include <mkvparser/mkvparser.h>
#include <vpx_image_transport/Packet.h>
#include <webm_file.h>
#include "codec_factory.h"
#include "decoder.h"

namespace vpx_image_transport {

class VPXSubscriber;

class VPXBufferManager {
public:
  VPXBufferManager(VPXSubscriber* subscriber);
  ~VPXBufferManager();
  bool load(const typename vpx_image_transport::Packet::ConstPtr& message);
private:
  VPXSubscriber *subscriber_;
  int bytes_read_;
};

class VPXSubscriber
  : public image_transport::SimpleSubscriberPlugin<vpx_image_transport::Packet>,
    public DecoderDelegate {
public:
  VPXSubscriber();
  virtual ~VPXSubscriber();
  virtual std::string getTransportName() const { return "vpx"; }

protected:
  virtual void internalCallback(
      const typename vpx_image_transport::Packet::ConstPtr& message,
      const Callback& user_cb);

private:
  webm_tools::WebMFile* webm_file_;
  std::vector<uint8_t> buffer_;
  uint64_t bytes_consumed_;

  const mkvparser::VideoTrack* track_;
  const mkvparser::BlockEntry* current_block_;

  CodecFactory codec_factory_;
  boost::shared_ptr<Decoder> decoder_;

  unsigned long image_index_;
  const Callback* user_callback_;

  const mkvparser::BlockEntry* retriveBlockEntry(const mkvparser::BlockEntry* current);
  void processBlockEntry(const mkvparser::BlockEntry* entry, const Callback& user_cb);
  void decodeFrame(uint8_t* buffer, long int size, const Callback& user_cb);

  // DecoderDelegate Implementation.
  virtual void onFrameDecoded(const cv::Mat& bgr);

  friend class VPXBufferManager;
};

} // namespace vpx_image_transport

#endif // VPX_SUBSCRIBER_H
