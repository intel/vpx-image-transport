// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef VPX_SUBSCRIBER_H
#define VPX_SUBSCRIBER_H

#include <image_transport/simple_subscriber_plugin.h>
#include <vpx_image_transport/Packet.h>
#include "stream_parser.h"

namespace vpx_image_transport {

class VPXSubscriber
  : public image_transport::SimpleSubscriberPlugin<vpx_image_transport::Packet>,
    public vpx_streamer::StreamParserDelegate {
public:
  VPXSubscriber();
  virtual ~VPXSubscriber();
  virtual std::string getTransportName() const { return "vpx"; }

protected:
  virtual void internalCallback(
      const typename vpx_image_transport::Packet::ConstPtr& message,
      const Callback& user_cb);

private:
  // StreamParserDelegate implementation
  virtual void onImageDecoded(const cv::Mat& bgr);

  vpx_streamer::StreamParser stream_parser_;
  unsigned long image_index_;
  const Callback* user_callback_;
};

} // namespace vpx_image_transport

#endif // VPX_SUBSCRIBER_H
