// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef VPX_SUBSCRIBER_H
#define VPX_SUBSCRIBER_H

#include <image_transport/simple_subscriber_plugin.h>
#include <vpx_image_transport/Packet.h>

namespace vpx_image_transport {

class VPXSubscriber : public
  image_transport::SimpleSubscriberPlugin<vpx_image_transport::Packet> {
public:
  virtual ~VPXSubscriber();
  virtual std::string getTransportName() const { return "vpx"; }

protected:
  virtual void internalCallback(
      const typename vpx_image_transport::Packet::ConstPtr& message,
      const Callback& user_cb);
};

} // namespace vpx_image_transport

#endif // VPX_SUBSCRIBER_H
