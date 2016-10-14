// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "vpx_image_transport/vpx_subscriber.h"

namespace vpx_image_transport {

VPXSubscriber::~VPXSubscriber() {
}

void VPXSubscriber::internalCallback(
    const typename vpx_image_transport::Packet::ConstPtr& message,
    const Callback& user_cb) {
}

} // namespace vpx_image_transport
