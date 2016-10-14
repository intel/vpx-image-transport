// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <pluginlib/class_list_macros.h>
#include "vpx_image_transport/vpx_publisher.h"
#include "vpx_image_transport/vpx_subscriber.h"

PLUGINLIB_EXPORT_CLASS(vpx_image_transport::VPXPublisher,
                       image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(vpx_image_transport::VPXSubscriber,
                       image_transport::SubscriberPlugin)
