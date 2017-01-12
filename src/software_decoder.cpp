// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "software_decoder.h"

namespace vpx_image_transport {

SoftwareDecoder::SoftwareDecoder(DecoderDelegate* delegate)
  : Decoder(delegate) {
}

} // namespace vpx_image_transport
