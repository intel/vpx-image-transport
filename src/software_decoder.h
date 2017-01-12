// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SOFTWARE_DECODER_H
#define SOFTWARE_DECODER_H

#include "decoder.h"

namespace vpx_image_transport {

class SoftwareDecoder : public Decoder {
public:
  SoftwareDecoder(DecoderDelegate* delegate);
};

} // namespace vpx_image_transport


#endif // SOFTWARE_DECODER_H
