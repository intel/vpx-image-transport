// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef SOFTWARE_DECODER_H
#define SOFTWARE_DECODER_H

#include <boost/shared_ptr.hpp>
#include <vpx/vpx_decoder.h>
#include "decoder.h"

namespace vpx_streamer {

class SoftwareDecoder : public Decoder {
public:
  SoftwareDecoder(DecoderDelegate* delegate);
  virtual ~SoftwareDecoder();

  virtual bool initialize(int frameWidth, int frameHeight);
  virtual void decode(uint8_t* buffer, uint64_t size);
  virtual bool initialized();

private:
  boost::shared_ptr<vpx_codec_ctx_t> codec_context_;
  boost::shared_ptr<vpx_codec_dec_cfg_t> decoder_config_;
};

} // namespace vpx_streamer

#endif // SOFTWARE_DECODER_H
