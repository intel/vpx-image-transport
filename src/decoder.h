// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef DECODER_H
#define DECODER_H

namespace vpx_image_transport {

class DecoderDelegate {
public:
  //virtual void
};

class Decoder {
public:
  virtual ~Decoder() = 0;

protected:
  Decoder(DecoderDelegate* delegate) : delegate_(delegate) {};

  DecoderDelegate* delegate_;
};

} // namespace vpx_image_transport

#endif // DECODER_H
