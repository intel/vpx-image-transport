// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef STREAM_MUXER_H
#define STREAM_MUXER_H

#include <boost/shared_ptr.hpp>
#include "codec_factory.h"
#include "encoder.h"

namespace webm_tools {
  class WebMLiveMuxer;
}

namespace vpx_streamer {

class StreamMuxerDelegate {
public:
  virtual void onChunkReady(std::vector<uint8_t>& buffer) = 0;
  virtual void onChunkReadyRawData(uint8_t* buffer, int32_t length) = 0;
};

class StreamMuxer : public EncoderDelegate {
public:
  StreamMuxer(StreamMuxerDelegate* delegate, bool use_vector = true);
  ~StreamMuxer();

  void configure(const EncoderConfig& config);
  void encodeImage(
      const cv::Mat& bgr, int frameWidth, int frameHeight, bool isBgr = true);
  void connect();
  void disconnect();

private:
  // EncoderDelegate implementation.
  virtual void onWriteFrame(uint8_t* buffer, uint64_t size,
                            uint64_t timeStamp, bool isKeyFrame);

  StreamMuxerDelegate* delegate_;
  boost::shared_ptr<webm_tools::WebMLiveMuxer> muxer_;
  CodecFactory codec_factory_;
  boost::shared_ptr<Encoder> encoder_;
  bool wrap_chunk_in_vector_;
};

} // namespace vpx_streamer

#endif // STREAM_MUXER_H
