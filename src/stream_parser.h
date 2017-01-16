// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef STREAM_PARSER_H
#define STREAM_PARSER_H

#include <boost/shared_ptr.hpp>
#include <mkvparser/mkvparser.h>
#include <vector>
#include <webm_file.h>
#include "codec_factory.h"
#include "decoder.h"

namespace vpx_image_transport {

class StreamParserDelegate {
public:
  virtual void onImageDecoded(const cv::Mat& bgr) = 0;
};

class StreamParser : public DecoderDelegate {
public:
  StreamParser(StreamParserDelegate* delegate);
  ~StreamParser();

  void decodeStream(const std::vector<uint8_t>& buffer);

private:
  StreamParserDelegate* delegate_;

  webm_tools::WebMFile* webm_file_;
  std::vector<uint8_t> buffer_;
  uint64_t bytes_consumed_;

  const mkvparser::VideoTrack* track_;
  const mkvparser::BlockEntry* current_block_;

  CodecFactory codec_factory_;
  boost::shared_ptr<Decoder> decoder_;

  const mkvparser::BlockEntry* retriveBlockEntry(const mkvparser::BlockEntry* current);
  void processBlockEntry(const mkvparser::BlockEntry* entry);

  // DecoderDelegate Implementation.
  virtual void onFrameDecoded(const cv::Mat& bgr);

  friend class StreamBufferManager;
};

} // namespace vpx_image_transport

#endif // STREAM_PARSER_H
