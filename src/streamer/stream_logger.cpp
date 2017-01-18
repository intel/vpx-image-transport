// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "stream_logger.h"

#include <sstream>
#include <cstdlib>
#include <memory>
#include <cstring>
#include <stdexcept>

namespace vpx_streamer {

namespace logger {

#define COLOR_NORMAL "\033[0m"
#define COLOR_RED "\033[31m"
#define COLOR_GREEN "\033[32m"
#define COLOR_YELLOW "\033[33m"

void print(Level level, const char* file, int line, const char* format, ...) {
  const char* color = NULL;
  FILE* f = stdout;

  switch (level) {
    case Fatal:
    case Error:
      color = COLOR_RED;
      f = stderr;
      break;
    case Warn:
      color = COLOR_YELLOW;
      break;
    case Info:
      color = COLOR_NORMAL;
      break;
    case Debug:
      color = COLOR_GREEN;
      break;
    default:
      color = COLOR_NORMAL;
  }

  std::stringstream ss;
  ss << color;

  va_list arg_list;
  va_start(arg_list, format);
  // TODO: print out to string stream object.
  va_end(arg_list);

  ss << COLOR_NORMAL;
}

} // namespace logger

} // namespace vpx_streamer
