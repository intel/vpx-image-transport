// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef STREAM_LOGGER_H
#define STREAM_LOGGER_H

#include <cstdio>
#include <cstdarg>

namespace vpx_streamer {

namespace logger {

enum Level {
  Debug,
  Info,
  Warn,
  Error,
  Fatal
};

void print(Level level, const char* file, int line, const char* format, ...);

} // namespace logger

#define STREAM_LOG(level, fmt, ...) \
  logger::print(level, __FILE__, __LINE__, fmt, ## __VA_ARGS__)
#define STREAM_LOG_DEBUG(fmt, ...) STREAM_LOG(logger::Debug, fmt, ## __VA_ARGS__)
#define STREAM_LOG_INFO(fmt, ...) STREAM_LOG(logger::Info, fmt, ## __VA_ARGS__)
#define STREAM_LOG_WARN(fmt, ...) STREAM_LOG(logger::Warn, fmt, ## __VA_ARGS__)
#define STREAM_LOG_ERROR(fmt, ...) STREAM_LOG(logger::Error, fmt, ## __VA_ARGS__)
#define STREAM_LOG_FATAL(fmt, ...) STREAM_LOG(logger::Fatal, fmt, ## __VA_ARGS__)

} // namespace vpx_streamer

#endif // STREAM_LOGGER_H
