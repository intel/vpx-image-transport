// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "utils.h"

#include <iostream>

void DebugInfoMessage() {
  std::cout << std::endl << std::flush;
}

void DebugErrorMessage() {
  std::cerr << std::endl << std::flush;
}

v8::Local<v8::Promise> EncoderUtils::CreateRejectedPromise(
    std::string reason) {
  PromiseHelper helper;
  v8::Local<v8::Promise> promise = helper.CreatePromise();
  helper.RejectPromise(reason);
  return promise;
}