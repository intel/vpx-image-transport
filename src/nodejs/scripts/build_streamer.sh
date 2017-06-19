#!/bin/bash
#
# Copyright (c) 2017 Intel Corporation. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

echo "---- build_streamer.sh $@"
tmpDir="$1"
moduleDir="$2"
binaryDir="$3"
buildType="$4"
cd ${tmpDir} && mkdir -p streamer_build && cd streamer_build
cmake -DCMAKE_BUILD_TYPE=${buildType} ${moduleDir}/src/streamer/
nproc | xargs make -j
mv libvpx_streamer.so ${binaryDir}/

