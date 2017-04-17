// Copyright (c) 2016 Intel Corporation. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be
// found in the LICENSE file.

'use strict';

let vpx = require('../../..');
let pt = require('node-person');

let encoderOptions = {
  bitRate: 300000,
  keyFrameForceInterval: 10
};

let trackerOptions = {
  skeleton: {
    enable: true,
  },
  tracking: {
    enable: true,
    enableHeadBoundingBox: true,
  },
  recognition: {
    enable: true,
    policy: 'standard',
  },
  gesture: {
    enable: true,
    enableAllGestures: true,
  },
  personFace: {
    enableFaceLandmarks: true,
    enableHeadPose: true,
  },
};

let cameraOptions = {
  color: {
    width: 640,
    height: 480,
    frameRate: 30,
    isEnabled: true,
  },
  depth: {
    width: 320,
    height: 240,
    frameRate: 30,
    isEnabled: true,
  },
};

let encoder = null;
let tracker = null;
pt.createPersonTracker(trackerOptions, cameraOptions).then((personTracker) => {
  tracker = personTracker;
  return vpx.createEncoder();
}).then((vpxEncoder) => {
  encoder = vpxEncoder;
  operate('start', 1000, 'Start');
}).catch((e) => {
  console.log('Failed to start, ', e);
});

function registerTestWork() {
  encoder.on('video_data', function(data) {
    console.log('encoded data:', 'len:', data.length, data);
  })
  tracker.on('frameprocessed', function(result) {
    tracker.getFrameData().then((frame) => {
      console.log('data before encode:', 'len:', frame.color.data.length, frame.color.data);
      encoder.encode(frame.color.data, frame.color.width, frame.color.height);
    });
  });
}

function operate(op, time, des) {
  setTimeout(function() {
    console.log('------------', des);
  }, time);
  // eslint-disable-next-line
  let theArgs = arguments;
  setTimeout(function() {
    if (op === 'start') {
      registerTestWork();
    }
    // eslint-disable-next-line
    tracker[op].apply(tracker, Array.prototype.slice.call(theArgs, 3)).then(function() {
      console.log('------------ ', op, ' done');
    }).catch(function(e) {
      console.log('------------', op, ' failed');
    });
  }, time + 500);
}
