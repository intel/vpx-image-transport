// Copyright (c) 2017 Intel Corporation. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

'use strict';

/* global describe, it */
const addon = require('bindings')('node-vpx');
const assert = require('assert');
const EventEmitter = require('events').EventEmitter;
function inherits(target, source) {
  // eslint-disable-next-line guard-for-in
  for (let k in source.prototype) {
    target.prototype[k] = source.prototype[k];
  }
}
inherits(addon.VPXEncoder, EventEmitter);

describe('VPX Encoder Test Suite - States', function() {
  let encoder = null;
  let options = {
    keyFrameForceInterval: 10,
    frameRate: 15,
    quality: 80
  };

  afterEach(function() {
  });

  it('Create encoder', function() {
    return new Promise(function(resolve, reject) {
      addon.createEncoder(options).then(function(inst) {
        encoder = inst;
        resolve();
      }).catch(function(e) {
        console.log(e);
        reject(e);
      });
    });
  });

  it('Config encoder', function() {
    return new Promise(function(resolve, reject) {
      addon.createEncoder().then(function(inst) {
        encoder = inst;
        return encoder.config(options);
      }).then(() => {
        resolve();
      }).catch(function(e) {
        console.log(e);
        reject(e);
      });
    });
  });

  it('Encode data', function() {
    let input = new Buffer(100);
    return new Promise(function(resolve, reject) {
      addon.createEncoder().then(function(inst) {
        encoder = inst;
        return encoder.config(options);
      }).then(() => {
        return encoder.encode(input);
      }).then(() => {
        resolve();
      }).catch(function(e) {
        console.log(e);
        reject(e);
      });
    });
  });
});
