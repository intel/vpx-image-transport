// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

'use strict';

let os = require('os');
let fs = require('fs');
let jpeg = require('jpeg-turbo');
let express = require('express');
let app = express();
let server = require('http').createServer(app);
let WsServer = require('ws').Server;
let addon = require('node-librealsense')
let vpx = require('../../..');

let encoderOptions = {
  keyFrameForcedInterval: 10,
  frameRate: 15,
  quality: 80
};

let pt;
let encoder = null;
let datasend = 0;
let presend = 0;
let res = [
  {
    idx: 0,
    width: 320,
    height: 240,
    rate: 30
  },
  {
    idx: 1,
    width: 640,
    height: 480,
    rate: 15
  },
  {
    idx: 2,
    width: 640,
    height: 480,
    rate: 30
  },
  {
    idx: 3,
    width: 640,
    height: 480,
    rate: 60
  },
  {
    idx: 4,
    width: 960,
    height: 540,
    rate: 15
  },
  {
    idx: 5,
    width: 1280,
    height: 720,
    rate: 15
  },
  {
    idx: 6,
    width: 1920,
    height: 1080,
    rate: 15
  },
  {
    idx: 7,
    width: 1920,
    height: 1080,
    rate: 30
  },
];

console.log('----supported resolutions:\n', res);
let resIndex = 0;
let isJpeg = false;
let printTime = false;

process.argv.forEach((val, index) => {
  if (val === 'jpg' || val === 'jpeg')
    isJpeg = true;
  if (val === 'time')
    printTime = true;
  if (!isNaN(val))
    resIndex = val;
});

if (isJpeg)
  console.log('====== JPEG encoding');
else
  console.log('====== VPX encoding');
encoderOptions.frameRate = res[resIndex].rate;
console.log('--- encoder options:', encoderOptions);

let context = new addon.Context;
let camera;
let mode = {
  width: res[resIndex].width,
  height: res[resIndex].height,
  format: 'rgb',
  framerate: res[resIndex].rate,
};

console.log('--- camera mode:', mode);
function encodeToJPEG(buffer, width, height) {
  let options = {
    format: jpeg.FORMAT_RGB,
    width: width,
    height: height,
    quality: 80,
  };
  let jpegImageData = jpeg.compressSync(buffer, options);
  return jpegImageData;
}

function printStreamModes() {
  camera.getStreamModeCount('color').then((cnt) => {
    for(let i=0; i<cnt; i++) {
      camera.getStreamMode('color', i).then((mode) => {
        if (mode.format === 'rgb8')
          console.log('Camera supported Mode: ', mode);
    });
    }
  });
}

context.getDevice(0).then((device) => {
  camera = device;
  printStreamModes();
  device.enableStream('color', mode).then(() => {
    device.start().catch((e) => {
      console.log(e);
    });
  });

  return vpx.createEncoder(encoderOptions);
}).then((vpxEncoder) => {
  encoder = vpxEncoder;
  encoder.on('video_data', function(data) {
    if (printTime) {
      let time = new Date();
      console.log('encode finish time:', time.getTime(), '--size:', data.length);
    }
    sendEncodedData(data);
  });
  startServer();
}).catch((error) => {
  console.log('error: ' + error);
});


let clients = [];
let connected = false;
let firstTime = true;
let frameSkipInterval = 1;
let counter = 0;
function sendEncodedData(data) {
  if (firstTime) {
    sendData(JSON.stringify({w: res[resIndex].width, h: res[resIndex].height, fps: res[resIndex].rate}));
    firstTime = false;
  }
  if ((counter % frameSkipInterval) === 0) {
    datasend += data.length;
    sendData(data);
  }
  counter ++;
}

function sendData(data) {
  if (clients.length !== 0) {
    try {
      clients.forEach((client) => {
        client.send(data);
      });
    } catch (exception) {
      console.log('Exception: send data failed exception:', exception);
    }
  }
}

function getEthernetIp() {
  let ifaces = os.networkInterfaces();
  let ip = '';
  for (let ifname in ifaces) {
    if (ifname === undefined)
      continue;
    ifaces[ifname].forEach(function(iface) {
      if ('IPv4' !== iface.family || iface.internal !== false) {
        return;
      }
      ip = iface.address;
    });
    if (ip !== '')
      return ip;
  }
  return ip;
}
function startServer() {
  app.use(express.static('.', {fallthrough:true}));
  const ip = getEthernetIp();
  const port = 8000;
  server.listen(port, ip);
  let wss = new WsServer({
    server: server,
  });

  console.log('\nEthernet ip:' + ip);
  console.log(' >>> point your browser to: http://' + ip + ':' + port + '/index.html');

  wss.on('connection', function(client) {
    console.log('server: got connection ' + client._socket.remoteAddress + ':' +
        client._socket.remotePort);
    clients.push(client);
    if (!connected)
      connected = true;
    client.on('close', function() {
      console.log('server: disconnect ' + client._socket.remoteAddress + ':' +
          client._socket.remotePort);
      let index = clients.indexOf(client);
      if (index > -1) {
        clients.splice(index, 1);
      }
      if (clients.length === 0)
        connected = false;
    });
    startStreamingEncodedData();
  });
}
function startStreamingEncodedData() {
  camera.on('frameready', function(frame) {
    if (printTime) {
      let time = new Date();
      console.log('encode start time:', time.getTime());
    }
    if (isJpeg) {
      let imageBuffer;
      imageBuffer = encodeToJPEG(frame.data, res[resIndex].width, res[resIndex].height);
      if (printTime) {
        let time = new Date();
        console.log('encode finis time:', time.getTime(), '--size:', imageBuffer.length);
      }

      sendEncodedData(imageBuffer);
    } else {
      encoder.encode(frame.data, res[resIndex].width, res[resIndex].height);
    }
  });
}

setInterval(function() {
  let bitrate = (datasend - presend)/1024;
  console.log('------------ bitrate (KB):', bitrate);
  presend = datasend;
},1000)
