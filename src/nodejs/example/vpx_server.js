// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved.

'use strict';

let os = require('os');
let fs = require('fs');
let express = require('express');
let app = express();
let server = require('http').createServer(app);
let WsServer = require('ws').Server;
let ptModule = require('node-person');
let vpx = require('../../..');

let ptConfig = {tracking: {enable: true, trackingMode: 'following'}};
let cameraConfig = {color: {width: 320, height: 240, frameRate: 30, isEnabled: true},
                    depth: {width: 320, height: 240, frameRate: 30, isEnabled: true}};
let encoderOptions = {
  bitRate: 1024,
  keyFrameForcedInterval: 4
};

let pt;
let encoder = null;
let encodedData = [];
ptModule.createPersonTracker(ptConfig, cameraConfig).then((instance) => {
  pt = instance;
  console.log('Enabling Tracking with mode set to 0');
  return vpx.createEncoder(encoderOptions);
}).then((vpxEncoder) => {
  encoder = vpxEncoder;
  encoder.on('video_data', function(data) {
    console.log('--- encoded data:', 'len:', data.length, data);
    sendEncodedData(data);
  });  
  startServer();
  return pt.start();
}).catch((error) => {
  console.log('error: ' + error);
});

console.log('\n-------- Press Esc key to exit --------\n');

const ESC_KEY = '\u001b';
const CTRL_C = '\u0003';
let stdin = process.stdin;
stdin.setRawMode(true);
stdin.resume();
stdin.setEncoding('utf8');
stdin.on('data', function(key) {
  if (key === ESC_KEY || key === CTRL_C) {
    exit();
  }
});

function exit() {
  console.log('\n-------- Stopping --------');
  if (pt) {
    pt.stop().then(() => {
      process.exit();
    }).catch((error) => {
      console.log('error: ' + error);
      process.exit();
    });
  } else {
    process.exit();
  }
}

let lastPersonCount = 0;
let totalPersonIncrements = 0;
let prevPeopleInFrame = 0;
let prevPeopleTotal = 0;
let idSet = null;
let w = 25;

function printPersonCount(result) {
  let persons = result.persons;
  let numPeopleInFrame = 0;
  let newIdSet = new Set();

  persons.forEach(function(person) {
    newIdSet.add(person.trackInfo.id);
  });

  numPeopleInFrame = persons.length;

  if (numPeopleInFrame > lastPersonCount)
    totalPersonIncrements += (numPeopleInFrame - lastPersonCount);
  else if (numPeopleInFrame === lastPersonCount && idSet !== null) {
    let diff = new Set(Array.from(idSet).filter((x) => !newIdSet.has(x)));
    totalPersonIncrements += diff.size;
  }

  idSet = newIdSet;

  lastPersonCount = numPeopleInFrame;

  if (numPeopleInFrame !== prevPeopleInFrame || totalPersonIncrements !== prevPeopleTotal) {
    console.log(padding('Current Frame Total', w), padding('Cumulative', w));
    console.log(padding('--------------------', w), padding('----------', w));
    console.log(padding(numPeopleInFrame, w), padding(totalPersonIncrements, w));

    prevPeopleInFrame = numPeopleInFrame;
    prevPeopleTotal = totalPersonIncrements;
  }
}

function padding(string, width) {
  if (!(string instanceof String))
    string = String(string);
  let length = width - string.length;
  if (length <= 0) return string;
  return string + new Array(length + 1).join(' ');
}

function sendTrackingData(result) {
  if (!connected) {
    return;
  }
  let persons = result.persons;
  let resultArray = [];
  persons.forEach(function(person) {
    let trackInfo = person.trackInfo;
    if (trackInfo) {
      let element = {};
      let boundingBox = trackInfo.boundingBox;
      let center = trackInfo.center;
      element.pid = trackInfo.id;
      element.cumulative_total = totalPersonIncrements;
      if (boundingBox) {
        element.person_bounding_box = {
          x: boundingBox.rect.x,
          y: boundingBox.rect.y,
          w: boundingBox.rect.width,
          h: boundingBox.rect.height,
        };
      }
      if (center) {
        element.center_mass_image = {
          x: center.imageCoordinate.x,
          y: center.imageCoordinate.x,
        };
        element.center_mass_world = {
          x: center.worldCoordinate.x,
          y: center.worldCoordinate.y,
          z: center.worldCoordinate.z,
        };
      }
      resultArray.push(element);
    }
  });
  let resultToDisplay = {
    Object_result: resultArray,
    type: 'person_tracking',
  };
  sendData(JSON.stringify(resultToDisplay));
}

function getTotalEncodedData() {
  Buffer.concate(encodedData);
}

let clients = [];
let connected = false;

function sendEncodedData(data) {
  sendData(data);
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
  pt.on('frameprocessed', function(result) {
    pt.getFrameData().then((frame) => {
      let time = new Date();
      console.log('encode start: time:', time.getTime(), '--', time.toTimeString());
      encoder.encode(frame.color.data, frame.color.width, frame.color.height);
    });
  });
}
