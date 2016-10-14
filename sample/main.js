'use strict';

const rosnodejs = require('rosnodejs');

let displayVideo = document.getElementById('displayVideo');

let mimeType = 'video/webm; codecs=vp8';
const topicConfig = {
  topic: '/camera/color/image_raw/vpx',
  message: 'vpx_image_transport/Packet'
};

let startROS = dataCallback => {
  rosnodejs.initNode('/realsense_display',{
    messages: [topicConfig.message]
  }).then((rosNode) => {
    console.log(`ros node created:${rosNode.toString()}`);

    let sub = rosNode.subscribe(topicConfig.topic, topicConfig.message, data => {
      dataCallback(data.data);
    });
  })
}

if (!MediaSource.isTypeSupported(mimeType)) {
  console.error(`mimetype:${mimeType} is not supported.`);
} else {
  let ms = new MediaSource();
  displayVideo.src = window.URL.createObjectURL(ms);

  let currentSourceBuffer = null;
  ms.addEventListener('sourceopen', (e) => {
    startROS((data) => {
      if (!currentSourceBuffer) {
        console.log("ROS::No source, create one.")
        let sourceBuffer = ms.addSourceBuffer(mimeType);
        sourceBuffer.addEventListener('updateend', (ee) => {
          if (sourceBuffer.updating) {
            console.log("still updating, exit updateend");
            return;
          }
        }, false);
        currentSourceBuffer = sourceBuffer;
      }
      currentSourceBuffer.appendBuffer(new Uint8Array(data));
    });
  }, false);
  ms.addEventListener('sourceended', (e) => {
    ms.endOfStream();
    displayVideo.play();
  }, false);
}
