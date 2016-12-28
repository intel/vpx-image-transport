'use strict';

const rosnodejs = require('rosnodejs');
const vpx_msgs = rosnodejs.require('vpx_image_transport');

const VPXMsg = vpx_msgs.msg.Packet;

rosnodejs.initNode('/realsense_display').then((rosNode) => {
  console.log(`ros node created:${rosNode.toString()}`);

  let displayVideo = document.getElementById('displayVideo');

  // VP8 video stream setup
  const mimeType = 'video/webm; codecs=vp8';
  if (!MediaSource.isTypeSupported(mimeType)) {
    console.error(`mimetype:${mimeType} is not supported.`);
    return;
  }

  let ms = new MediaSource();
  displayVideo.src = window.URL.createObjectURL(ms);

  let sourceBuffer = null;
  ms.addEventListener('sourceopen', (e) => {
    sourceBuffer = ms.addSourceBuffer(mimeType);
  }, false);
  ms.addEventListener('sourceended', (e) => {
    ms.endOfStream();
    displayVideo.play();
  }, false);

  rosNode.subscribe('/camera/color/image_raw/vpx', VPXMsg, (data) => {
    if (sourceBuffer == null) {
      return;
    }
    sourceBuffer.appendBuffer(new Uint8Array(data.data));
  });
});
