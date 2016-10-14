VPX Stream Displayer
============

A JavaScript written ROS node to display VPX/WebM encoded data in HTML5 video element.

# Installation

This sample is a NPM based JavaScript application which needs to run in the [NW.js](https://nwjs.io) environment. So it means that you need to install the Node.js environment and NPM dependencies first, and then install the NW.js binaries to run the application.

## Node.js

As this sample uses JavaScript ES6 features, we recommend to install the Node.js v6.0+. You can easily find the Node.js runtime downloader [here](https://nodejs.org/en/download/). And if you are using Ubuntu, it is more recommended to install Node.js through the Package Manager `apt-get`, and you can follow the instruction [here](https://nodejs.org/en/download/package-manager/#debian-and-ubuntu-based-linux-distributions). The command lines for quick reference are as below:

```bash
curl -sL https://deb.nodesource.com/setup_6.x | sudo -E bash -
sudo apt-get install -y nodejs
```

## NW.js

[NW.js](http://nwjs.io/) is a Chromium based web application runtime which mix up the DOM environment and Node.js environment. It makes you to access Node.js modules within the DOM JavaScript context.

To run the sample, you need to download NW.js from [here](http://nwjs.io/downloads/), and we recommend to use the vertion above v0.16. We need Normal version only, choose the right architecture and download to your local drive. Then unzip/untar it to somewhere you like, and add the `path/to/nwjs` to your `PATH` environment variable.

## NPM Dependencies

```bash
cd sample;
npm install
```

Then wait for the NPM installation complete. It will install all the NPM dependencies for you.

# Run

This application subscribes the `/camera/color/image_raw/vpx` topic with message type `vpx_image_transport/Packet`, so you need to run the ROS node which publishes the topic first (e.g. `roslaunch realsense_camera r200_nodelet_default.launch`), then start the appligation by NW.js:

```bash
cd sample
nw .
```

