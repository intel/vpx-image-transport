# VPX Image Transport

An [image_transport](http://wiki.ros.org/image_transport) plugin to tranport [sensor_msg::Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) data via [VP8/VP9](https://github.com/webmproject/libvpx) encoded data stream within the [WebM](http://www.webmproject.org) container.

## Why Choose WebM?

WebM is an open source software project which is dedicated to develop a high-quality, open video format and codecs for the web that is freely available for anyone. It maintains the open video format which is widely supported by the modern browsers (Chromium, Firefox, Edge, etc.) and web engine based application runtimes ([NW.js](http://nwjs.io), [Electron](http://electron.atom.io), etc.). The video format and the VPX encoders/decoders (including VP8, VP9 and VP10) are totally free to use without any potential proprietary license issue.

## Environment

* [Ubuntu 16.04](http://www.ubuntu.com/download/desktop)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

## Installation

### Install ROS

* Follow the ROS Kinetic instruction in [ROS Wiki](http://wiki.ros.org/ROS/Installation/Ubuntu) to install the ROS Kinetic version onto your Ubuntu.

* Then setup your ROS environment and create your ROS workspace. Follow the [instruction](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) here.

### Install libvpx

Before cloning the project, you need to install `libvpx-dev` to make it compile:

```bash
sudo apt-get install libvpx-dev
```

### Install vpx_image_transport

All you need to is to clone this repository into the `src` directory of your workspace and build it with `catkin_make`:

```bash
cd ~/catkin_ws/src
git clone <vpx_image_transport_repository_url>
cd ..
catkin_make
```

## Usage

### Publish Image with WebM/VPX Data Stream

Firstly your ROS node should publish the `sensor_msg::Image` message through the image_transport. For more details on how to write your codes to publish through image_transport, please refer to the [image transport wiki](http://wiki.ros.org/image_transport).

After you successfully hook up the `sensor_msg::Image` message with image_transport, the vpx_image_transport plugin will be loaded automatically by image_transport and the WebM/VPX encoded data stream will be published at `<image_topic_name>/vpx`.

### Subscribe to WebM/VPX Data Stream

As browser / web application runtime has the ability to decode the WebM/VPX data stream, we can feed the data stream directly into the HTML5 VideoElement by creating a MediaSource for it and append the WebM/VPX encoded data into the SourceBuffer. You can follow the instruction of the [sample](sample) here to create a NW.js application to display the Image data.

## License

vpx_image_transport project is available under the BSD license. See the [LICENSE](LICENSE) file for more information.
