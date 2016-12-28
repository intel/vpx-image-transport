^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package vpx image transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2016-12-28)
------------------
* Added CHANGELOG.rst
* Added .gitignore
* Cleanup package.xml
  Migrated package.xml format from version 1 to 2.
* Enable sample again
  Changed the SHA of rosnodejs to a working revision. Also reworked
  the sample application logic.
* Enable hardware accelerated Yami Encoder/Decoder.
  Detects hardware capabilities first, if it supports hardware encoding
  or decoding, will create yami encoder / decoder, otherwise will
  fallback to create VPX encoder / decoder which is software based
  solution.
* Integrate libyami into project
* Moved codec related codes all into decodeFrame.
  This is the preparation for adding libyami as a replacement for
  libvpx if hardware supports it.
* Added support for 8UC1
  Which is mainly for IR.
* Updated webm-tools patch
* Implemented Subscriber
  Parse webm container on the fly, and decoded the image out of the
  vpx encoded frames.
* Remove unnecessary configuration.
* Initialized vpx encoder based on the message info.
  Initialize vpx encoder and muxer video track until we get the image
  dimension from image message. This is to fit in the different sizes
  for each topic.
* Convert 16UC1 depth data into BGR for encoding.
* Removed unnecessary bgr padded convertion
* Use vpx image wrap instead of vpx image alloc
  The new api can leverage the opencv data structure without another
  time image copy, which can boost the performance.
* Retriving webm-tools from official repo
  Also apply patch to add our local fix for webm-tools and libwebm.
* Change the project name due to PDT naming review
* Imported project contents.
