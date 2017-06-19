# Copyright (c) 2017 Intel Corporation. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.
#
{
  "targets": [
    {
      "target_name": "node-vpx",
      "variables": {
        "widl_files": [
          "src/nodejs/node-vpx.widl",
        ],
        "gen_files": [
          "<(INTERMEDIATE_DIR)/gen/nan__vpx_encoder.cpp",
          "<(INTERMEDIATE_DIR)/gen/thread_event_helper.cpp",
        ],
        "binary_dir": "<(module_root_dir)/build/$(BUILDTYPE)/",
      },
      "copies": [
        {
          'destination': '<(INTERMEDIATE_DIR)',
          'files': [
            './src/nodejs/scripts/process_idl.js',
            './src/nodejs/scripts/build_streamer.sh',
          ],
        },
      ],
      "actions": [
        {
          "action_name": "build_streamer",
          "variables": {
            "build_streamer_script": "<(INTERMEDIATE_DIR)/build_streamer.sh",
          },
          "inputs": [
            "<(build_streamer_script)",
          ],
          "outputs": [
            "<(INTERMEDIATE_DIR)/libvpx_streamer.so"
          ],
          "action": [
            "sh",
            "<(build_streamer_script)",
            "<(INTERMEDIATE_DIR)",
            "<(module_root_dir)",
            "<(binary_dir)",
            "${BUILDTYPE}"
          ],
          "message": "Build the core streamer",
        },
        {
          "action_name": "process_idl",
          "variables": {
            "process_idl_script": "<(INTERMEDIATE_DIR)/process_idl.js",
          },
          "inputs": [
            "<(process_idl_script)",
            "<@(widl_files)"
          ],
          "outputs": [
            "<@(gen_files)"
          ],
          "action": [
            "node",
            "<(process_idl_script)",
            "<(INTERMEDIATE_DIR)/gen",
            "<(widl_files)"
          ],
          "message": "Process .widl files",
        },
      ],
      "sources": [
        "src/nodejs/addon.cpp",
        "src/nodejs/utils.cpp",
        "src/nodejs/vpx_encoder.cpp",
        "src/nodejs/vpx_encoder_creator.cpp",
        "src/nodejs/vpx_encoder_delegate.cpp",
        "src/nodejs/task/async_task.cpp",
        "src/nodejs/task/async_task_runner.cpp",
        "src/nodejs/task/encoder_tasks.cpp",
        "<(INTERMEDIATE_DIR)/gen/nan__vpx_encoder.cpp",
        "<(INTERMEDIATE_DIR)/gen/thread-event-helper.cpp",
      ],
      "include_dirs": [
        "<(INTERMEDIATE_DIR)",
        "./src/nodejs",
        "./src/streamer",
        "<!(node -e \"require('nan')\")",
      ],
      "cflags!": [
        "-fno-exceptions"
      ],
      "cflags": [
        "-std=c++11"
      ],
      "cflags_cc!": [
        "-fno-exceptions"
      ],
      "libraries": [
        "-L<(binary_dir)",
        "-Wl,-rpath,<(binary_dir)",
        "-lpthread",
        "-lvpx_streamer",
      ],
      "xcode_settings": {
        "OTHER_CFLAGS": [
          "-std=c++11"
        ]
      },
      "conditions": [
        [
          "OS!=\"win\"",
          {
            "cflags+": [
              "-std=c++11"
            ],
            "cflags_c+": [
              "-std=c++11"
            ],
            "cflags_cc+": [
              "-std=c++11"
            ]
          }
        ],
        [
          "OS==\"mac\"",
          {
            "xcode_settings": {
              "OTHER_CPLUSPLUSFLAGS": [
                "-std=c++11",
                "-stdlib=libc++"
              ],
              "OTHER_LDFLAGS": [
                "-stdlib=libc++"
              ],
              "GCC_ENABLE_CPP_EXCEPTIONS": "YES",
              "MACOSX_DEPLOYMENT_TARGET": "10.8"
            }
          }
        ]
      ]
    }
  ]
}
