#pragma once

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

#include "camera_common.h"

CameraInfo cameras_supported[CAMERA_ID_MAX] = {
  // road facing
  // [CAMERA_ID_IMX477] = {
  //     .frame_width = FRAME_WIDTH,
  //     .frame_height = FRAME_HEIGHT,
  //     .frame_stride = FRAME_WIDTH*3,
  //     .bayer = false,
  //     .bayer_flip = false,
  // },
  [CAMERA_ID_AR0233] = {
      FRAME_WIDTH_FRONT,
      FRAME_HEIGHT_FRONT,
      FRAME_WIDTH_FRONT*3,
      false,
      false,
  },
  [CAMERA_ID_ISX031] = {
      FRAME_WIDTH,
      FRAME_HEIGHT,
      FRAME_WIDTH*3,
      false,
      false,
  },
};
