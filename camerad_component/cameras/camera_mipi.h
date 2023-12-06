#pragma once

#ifdef __APPLE__
#include <OpenCL/cl.h>
#else
#include <CL/cl.h>
#endif

#include "perception/camerad_component/cameras/camera_common.h"

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx);
void cameras_open(MultiCameraState *s);
void cameras_run(MultiCameraState *s, std::shared_ptr<Writer<Thumbnail>>thumbnail_writer,
                                          std::shared_ptr<Writer<FrameData>>camera_writer);
void cameras_close(MultiCameraState *s);
