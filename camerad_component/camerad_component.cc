/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "camerad_component.h"

bool CameradComponent::Init() {
  camera_conf_ = std::make_shared<CameraConf>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_conf_.get())) {
    return false;
  }
  AINFO << "Camera config: " << camera_conf_->DebugString();

  camera_writer_ = node_->CreateWriter<FrameData>(camera_conf_->camera_channel_name());
  thumbnail_writer_ = node_->CreateWriter<Thumbnail>(camera_conf_->thumbnail_channel_name());
 
  // camera init
  device_id = cl_get_device_id(CL_DEVICE_TYPE_GPU);
  // device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));

  vipc_server.setVisionIpcServer("camerad", device_id, context);

  async_result_ = apollo::cyber::Async(&CameradComponent::run, this);
  return true;
}

void CameradComponent::run() {
    running_.exchange(true);
    cameras_init(&vipc_server, &cameras, device_id, context);
    cameras_open(&cameras);
    vipc_server.start_listener();
    cameras_run(&cameras, cap_road, thumbnail_writer_, camera_writer_);
}

CameradComponent::~CameradComponent() {
  CL_CHECK(clReleaseContext(context));
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

// std::string CameradComponent::gstreamer_pipeline(std::string sensor_mode, std::string sensor_id, std::string flip_method, 
//                                 int display_width, int display_height, Format format, int framerate) {
//     // sensor mode 1 = 1920 x 1080
//     // "v4l2src device=/dev/video0 ! video/x-raw, width=(int)1920, height=(int)1536, format=(string)YUY2, framerate=(fraction)30/1 ! appsink"
//     return sensor_mode + " device=" + sensor_id +
//            " ! video/" + flip_method +
//            ", width=(int)" + std::to_string(display_width) +
//            ", height=(int)" + std::to_string(display_height) +
//            ", format=(string)" + formatToString[format] +
//            ", framerate=(fraction)" + std::to_string(framerate) +
//            "/1 ! appsink";
// }

