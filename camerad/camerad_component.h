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

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "selfdrive/camerad/cameras/camera_mipi.h"

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using selfdrive::camerad;

class CameradComponent : public Component<> {
 public:
  bool Init() override;
  ~CameradComponent();

 private:
  void run();
  void road_camera_thread(CameraState* s);
  void run_camera(CameraState *s, cv::VideoCapture &video_cap, float *ts);
  void processing_thread(MultiCameraState *cameras, CameraState *cs);
  void fill_frame_data(const FrameMetadata &frame_data, std::shared_ptr<FrameData>& out_msg);

  std::shared_ptr<Writer<FrameData>> camera_writer_ = nullptr;
  std::shared_ptr<Writer<Thumbnail>> thumbnail_writer_ = nullptr;

//   VisionIpcServer vipc_server;
  cl_device_id device_id;
  cl_context context;
  CameraState* camera_state;
  const CameraInfo* camera_info
  MultiCameraState* cameras = nullptr;
  std::shared_ptr<CameraConf> camera_conf_;
  // CameraDisplayInfo camera_display_info_;
  std::string camera_name_;
  std::string sensor_mode_;
  std::string sensor_id_;
  std::string flip_method_;
  camerad::Format format_;
  int display_width_;
  int display_height_;
  int frame_rate_;
  int fps_;

  std::shared_ptr<CameraBuf> camera_buf_;
  int index_ = 0;
  int buffer_size_ = 16;
  const int32_t MAX_IMAGE_SIZE = 20 * 1024 * 1024;
  std::future<void> async_result_;
  std::future<void> res_;
  std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(CameradComponent)
