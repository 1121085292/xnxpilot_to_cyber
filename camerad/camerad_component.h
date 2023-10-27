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

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "cyber/cyber.h"


// #include "selfdrive/camerad/cameras/camera_mipi.h"
#include "xnxpilot/selfdrive/camerad/cameras/camera_common.h"
#include "xnxpilot/selfdrive/camerad/proto/camera_conf.pb.h"
#include "xnxpilot/common_msgs/camerad/frame_data.pb.h"

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using selfdrive::camerad::CameraConf;
using selfdrive::camerad::Format;

using common_msgs::camerad::FrameData;
using common_msgs::camerad::Thumbnail;

class CameradComponent : public Component<> {
 public:
  bool Init() override;
  ~CameradComponent();
  bool isSensorExist(const std::string& name) const;
  std::string CameradComponent::gstreamer_pipeline(std::string sensor_mode, std::string sensor_id, std::string flip_method, 
                                int display_width, int display_height, Format format, int framerate);
 
 private:
  void run();
  void processing_thread();
  void road_camera_thread(CameraState* s);
  void run_camera(CameraState *s, cv::VideoCapture &video_cap, float *ts);
  void fill_frame_data(const FrameMetadata &frame_data, std::shared_ptr<FrameData>& out_msg);

  std::shared_ptr<Writer<FrameData>> camera_writer_ = nullptr;
  std::shared_ptr<Writer<Thumbnail>> thumbnail_writer_ = nullptr;

//   VisionIpcServer vipc_server;
  cl_device_id device_id;
  cl_context context;
  CameraState* camera_state;
  VisionIpcServer vipc_server("camerad", device_id, context);
  // const CameraInfo* camera_info
  MultiCameraState cameras = {};

  std::shared_ptr<CameraConf> camera_conf_;

  std::string camera_name_;
  std::string sensor_mode_;
  std::string sensor_id_;
  std::string flip_method_;
  camerad::Format format_;
  int display_width_;
  int display_height_;
  int frame_rate_;
  int fps_;

  int index_ = 0;
  int buffer_size_ = 16;
  const int32_t MAX_IMAGE_SIZE = 20 * 1024 * 1024;
  std::future<void> async_result_;
  std::future<void> res_;
  std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(CameradComponent)