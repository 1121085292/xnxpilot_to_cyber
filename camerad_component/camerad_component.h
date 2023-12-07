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
#include <unordered_map>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "cyber/cyber.h"
#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"

#include "perception/camerad_component/cameras/camera_mipi.h"
#include "perception/camerad_component/proto/camera_conf.pb.h"

using camerad_component::proto::CameraConf;
using camerad_component::proto::Format;

std::unordered_map<Format, std::string> formatToString = {
  {Format::YUY2, "YUY2"},
  {Format::RGB, "RGB"},
  {Format::YUYV, "YUYV"},
};

ExitHandler do_exit;

class CameradComponent : public Component<> {
 public:
  bool Init() override;
  ~CameradComponent();

 private:
  void run();

  std::shared_ptr<Writer<FrameData>> camera_writer_ = nullptr;
  std::shared_ptr<Writer<Thumbnail>> thumbnail_writer_ = nullptr;

  cl_device_id device_id;
  cl_context context;
  VisionIpcServer vipc_server;

  MultiCameraState cameras;
  cv::VideoCapture cap_road;

  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
};
CYBER_REGISTER_COMPONENT(CameradComponent)
