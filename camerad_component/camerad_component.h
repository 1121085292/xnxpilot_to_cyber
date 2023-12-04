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

#include "perception/camerad_component/cameras/camera_common.h"
#include "common_msgs/camerad/frame_data.pb.h"
#include "perception/camerad_component/proto/camera_conf.pb.h"

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using camerad_component::proto::CameraConf;
using camerad_component::proto::Format;

using common_msgs::camerad::FrameData;
using common_msgs::camerad::Thumbnail;

std::unordered_map<Format, std::string> formatToString = {
  {Format::YUY2, "YUY2"},
  {Format::RGB, "RGB"},
  {Format::YUYV, "YUYV"},
};

class CameradComponent : public Component<> {
 public:
  bool Init() override;
  ~CameradComponent();
  bool isSensorExist(const std::string& name) const;
  std::string gstreamer_pipeline(std::string sensor_mode, std::string sensor_id, std::string flip_method, 
                                int display_width, int display_height, Format format, int framerate);
 
 private:
  void run();
  void processing_thread();
  void road_camera_thread(std::shared_ptr<CameraState>& s);
  void run_camera(std::shared_ptr<CameraState>& s, cv::VideoCapture &video_cap, float *ts);
  void fill_frame_data(const FrameMetadata &frame_data, std::shared_ptr<FrameData>& out_msg);

  std::shared_ptr<Writer<FrameData>> camera_writer_ = nullptr;
  std::shared_ptr<Writer<Thumbnail>> thumbnail_writer_ = nullptr;

  cl_device_id device_id;
  cl_context context;
  VisionIpcServer vipc_server;
  std::shared_ptr<CameraState> road_cam;
  std::shared_ptr<CameraConf> camera_conf_;
  std::string camera_name_;
  std::string sensor_mode_;
  std::string sensor_id_;
  std::string flip_method_;
  Format format_;
  int display_width_;
  int display_height_;
  int frame_rate_;
  int fps_;

  // int index_ = 0;
  int buffer_size_ = 16;
  const int32_t MAX_IMAGE_SIZE = 20 * 1024 * 1024;
  std::future<void> async_result_;
  std::future<void> res_;
  std::future<void> lister_;
  std::atomic<bool> running_ = {false};
};

CYBER_REGISTER_COMPONENT(CameradComponent)
