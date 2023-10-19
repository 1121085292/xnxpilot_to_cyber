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

#include "selfdrive/camerad/camerad_component.h"

bool CameradComponent::Init() {
  selfdrive::camerad::CameraConf camera_conf;
  if (!GetProtoConfig(&camera_conf)) {
    return false;
  }
  AINFO << "Camera config: " << camera_conf.DebugString();
  // camera name
  camera_name_ = camera_conf.camera_name();
  if(!isSensorExist(camera_id_)){
    AEEOR << "camera name not support!";
    return cyber::FAIL;
  }
  //  openpilot camera frame info 
  camera_info_->frame_width = camera_conf.frame_width();
  camera_info_->frame_height = camera_conf.frame_height();
  camera_info_->frame_stride = camera_conf.frame_stride();
  camera_info_->bayer = camera_conf.bayer();
  camera_info_->bayer_flip = camera_conf.bayer_flip();
  //  display info
  sensor_mode_ = camera_conf.sensor_mode();
  sensor_id_ = camera_conf.sensor_id();
  flip_method_ = camera_conf.flip_method();
  format_ = camera_conf.format();
  display_width_ = camera_conf.dispaly_width();
  display_height_ = camera_conf.dispaly_height();
  frame_rate_ = camera_conf.frame_rate();
  fps_ = camera_conf.fps();
  // camera init
  device_id = cl_get_device_id(CL_DEVICE_TYPE_GPU);
  context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));
  VisionIpcServer vipc_server("camerad", device_id, context);
  cameras_init(&vipc_server, &cameras, kCameraName2CameraId[camera_name_], 
                fps_, device_id, context);
  


  for (int i = 0; i < buffer_size_; ++i) {
    auto pb_image = std::make_shared<Image>();
    pb_image->mutable_header()->set_frame_id(camera_config_->frame_id());
    pb_image->set_width(raw_image_->width);
    pb_image->set_height(raw_image_->height);
    pb_image->mutable_data()->reserve(raw_image_->image_size);

    if (camera_config_->output_type() == YUYV) {
      pb_image->set_encoding("yuyv");
      pb_image->set_step(2 * raw_image_->width);
    } else if (camera_config_->output_type() == RGB) {
      pb_image->set_encoding("rgb8");
      pb_image->set_step(3 * raw_image_->width);
    }

    pb_image_buffer_.push_back(pb_image);
  }

  writer_ = node_->CreateWriter<FrameDara>(camera_conf_.channel_name());
  async_result_ = cyber::Async(&CameradComponent::run, this);
  return true;
}

void CameradComponent::run() {
  running_.exchange(true);
  while (!cyber::IsShutdown()) {
    if (!camera_device_->wait_for_device()) {
      // sleep for next check
      cyber::SleepFor(std::chrono::milliseconds(device_wait_));
      continue;
    }

    if (!camera_device_->poll(raw_image_)) {
      AERROR << "camera device poll failed";
      continue;
    }

    cyber::Time image_time(raw_image_->tv_sec, 1000 * raw_image_->tv_usec);
    if (index_ >= buffer_size_) {
      index_ = 0;
    }
    auto pb_image = pb_image_buffer_.at(index_++);
    pb_image->mutable_header()->set_timestamp_sec(
        cyber::Time::Now().ToSecond());
    pb_image->set_measurement_time(image_time.ToSecond());
    pb_image->set_data(raw_image_->image, raw_image_->image_size);
    writer_->Write(pb_image);

    cyber::SleepFor(std::chrono::microseconds(spin_rate_));
  }
}

CameradComponent::~CameradComponent() {
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
  }
}

std::string CameradComponent::gstreamer_pipeline(std::string sensor_mode, std::string sensor_id, std::string flip_method, 
                                int display_width, int display_height, std::string format, int framerate) {
    // sensor mode 1 = 1920 x 1080
    // "v4l2src device=/dev/video0 ! video/x-raw, width=(int)1920, height=(int)1536, format=(string)YUY2, framerate=(fraction)30/1 ! appsink"
    return sensor_mode + " device=" + sensor_id +
           " ! video/" + flip_method +
           ", width=(int)" + std::to_string(display_width) +
           ", height=(int)" + std::to_string(display_height) +
           ", format=(string)" + format +
           ", framerate=(fraction)" + std::string(framerate) +
           "/1 ! appsink";
}

bool CameradComponent::isSensorExist(const std::string& name) const {
  return kCameraName2CameraId.find(name) != kCameraName2CameraId.end();
}