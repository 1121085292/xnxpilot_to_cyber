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

  //camera type RoadCam
  road_cam = std::make_shared<CameraState>();
  road_cam->camera_type = CameraType::RoadCam;

  // camera name
  camera_name_ = camera_conf_->camera_name();     //  AR0233
  if(!isSensorExist(camera_name_)){
    AERROR << "camera name not support!";
    return apollo::cyber::FAIL;
  }

  //  openpilot camera frame info
  road_cam->ci.frame_width = camera_conf_->frame_width();     //  1164
  road_cam->ci.frame_height = camera_conf_->frame_height();   //  874
  road_cam->ci.bayer = camera_conf_->bayer();                 //  false
  road_cam->ci.bayer_flip = camera_conf_->bayer_flip();       //  false
  // frame size = frame height * frame stride
  if (camera_conf_->format() == Format::YUY2 || camera_conf_->format() == Format::RGB) {
    road_cam->ci.frame_size = road_cam->ci.frame_width * road_cam->ci.frame_height * 3;
  } else if (camera_conf_->format() == Format::YUYV) {
    road_cam->ci.frame_size = road_cam->ci.frame_width * road_cam->ci.frame_height * 2;
  }
  if (road_cam->ci.frame_size > MAX_IMAGE_SIZE) {
    AERROR << "image size is too big ,must less than " << MAX_IMAGE_SIZE
            << " bytes.";
    return false;
  }

  //  display info
  sensor_mode_ = camera_conf_->sensor_mode();         //  v4l2src
  sensor_id_ = camera_conf_->sensor_id();             //  /dev/video0
  flip_method_ = camera_conf_->flip_method();         //  x-raw
  format_ = camera_conf_->format();                   //  Format::YUV2
  display_width_ = camera_conf_->display_width();     //  1536
  display_height_ = camera_conf_->display_height();   //  1920
  frame_rate_ = camera_conf_->frame_rate();           //  30
  fps_ = camera_conf_->fps();                         //  20

  camera_writer_ = node_->CreateWriter<FrameData>(camera_conf_->camera_channel_name());
  thumbnail_writer_ = node_->CreateWriter<Thumbnail>(camera_conf_->thumbnail_channel_name());
 
  // camera init
  // device_id = cl_get_device_id(CL_DEVICE_TYPE_GPU);
  device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));

  VisionIpcServer vipc_server("camerad", device_id, context);

  road_cam->buf.init(device_id, context, road_cam, &vipc_server, buffer_size_, 
                    VisionStreamType::VISION_STREAM_RGB_BACK, VisionStreamType::VISION_STREAM_YUV_BACK);

  // poll camera image data
  vipc_server.Listener();
  async_result_ = apollo::cyber::Async(&CameradComponent::run, this);
  res_ = apollo::cyber::Async(&CameradComponent::processing_thread, this);
  return true;
}

void CameradComponent::run() {
    running_.exchange(true);
    road_camera_thread(road_cam);
}

CameradComponent::~CameradComponent() {
  if (running_.load()) {
    running_.exchange(false);
    async_result_.wait();
    res_.wait();
  }
}

std::string CameradComponent::gstreamer_pipeline(std::string sensor_mode, std::string sensor_id, std::string flip_method, 
                                int display_width, int display_height, Format format, int framerate) {
    // sensor mode 1 = 1920 x 1080
    // "v4l2src device=/dev/video0 ! video/x-raw, width=(int)1920, height=(int)1536, format=(string)YUY2, framerate=(fraction)30/1 ! appsink"
    return sensor_mode + " device=" + sensor_id +
           " ! video/" + flip_method +
           ", width=(int)" + std::to_string(display_width) +
           ", height=(int)" + std::to_string(display_height) +
           ", format=(string)" + std::to_string(format) +
           ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! appsink";
}

bool CameradComponent::isSensorExist(const std::string& name) const {
  auto it = kCameraName2CameraId.find(name);
  if(it != kCameraName2CameraId.end()){
    road_cam->camera_num = it->second;
    return true;
  }
  return false;
}

void CameradComponent::road_camera_thread(CameraState *s) {
  std::string pipeline = gstreamer_pipeline(
    sensor_mode_, // v4l2src
    sensor_id_, // dev/video0
    flip_method_, // x-raw
    display_width_, // 1920
    display_height_,  // 1536
    format_,  //YUY2
    frame_rate_);   //30

  cv::VideoCapture cap_road(pipeline, cv::CAP_GSTREAMER); // road
  float ts[9] = {1.50330396, 0.0, -59.40969163,
                  0.0, 1.50330396, 76.20704846,
                  0.0, 0.0, 1.0};
  run_camera(s, cap_road, ts);
}

void CameradComponent::run_camera(CameraState *s, cv::VideoCapture &video_cap, float *ts) {
  assert(video_cap.isOpened());

  cv::Size size(s->ci.frame_width, s->ci.frame_height);
  const cv::Mat transform = cv::Mat(3, 3, CV_32F, ts);
  uint32_t frame_id = 0;
  size_t buf_idx = 0;

  while (!apollo::cyber::IsShutdown()) {
    cv::Mat frame_mat, transformed_mat;
    video_cap >> frame_mat;
    cv::warpPerspective(frame_mat, transformed_mat, transform, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

    s->buf.camera_bufs_metadata[buf_idx] = {.frame_id = frame_id};

    auto &buf = s->buf.camera_bufs[buf_idx];
    int transformed_size = transformed_mat.total() * transformed_mat.elemSize();
    CL_CHECK(clEnqueueWriteBuffer(buf.copy_q, buf.buf_cl, CL_TRUE, 0, transformed_size, transformed_mat.data, 0, NULL, NULL));

    s->buf.queue(buf_idx);

    ++frame_id;
    buf_idx = (buf_idx + 1) % buffer_size_;
  }
}

void CameradComponent::processing_thread() {
  uint32_t cnt = 0;
  while (!apollo::cyber::IsShutdown()) {
    if (!road_cam->buf.acquire()) continue;
      const CameraBuf *b = &road_cam->buf;
      auto out_msg = std::make_shared<FrameData>();
      fill_frame_data(b->cur_frame_data, out_msg);
      out_msg->set_image(b->cur_yuv_buf->addr, b->cur_yuv_buf->len);
      for(int i = 0; i < 9; i++){
        out_msg->set_transform(i, b->yuv_transform.v[i]);
      }
      // s->pm->send("roadCameraState", msg);
      camera_writer_->Write(out_msg);
    if (thumbnail_writer_ && cnt % 100 == 3) {
      // this takes 10ms???
      publish_thumbnail(thumbnail_writer_, &(road_cam->buf));
    }
    // cs->buf.release();
    ++cnt;
  }
}

void CameradComponent::fill_frame_data(const FrameMetadata &frame_data, std::shared_ptr<FrameData> &out_msg)
{
  out_msg->set_frame_id(frame_data.frame_id);
  out_msg->set_timestamp_eof(frame_data.timestamp_eof);
  out_msg->set_timestamp_sof(frame_data.timestamp_sof);
  out_msg->set_frame_length(frame_data.frame_length);
  out_msg->set_integ_lines(frame_data.integ_lines);
  out_msg->set_gain(frame_data.gain);
  out_msg->set_high_conversion_gain(frame_data.high_conversion_gain);
  out_msg->set_measured_grey_fraction(frame_data.measured_grey_fraction);
  out_msg->set_target_grey_fraction(frame_data.target_grey_fraction);
  out_msg->set_lens_pos(frame_data.lens_pos);
  out_msg->set_lens_sag(frame_data.lens_sag);
  out_msg->set_lens_err(frame_data.lens_err);
  out_msg->set_lens_true_pos(frame_data.lens_true_pos);
}
