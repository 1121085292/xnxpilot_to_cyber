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

#include "xnxpilot/selfdrive/camerad/camerad_component.h"

bool CameradComponent::Init() {
  // selfdrive::camerad::CameraConf camera_conf;
  // if (!GetProtoConfig(&camera_conf)) {
  //   return false;
  // }
  camera_conf_ = std::make_shared<CameraConf>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_conf_.get())) {
    return false;
  }
  AINFO << "Camera config: " << camera_conf_->DebugString();

  // camera name
  camera_name_ = camera_conf_->camera_name();
  if(!isSensorExist(camera_name_)){
    AERROR << "camera name not support!";
    return apollo::cyber::FAIL;
  }
  //  openpilot camera frame info
  camera_state = &cameras.road_cam;
  CameraInfo camera_info = camera_state->ci; 
  camera_info.frame_width = camera_conf_->frame_width();
  camera_info.frame_height = camera_conf_->frame_height();
  camera_info.bayer = camera_conf_->bayer();
  camera_info.bayer_flip = camera_conf_->bayer_flip();
  // frame size = frame height * frame stride
  if (camera_conf_->format() == YUY2 || camera_conf_->fprmat() == RGB) {
    camera_info.frame_size = camera_info.frame_width * camera_info.frame_height * 3;
  } else if (camera_conf_->format() == YUYV) {
    camera_info.frame_size = camera_info.frame_width * camera_info.frame_height * 2;
  }
  if (camera_info->frame_size > MAX_IMAGE_SIZE) {
    AERROR << "image size is too big ,must less than " << MAX_IMAGE_SIZE
            << " bytes.";
    return false;
  }
  //  display info
  sensor_mode_ = camera_conf_->sensor_mode();
  sensor_id_ = camera_conf_->sensor_id();
  flip_method_ = camera_conf_->flip_method();
  format_ = camera_conf_->format();
  display_width_ = camera_conf_->dispaly_width();
  display_height_ = camera_conf_->dispaly_height();
  frame_rate_ = camera_conf_->frame_rate();
  fps_ = camera_conf_->fps();
  // camera init
  // device_id = cl_get_device_id(CL_DEVICE_TYPE_GPU);
  device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));

  camera_state->buf.init(device_id, context, camera_state, &vipc_server, buffer_size_, 
                    VisionStreamType::VISION_STREAM_RGB_BACK, VisionStreamType::VISION_STREAM_YUV_BACK);

  camera_writer_ = node_->CreateWriter<FrameDara>(camera_conf__->camera_channel_name());
  thumbnail_writer_ = node_->CreateWriter<FrameDara>(camera_conf__->thumbnail_channel_name());

  async_result_ = cyber::Async(&CameradComponent::run, this);
  res_ = cyber::Async(&CameradComponent::processing_thread, this);
  return true;
}

void CameradComponent::run() {
    running_.exchange(true);
    // poll camera image data
    vipc_server.Listener();

    road_camera_thread(camera_state);

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
    return false;
  }
  camera_state->camera_num = it->second;
  return true;
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
  res_ = cyber::Async(&CameradComponent::processing_thread, this);
}

void CameradComponent::run_camera(CameraState *s, cv::VideoCapture &video_cap, float *ts) {
  assert(video_cap.isOpened());

  cv::Size size(s->ci.frame_width, s->ci.frame_height);
  const cv::Mat transform = cv::Mat(3, 3, CV_32F, ts);
  uint32_t frame_id = 0;
  size_t buf_idx = 0;

  while (!cyber::IsShutdown()) {
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
  while (!cyber::IsShutdown()) {
    if (!cs->buf.acquire()) continue;
      const CameraBuf *b = &cs->buf;
      auto out_msg = std::make_shared<FrameData>();
      fill_frame_data(b->cur_frame_data, out_msg);
      out_msg->set_Image(kj::arrayPtr((const uint8_t *)b->cur_yuv_buf->addr, b->cur_yuv_buf->len));
      out_msg->set_Transform(b->yuv_transform.v);
      // s->pm->send("roadCameraState", msg);
      frame_date_writer_->Write(out_msg);
    if (cs == &(cameras->road_cam) && thumbnail_writer_ && cnt % 100 == 3) {
      // this takes 10ms???
      publish_thumbnail(cameras->pm, &(cs->buf));
    }
    // cs->buf.release();
    ++cnt;
  }

  return NULL;
}

void CameradComponent::fill_frame_data(const FrameMetadata &frame_data, std::shared_ptr<FrameData> &out_msg)
{
  out_msg->set_frameid(frame_data.frame_id);
  out_msg->set_timestampeof(frame_data.timestamp_eof);
  out_msg->set_timestampsof(frame_data.timestamp_sof);
  out_msg->set_framelength(frame_data.frame_length);
  out_msg->set_integlines(frame_data.integ_lines);
  out_msg->set_gain(frame_data.gain);
  out_msg->set_highconversiongain(frame_data.high_conversion_gain);
  out_msg->set_measuredgreyfraction(frame_data.measured_grey_fraction);
  out_msg->set_targetgreyfraction(frame_data.target_grey_fraction);
  out_msg->set_lenspos(frame_data.lens_pos);
  out_msg->set_lenssag(frame_data.lens_sag);
  out_msg->set_lenserr(frame_data.lens_err);
  out_msg->set_lenstruepos(frame_data.lens_true_pos);
}
