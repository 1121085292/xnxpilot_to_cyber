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
  // road_cam = std::make_shared<CameraState>();
  // road_cam->camera_type = CameraType::RoadCam;

  // camera name
  camera_name_ = camera_conf_->camera_name();     //  AR0233
  // if(!isSensorExist(camera_name_)){
  //   AERROR << "camera name not support!";
  //   return apollo::cyber::FAIL;
  // }

  //  openpilot camera frame info
  // road_cam->ci.frame_width = camera_conf_->frame_width();     //  1164
  // road_cam->ci.frame_height = camera_conf_->frame_height();   //  874
  // road_cam->ci.bayer = camera_conf_->bayer();                 //  false
  // road_cam->ci.bayer_flip = camera_conf_->bayer_flip();       //  false
  // road_cam->ci.frame_stride = camera_conf_->frame_width() * 3;
  // // frame size = frame height * frame stride
  // if (camera_conf_->format() == Format::YUY2 || camera_conf_->format() == Format::RGB) {
  //   road_cam->ci.frame_size = road_cam->ci.frame_width * road_cam->ci.frame_height * 3;
  // } else if (camera_conf_->format() == Format::YUYV) {
  //   road_cam->ci.frame_size = road_cam->ci.frame_width * road_cam->ci.frame_height * 2;
  // }
  // if (road_cam->ci.frame_size > MAX_IMAGE_SIZE) {
  //   AERROR << "image size is too big ,must less than " << MAX_IMAGE_SIZE
  //           << " bytes.";
  //   return false;
  // }

  //  display info
  // sensor_mode_ = camera_conf_->sensor_mode();         //  v4l2src
  // sensor_id_ = camera_conf_->sensor_id();             //  /dev/video0
  // flip_method_ = camera_conf_->flip_method();         //  x-raw
  // format_ = camera_conf_->format();                   //  Format::YUV2
  // display_width_ = camera_conf_->display_width();     //  1536
  // display_height_ = camera_conf_->display_height();   //  1920
  // frame_rate_ = camera_conf_->frame_rate();           //  30
  // fps_ = camera_conf_->fps();                         //  20

  // camera_writer_ = node_->CreateWriter<FrameData>(camera_conf_->camera_channel_name());
  // thumbnail_writer_ = node_->CreateWriter<Thumbnail>(camera_conf_->thumbnail_channel_name());
 
  // camera init
  device_id = cl_get_device_id(CL_DEVICE_TYPE_GPU);
  // device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  context = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));
  vipc_server = VisionIpcServer("camerad", device_id, context);
  // vipc_server.setVisionIpcServer("camerad", device_id, context);
  // road_cam->buf.init(device_id, context, road_cam, &vipc_server, buffer_size_, 
  //                   VisionStreamType::VISION_STREAM_RGB_BACK, VisionStreamType::VISION_STREAM_YUV_BACK);

  // poll camera image data
  // lister_ = apollo::cyber::Async(&VisionIpcServer::listener, &vipc_server);
  
  async_result_ = apollo::cyber::Async(&CameradComponent::run, this);
  // res_ = apollo::cyber::Async(&CameradComponent::processing_thread, this);
  return true;
}

void CameradComponent::run() {
    running_.exchange(true);

    cameras_init(&vipc_server, &cameras, device_id, context);
    cameras_open(&cameras);
    vipc_server.start_listener();
    cameras_run(&cameras);
}

CameradComponent::~CameradComponent() {
  if (running_.load()) {
    running_.exchange(false);
    // lister_.wait();
    async_result_.wait();
    // res_.wait();
  }
}

void CameradComponent::cameras_open(MultiCameraState *s) {
  camera_open(&s->road_cam, true);
  // camera_open(&s->driver_cam, true);
}

void CameradComponent::camera_open(CameraState *s, bool rear) {
  // empty
}

// void CameradComponent::camera_close(CameraState *s) {
//   // empty
// }

// void cameras_close(MultiCameraState *s) {
//   camera_close(&s->road_cam);
//   camera_close(&s->driver_cam);
//   // delete s->pm;
// }


//此处需修改为实际使用的相机参数，包括相机型号和fps
void CameradComponent::cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
  camera_init(v, &s->road_cam, CAMERA_ID_ISX031, 30, device_id, ctx,
              VISION_STREAM_RGB_BACK, VISION_STREAM_YUV_BACK);
  // camera_init(v, &s->driver_cam, CAMERA_ID_AR0233, 30, device_id, ctx,
  //              VISION_STREAM_RGB_FRONT, VISION_STREAM_YUV_FRONT);

}

void CameradComponent::camera_init(VisionIpcServer * v, CameraState *s, int camera_id, unsigned int fps, cl_device_id device_id, 
                cl_context ctx, VisionStreamType rgb_type, VisionStreamType yuv_type) {
  initializeCameraInfo();
  assert(camera_id < static_cast<int>(std::size(cameras_supported)));
  s->ci = cameras_supported[camera_id];
  assert(s->ci.frame_width != 0);

  s->camera_num = camera_id;
  s->fps = fps;
  
  s->buf.init(device_id, ctx, s, v, FRAME_BUF_COUNT, rgb_type, yuv_type);
  
}

void process_road_camera(MultiCameraState *s, CameraState *c, int cnt) {
  const CameraBuf *b = &c->buf;
  auto out_msg = std::make_shared<common_msgs::camerad::FrameData>();
  fill_frame_data(b->cur_frame_data, out_msg);
  out_msg->set_image(b->cur_yuv_buf->addr, b->cur_yuv_buf->len);
  for(int i = 0; i < 9; i++){
    out_msg->add_transform(b->yuv_transform.v[i]);
  }
  camera_writer_->Write(out_msg);
}

std::thread start_process_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback) {
  return std::thread(processing_thread, cameras, cs, callback);
//   return std::thread([this, cameras, cs, callback]() {
//     this->processing_thread(cameras, cs, callback);
// });

}

void CameradComponent::cameras_run(MultiCameraState *s) {
  std::vector<std::thread> threads;
  threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));

  // threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));

  std::thread t_rear = std::thread(road_camera_thread, &s->road_cam);
  set_thread_name("mipi_thread");

  // std::thread t_front = std::thread(driver_camera_thread, &s->driver_cam);
  // set_thread_name("mipidriver_thread");
  
  for (auto &t : threads) t.join();
  // t_front.join();
  t_rear.join();

  cameras_close(s);
}



void road_camera_thread(CameraState *s) {
  set_thread_name("mipi_road_camera_thread");

  // std::string pipeline = gstreamer_pipeline(
  //   1, // sensor mode
  //   0, // sensor id
  //   2, // flip method
  //   864, // width
  //   486); // height

  // camera-ar0233:
  // cv::VideoCapture cap_road("v4l2src device=/dev/video0 ! video/x-raw, width=(int)1920, height=(int)1080,format=(string)YUY2, framerate=(fraction)30/1 ! appsink");
  // camera-isx031:
  //cv::VideoCapture cap_road("nvv4l2camerasrc device=/dev/video0 ! 'video/x-raw(memory:NVMM),format=YUY2,width=1920,height=1536,framerate=30/1' ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1");
  cv::VideoCapture cap_road("nvv4l2camerasrc device=/dev/video0 ! video/x-raw(memory:NVMM),format=YUY2, width=1920,height=1536,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1", cv::CAP_GSTREAMER);

  // camera-ar0233:
  // float ts[9] = {0.46414363, 0.0, 124.44721004,
  //                 0.0, 0.46099291, 215.07801418,
  //                 0.0, 0.0, 1.0};
  // camera-isx031:
  float ts[9] = {0.76528467, 0.0, -151.52535531,
                  0.0, 0.76086957, -155.7173913,
                  0.0, 0.0, 1.0};
  run_camera(s, cap_road, ts);
}

void CameradComponent::run_camera(CameraState *s, cv::VideoCapture &video_cap, float *ts) {
  assert(video_cap.isOpened());

  cv::Size size(s->ci.frame_width, s->ci.frame_height);
  const cv::Mat transform = cv::Mat(3, 3, CV_32F, ts);
  uint32_t frame_id = 0;
  size_t buf_idx = 0;
  
  while (!do_exit) {
    cv::Mat YUV_mat, frame_mat, transformed_mat;
    video_cap >> frame_mat;
    // ar0233:
    // COLOR_YUV2BGR_UYVY
    // cv::cvtColor(YUV_mat, frame_mat, cv::COLOR_YUV2BGR_UYVY);
    // isx031:
    // COLOR_YUV2BGR_YUY2
    //cv::cvtColor(YUV_mat, frame_mat, cv::COLOR_YUV2BGR_YUY2);

    
    cv::warpPerspective(frame_mat, transformed_mat, transform, size, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

    s->buf.camera_bufs_metadata[buf_idx] = {.frame_id = frame_id};

    auto &buf = s->buf.camera_bufs[buf_idx];
    int transformed_size = transformed_mat.total() * transformed_mat.elemSize();

    CL_CHECK(clEnqueueWriteBuffer(buf.copy_q, buf.buf_cl, CL_TRUE, 0, transformed_size, transformed_mat.data, 0, NULL, NULL));

    s->buf.queue(buf_idx);
    ++frame_id;
    buf_idx = (buf_idx + 1) % FRAME_BUF_COUNT;
  }
}

void *processing_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback) {
  const char *thread_name = nullptr;
  if (cs == &cameras->road_cam) {
    thread_name = "RoadCamera";
  } else if (cs == &cameras->driver_cam) {
    thread_name = "DriverCamera";
  } else {
    thread_name = "WideRoadCamera";
  }
  set_thread_name(thread_name);

  uint32_t cnt = 0;
  while (!do_exit) {
    if (!cs->buf.acquire()) continue;

    callback(cameras, cs, cnt);

    if (cs == &(cameras->road_cam) && thumbnail_writer_ && cnt % 100 == 3) {
      // this takes 10ms???
      publish_thumbnail(thumbnail_writer_, &(cs->buf));
    }
    cs->buf.release();
    ++cnt;
  }

  return NULL;
}
