#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wundefined-inline"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#pragma clang diagnostic pop

#include "perception/camerad_component/cameras/camera_mipi.h"
#include "perception/camerad_component/common/clutil.h"
#include "perception/camerad_component/common/swaglog.h"
#include "perception/camerad_component/common/timing.h"
#include "perception/camerad_component/common/util.h"

namespace {

CameraInfo cameras_supported[CAMERA_ID_MAX];

void initializeCameraInfo() {
    cameras_supported[CAMERA_ID_AR0233] = {
        .frame_width = FRAME_WIDTH_FRONT,
        .frame_height = FRAME_HEIGHT_FRONT,
        .frame_stride = FRAME_WIDTH_FRONT * 3,
        .bayer = false,
        .bayer_flip = false,
    };

      cameras_supported[CAMERA_ID_ISX031] = {
      FRAME_WIDTH,
      FRAME_HEIGHT,
      FRAME_WIDTH*3,
      false,
      false,
    };
}

void camera_open(CameraState *s, bool rear) {
  // empty
}

void camera_close(CameraState *s) {
  // empty
}

void camera_init(VisionIpcServer * v, CameraState *s, int camera_id, unsigned int fps, cl_device_id device_id, 
                cl_context ctx, VisionStreamType rgb_type, VisionStreamType yuv_type) {
  initializeCameraInfo();
  assert(camera_id < static_cast<int>(std::size(cameras_supported)));
  s->ci = cameras_supported[camera_id];
  assert(s->ci.frame_width != 0);

  s->camera_num = camera_id;
  s->fps = fps;
  
  s->buf.init(device_id, ctx, s, v, FRAME_BUF_COUNT, rgb_type, yuv_type);
  
}

void run_camera(CameraState *s, cv::VideoCapture &video_cap, float *ts) {
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

// static void road_camera_thread(CameraState *s, cv::VideoCapture cap_road) {
static void road_camera_thread(CameraState *s) {
  set_thread_name("mipi_road_camera_thread");

  // std::string pipeline = gstreamer_pipeline(
  //   1, // sensor mode
  //   0, // sensor id
  //   2, // flip method
  //   864, // width
  //   486); // height

  // camera-ar0233:
  cv::VideoCapture cap_road("v4l2src device=/dev/video0 ! video/x-raw, width=(int)1920, height=(int)1080,format=(string)YUY2, framerate=(fraction)30/1 ! appsink");
  // camera-isx031:
  //cv::VideoCapture cap_road("nvv4l2camerasrc device=/dev/video0 ! 'video/x-raw(memory:NVMM),format=YUY2,width=1920,height=1536,framerate=30/1' ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1");
  // cv::VideoCapture cap_road("nvv4l2camerasrc device=/dev/video0 ! video/x-raw(memory:NVMM),format=YUY2, width=1920,height=1536,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1", cv::CAP_GSTREAMER);

  // camera-ar0233:
  float ts[9] = {0.46414363, 0.0, 124.44721004,
                  0.0, 0.46099291, 215.07801418,
                  0.0, 0.0, 1.0};
  // camera-isx031:
  // float ts[9] = {0.76528467, 0.0, -151.52535531,
  //                 0.0, 0.76086957, -155.7173913,
  //                 0.0, 0.0, 1.0};
  run_camera(s, cap_road, ts);
}

}  // namespace

//此处需修改为实际使用的相机参数，包括相机型号和fps
void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
  camera_init(v, &s->road_cam, CAMERA_ID_ISX031, 30, device_id, ctx,
              VISION_STREAM_RGB_BACK, VISION_STREAM_YUV_BACK);
  // camera_init(v, &s->driver_cam, CAMERA_ID_AR0233, 30, device_id, ctx,
  //              VISION_STREAM_RGB_FRONT, VISION_STREAM_YUV_FRONT);

}

void camera_autoexposure(CameraState *s, float grey_frac) {}

void cameras_open(MultiCameraState *s) {
  camera_open(&s->road_cam, true);
  // camera_open(&s->driver_cam, true);
}

void cameras_close(MultiCameraState *s) {
  camera_close(&s->road_cam);
  camera_close(&s->driver_cam);
  // delete s->pm;
}

void process_road_camera(MultiCameraState *s, CameraState *c, int cnt, std::shared_ptr<Writer<FrameData>>camera_writer) {
  const CameraBuf *b = &c->buf;
  auto out_msg = std::make_shared<FrameData>();
  fill_frame_data(b->cur_frame_data, out_msg);
  out_msg->set_image(b->cur_yuv_buf->addr, b->cur_yuv_buf->len);
  for(int i = 0; i < 9; i++){
    out_msg->add_transform(b->yuv_transform.v[i]);
  }
  camera_writer->Write(out_msg);
}

// void cameras_run(MultiCameraState *s, cv::VideoCapture cap_road, std::shared_ptr<Writer<Thumbnail>>thumbnail_writer,
//                                                                  std::shared_ptr<Writer<FrameData>>camera_writer) {
void cameras_run(MultiCameraState *s, std::shared_ptr<Writer<Thumbnail>>thumbnail_writer, std::shared_ptr<Writer<FrameData>>camera_writer) {
  std::vector<std::thread> threads;
  threads.push_back(start_process_thread(s, &s->road_cam, thumbnail_writer, camera_writer, process_road_camera));
  // threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));

  // std::thread t_rear = std::thread(road_camera_thread, &s->road_cam, cap_road);
  std::thread t_rear = std::thread(road_camera_thread, &s->road_cam);
  set_thread_name("mipi_thread");

  // std::thread t_front = std::thread(driver_camera_thread, &s->driver_cam);
  // set_thread_name("mipidriver_thread");
  
  for (auto &t : threads) t.join();
  // t_front.join();
  t_rear.join();

  cameras_close(s);
}


/*std::string gstreamer_pipeline(int sensor_mode, int sensor_id, int flip_method, int display_width, int display_height) {
    // sensor mode 1 = 1920 x 1080
    return "nvarguscamerasrc sensor_mode=" + std::to_string(sensor_mode) +
           " sensor_id=" + std::to_string(sensor_id) +
           " ! video/x-raw(memory:NVMM)" +
           ", format=(string)NV12" +
           ", framerate=(fraction)20/1" +
           ", width=(int)" + std::to_string(display_width) +
           ", height=(int)" + std::to_string(display_height) +
           " ! nvvidconv flip-method=" + std::to_string(flip_method) +
           " ! video/x-raw, format=BGRx" +
           " ! videoconvert ! video/x-raw, format=(string)BGR" +
           " ! appsink";
}*/

// void run_camera2(CameraState *s, cv::VideoCapture &video_cap, float *ts) {
//   assert(video_cap.isOpened());

//   cv::Size size(s->ci.frame_width, s->ci.frame_height);
//   const cv::Mat transform = cv::Mat(3, 3, CV_32F, ts);
//   uint32_t frame_id = 0;
//   size_t buf_idx = 0;

//   while (!do_exit) {
//     cv::Mat YUV_mat, frame_mat, transformed_mat;
    
//     video_cap >> YUV_mat;
//     // ar0233:
//     // COLOR_YUV2BGR_UYVY
//     cv::cvtColor(YUV_mat, frame_mat, cv::COLOR_YUV2BGR_UYVY);
//     // isx031:
//     // COLOR_YUV2BGR_YUY2
//     //cv::cvtColor(YUV_mat, frame_mat, cv::COLOR_YUV2BGR_YUY2);
    
//     cv::resize(frame_mat, transformed_mat, size);

//     s->buf.camera_bufs_metadata[buf_idx] = {.frame_id = frame_id};

//     auto &buf = s->buf.camera_bufs[buf_idx];
//     int transformed_size = transformed_mat.total() * transformed_mat.elemSize();

//     CL_CHECK(clEnqueueWriteBuffer(buf.copy_q, buf.buf_cl, CL_TRUE, 0, transformed_size, transformed_mat.data, 0, NULL, NULL));

//     s->buf.queue(buf_idx);

//     ++frame_id;
//     buf_idx = (buf_idx + 1) % FRAME_BUF_COUNT;

//   }
// }

// static void driver_camera_thread(CameraState *s) {
//   set_thread_name("mipi_driver_camera_thread");

//   // std::string pipeline = gstreamer_pipeline(
//   //   1, // sensor mode
//   //   0, // sensor id
//   //   2, // flip method
//   //   864, // width
//   //   486); // height

//   // camera-ar0233:
//   cv::VideoCapture cap_driver("v4l2src device=/dev/video2 ! video/x-raw, width=(int)1920, height=(int)1080,format=(string)YUY2, framerate=(fraction)30/1 ! appsink");
//   // camera-isx031:
//   //cv::VideoCapture cap_driver("v4l2src device=/dev/video2 ! video/x-raw, width=(int)1920, height=(int)1536,format=(string)YUY2, framerate=(fraction)30/1 ! appsink");
  
//   // camera-ar0233:
//   float ts[9] = {0.46414363, 0.0, 124.44721004,
//                   0.0, 0.46099291, 215.07801418,
//                   0.0, 0.0, 1.0};
//   // camera-isx031:
//   // float ts[9] = {0.76528467, 0.0, -151.52535531,
//   //                 0.0, 0.76086957, -155.7173913,
//   //                 0.0, 0.0, 1.0};
//   run_camera2(s, cap_driver, ts);
// }

// void process_driver_camera(MultiCameraState *s, CameraState *c, int cnt) {
//   const CameraBuf *b = &c->buf;
//   MessageBuilder msg;
//   auto framed = msg.initEvent().initDriverCameraState();
//   fill_frame_data(framed, b->cur_frame_data);
//   framed.setImage(kj::arrayPtr((const uint8_t *)b->cur_yuv_buf->addr, b->cur_yuv_buf->len));
//   framed.setTransform(b->yuv_transform.v);
//   s->pm->send("driverCameraState", msg);
// }
