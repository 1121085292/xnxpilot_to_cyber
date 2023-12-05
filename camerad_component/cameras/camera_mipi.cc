#include "camera_mipi.h"

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

#include "perception/camerad_component/common/clutil.h"
#include "perception/camerad_component/common/swaglog.h"
#include "perception/camerad_component/common/timing.h"
#include "perception/camerad_component/common/util.h"

extern ExitHandler do_exit;

namespace {


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

void run_camera2(CameraState *s, cv::VideoCapture &video_cap, float *ts) {
  assert(video_cap.isOpened());

  cv::Size size(s->ci.frame_width, s->ci.frame_height);
  const cv::Mat transform = cv::Mat(3, 3, CV_32F, ts);
  uint32_t frame_id = 0;
  size_t buf_idx = 0;

  while (!do_exit) {
    cv::Mat YUV_mat, frame_mat, transformed_mat;
    
    video_cap >> YUV_mat;
    // ar0233:
    // COLOR_YUV2BGR_UYVY
    cv::cvtColor(YUV_mat, frame_mat, cv::COLOR_YUV2BGR_UYVY);
    // isx031:
    // COLOR_YUV2BGR_YUY2
    //cv::cvtColor(YUV_mat, frame_mat, cv::COLOR_YUV2BGR_YUY2);
    
    cv::resize(frame_mat, transformed_mat, size);

    s->buf.camera_bufs_metadata[buf_idx] = {.frame_id = frame_id};

    auto &buf = s->buf.camera_bufs[buf_idx];
    int transformed_size = transformed_mat.total() * transformed_mat.elemSize();

    CL_CHECK(clEnqueueWriteBuffer(buf.copy_q, buf.buf_cl, CL_TRUE, 0, transformed_size, transformed_mat.data, 0, NULL, NULL));

    s->buf.queue(buf_idx);

    ++frame_id;
    buf_idx = (buf_idx + 1) % FRAME_BUF_COUNT;

  }
}




static void driver_camera_thread(CameraState *s) {
  set_thread_name("mipi_driver_camera_thread");

  // std::string pipeline = gstreamer_pipeline(
  //   1, // sensor mode
  //   0, // sensor id
  //   2, // flip method
  //   864, // width
  //   486); // height

  // camera-ar0233:
  cv::VideoCapture cap_driver("v4l2src device=/dev/video2 ! video/x-raw, width=(int)1920, height=(int)1080,format=(string)YUY2, framerate=(fraction)30/1 ! appsink");
  // camera-isx031:
  //cv::VideoCapture cap_driver("v4l2src device=/dev/video2 ! video/x-raw, width=(int)1920, height=(int)1536,format=(string)YUY2, framerate=(fraction)30/1 ! appsink");
  
  // camera-ar0233:
  float ts[9] = {0.46414363, 0.0, 124.44721004,
                  0.0, 0.46099291, 215.07801418,
                  0.0, 0.0, 1.0};
  // camera-isx031:
  // float ts[9] = {0.76528467, 0.0, -151.52535531,
  //                 0.0, 0.76086957, -155.7173913,
  //                 0.0, 0.0, 1.0};
  run_camera2(s, cap_driver, ts);
}

}  // namespace



void camera_autoexposure(CameraState *s, float grey_frac) {}



// void process_driver_camera(MultiCameraState *s, CameraState *c, int cnt) {
//   const CameraBuf *b = &c->buf;
//   MessageBuilder msg;
//   auto framed = msg.initEvent().initDriverCameraState();
//   fill_frame_data(framed, b->cur_frame_data);
//   framed.setImage(kj::arrayPtr((const uint8_t *)b->cur_yuv_buf->addr, b->cur_yuv_buf->len));
//   framed.setTransform(b->yuv_transform.v);
//   s->pm->send("driverCameraState", msg);
// }