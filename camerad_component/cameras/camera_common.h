#pragma once

#include <cstdint>
#include <cstdlib>
#include <memory>
#include <thread>
#include <map>
// #include "cereal/messaging/messaging.h"
// #include "xnxpilot/cereal/visionipc/visionbuf.h"
// #include "xnxpilot/cereal/visionipc/visionipc.h"
#include "camerad_component/visionipc/visionipc_server.h"
#include "camerad_component/transforms/rgb_to_yuv.h"
#include "camerad_component/common/mat.h"
#include "camerad_component/common/queue.h"
#include "camerad_component/common/modeldata.h"
// #include "xnxpilot/selfdrive/common/swaglog.h"
// #include "xnxpilot/selfdrive/common/visionimg.h"
#include "common_msgs/camerad/frame_data.pb.h"
#include "camerad_component/common/clutil.h"

// #define CAMERA_ID_IMX298 0
// #define CAMERA_ID_IMX179 1
// #define CAMERA_ID_S5K3P8SP 2
// #define CAMERA_ID_OV8865 3
// #define CAMERA_ID_IMX298_FLIPPED 4
// #define CAMERA_ID_OV10640 5
// #define CAMERA_ID_LGC920 6
// #define CAMERA_ID_LGC615 7
// #define CAMERA_ID_AR0231 8
// #define CAMERA_ID_IMX477 9
// #define CAMERA_ID_MAX 10

#define UI_BUF_COUNT 4

using common_msgs::camerad::Thumbnail;

enum CameraID {
  IMX298 = 0,
  IMX179,
  S5K3P8SP,
  OV8865,
  IMX298_FLIPPED,
  OV10640,
  LGC920,
  LGC615,
  AR0231,
  AR0233,
  IMX477
};

const std::map<std::string, CameraID> kCameraName2CameraId = {
  {"IMX298", CameraID::IMX298},
  {"IMX179", CameraID::IMX179},
  {"S5K3P8SP", CameraID::S5K3P8SP},
  {"OV8865", CameraID::OV8865},
  {"IMX298_FLIPPED", CameraID::IMX298_FLIPPED},
  {"OV10640", CameraID::OV10640},
  {"LGC920", CameraID::LGC920},
  {"LGC615", CameraID::LGC615},
  {"AR0231", CameraID::AR0231},
  {"AR0233", CameraID::AR0233},
  {"IMX477", CameraID::IMX477},
};

enum CameraType {
  RoadCam = 0,
  DriverCam,
  WideRoadCam
};

// const bool env_send_driver = getenv("SEND_DRIVER") != NULL;
// const bool env_send_road = getenv("SEND_ROAD") != NULL;
// const bool env_send_wide_road = getenv("SEND_WIDE_ROAD") != NULL;

// typedef void (*release_cb)(void *cookie, int buf_idx);

typedef struct CameraInfo {
  int frame_width, frame_height;
  int frame_stride;
  int frame_size;
  bool bayer;
  int bayer_flip;
  bool hdr;
} CameraInfo;

// typedef struct LogCameraInfo {
//   CameraType type;
//   const char* filename;
//   const char* frame_packet_name;
//   const char* encode_idx_name;
//   VisionStreamType stream_type;
//   int frame_width, frame_height;
//   int fps;
//   int bitrate;
//   bool is_h265;
//   bool downscale;
//   bool has_qcamera;
//   bool trigger_rotate;
//   bool enable;
// } LogCameraInfo;

typedef struct FrameMetadata {
  uint32_t frame_id;
  unsigned int frame_length;

  // Timestamps
  uint64_t timestamp_sof; // only set on tici
  uint64_t timestamp_eof;

  // Exposure
  unsigned int integ_lines;
  bool high_conversion_gain;
  float gain;
  float measured_grey_fraction;
  float target_grey_fraction;

  // Focus
  unsigned int lens_pos;
  float lens_sag;
  float lens_err;
  float lens_true_pos;
} FrameMetadata;

typedef struct CameraExpInfo {
  int op_id;
  float grey_frac;
} CameraExpInfo;

// struct MultiCameraState;
struct CameraState;

class CameraBuf {
private:
  VisionIpcServer *vipc_server;
  // CameraState *camera_state;
  std::shared_ptr<CameraState> camera_state;
  cl_kernel krnl_debayer;

  std::unique_ptr<Rgb2Yuv> rgb2yuv;

  VisionStreamType rgb_type, yuv_type;

  int cur_buf_idx;

  SafeQueue<int> safe_queue;

  int frame_buf_count;
  // release_cb release_callback;

public:
  cl_command_queue q;
  FrameMetadata cur_frame_data;
  VisionBuf *cur_rgb_buf;
  VisionBuf *cur_yuv_buf;
  std::unique_ptr<VisionBuf[]> camera_bufs;
  std::unique_ptr<FrameMetadata[]> camera_bufs_metadata;
  int rgb_width, rgb_height, rgb_stride;

  mat3 yuv_transform;

  // CameraBuf() = default;
  ~CameraBuf();
  void init(cl_device_id device_id, cl_context context, std::shared_ptr<CameraState>& s, VisionIpcServer * v, 
              int frame_cnt, VisionStreamType rgb_type, VisionStreamType yuv_type);
  bool acquire();
  // void release();
  void queue(size_t buf_idx);
};

typedef struct CameraState {
  CameraInfo ci;
  int camera_num;
  int fps;
  float digital_gain;
  CameraBuf buf;
  CameraType camera_type;
} CameraState;


typedef struct MultiCameraState {
  CameraState road_cam;
  CameraState driver_cam;

  // SubMaster *sm;
  // PubMaster *pm;
} MultiCameraState;

// typedef void (*process_thread_cb)(MultiCameraState *s, CameraState *c, int cnt);

// void fill_frame_data(cereal::FrameData::Builder &framed, const FrameMetadata &frame_data);
// kj::Array<uint8_t> get_frame_image(const CameraBuf *b);
// float set_exposure_target(const CameraBuf *b, int x_start, int x_end, int x_skip, int y_start, int y_end, int y_skip);
// std::thread start_process_thread(MultiCameraState *cameras, CameraState *cs, process_thread_cb callback);
// void common_process_driver_camera(SubMaster *sm, PubMaster *pm, CameraState *c, int cnt);

// void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx);
// void cameras_open(MultiCameraState *s);
// void cameras_run(MultiCameraState *s);
// void cameras_close(MultiCameraState *s);
// void camera_autoexposure(CameraState *s, float grey_frac);
// void processing_thread(MultiCameraState *cameras, CameraState *cs);
// void publish_thumbnail(PubMaster *pm, const CameraBuf *b);
void publish_thumbnail(std::shared_ptr<apollo::cyber::Writer<Thumbnail>>& writer, const CameraBuf *b);
