#include "modeld_component.h"

bool ModeldComponent::Init()
{
  camera_odometry_writer_ = node_->CreateWriter<CameraOdometry>("CameraOdometry");
  model_data_v2_writer_ = node_->CreateWriter<ModelDataV2>("ModelDataV2"); 

  calibration_ = std::make_shared<LiveCalibrationData>();
  calibration_reader_ = node_->CreateReader<LiveCalibrationData>(
      "liveCalibration", [this](const std::shared_ptr<LiveCalibrationData>& calibration){
        std::lock_guard<std::mutex> lock(mutex_);
        calibration_->CopyFrom(*calibration);
      });
  
  wide_camera_ = Hardware::TICI() ? params_.getBool("EnableWideCamera") : false;

  res_ = apollo::cyber::Async(&ModeldComponent::calibration_thread, this);

  device_id_ = cl_get_device_id(CL_DEVICE_TYPE_GPU);
  context_ = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id_, NULL, NULL, &err));

  model_ = std::make_shared<ModelState>();
  // camera init
  model_init(model_, device_id_, context_);
  AINFO << "models loaded, modeld starting";

  vipc_client_ = VisionIpcClient("camerad", wide_camera_ ? VISION_STREAM_YUV_WIDE : VISION_STREAM_YUV_BACK, true, device_id_, context_);
  while (!vipc_client_.connect(false))
  {
    util::sleep_for(100);
  }
  
  // frame_dropped_filter_.setFirstOrderFilter(0., 10., 1. / MODEL_FREQ);
  frame_dropped_filter_ = FirstOrderFilter(0., 10., 1. / MODEL_FREQ);
  frame_id_ = 0;
  last_vipc_frame_id_ = 0;
  last_ = 0.0;
  run_count_ = 0;

  return true;
}

bool ModeldComponent::Proc(const std::shared_ptr<FrameData> &frame_data, 
                            const std::shared_ptr<LateralPlan> &lateral_plan) {
  if(!vipc_client_.connected){
    apollo::cyber::Rate rate(10.0);
    rate.Sleep();
  } else {
    const VisionBuf *b = &vipc_client_.buffers[0];
    AINFO << "connected with buffer size:" << b->len << "(" << b->width << "X" << b->height <<")";
    VisionIpcBufExtra extra = {};
    VisionBuf *buf = vipc_client_.recv(&extra);

    mutex_.lock();
    mat3 model_transform = cur_transform_;
    const bool run_model_this_iter = live_calib_seen_;
    mutex_.unlock();

    int desire = lateral_plan->desire();
    frame_id_ = frame_data->frame_id();

    if (run_model_this_iter) {
      run_count_++;

      float vec_desire[DESIRE_LEN] = {0};
      if (desire >= 0 && desire < DESIRE_LEN) {
        vec_desire[desire] = 1.0;
      }

      double mt1 = millis_since_boot();
      ModelDataRaw model_buf = model_eval_frame(model_, buf->buf_cl, buf->width, buf->height,
                                                model_transform, vec_desire);
      double mt2 = millis_since_boot();
      float model_execution_time = (mt2 - mt1) / 1000.0;

      // tracked dropped frames
      // frame_dropped_filter_ = FirstOrderFilter(0., 10., 1. / MODEL_FREQ);

      uint32_t vipc_dropped_frames = extra.frame_id - last_vipc_frame_id_ - 1;
      float frames_dropped = frame_dropped_filter_.update((float)std::min(vipc_dropped_frames, 10U));
      if (run_count_ < 10) { // let frame drops warm up
        frame_dropped_filter_.reset(0);
        frames_dropped = 0.;
      }

      float frame_drop_ratio = frames_dropped / (1 + frames_dropped);

      model_publish(extra.frame_id, frame_id_, frame_drop_ratio, model_buf, 
                    extra.timestamp_eof, model_execution_time, model_->output);
      posenet_publish(extra.frame_id, vipc_dropped_frames, model_buf, extra.timestamp_eof);

      //printf("model process: %.2fms, from last %.2fms, vipc_frame_id %u, frame_id, %u, frame_drop %.3f\n", mt2 - mt1, mt1 - last, extra.frame_id, frame_id, frame_drop_ratio);
      last_ = mt1;
      last_vipc_frame_id_ = extra.frame_id;
    }
  }
  return true;
}

void ModeldComponent::calibration_thread() {
  running_.exchange(true);
  /*
     import numpy as np
     from common.transformations.model import medmodel_frame_from_road_frame
     medmodel_frame_from_ground = medmodel_frame_from_road_frame[:, (0, 1, 3)]
     ground_from_medmodel_frame = np.linalg.inv(medmodel_frame_from_ground)
  */
  Eigen::Matrix<float, 3, 3> ground_from_medmodel_frame;
  ground_from_medmodel_frame <<
    0.00000000e+00, 0.00000000e+00, 1.00000000e+00,
    -1.09890110e-03, 0.00000000e+00, 2.81318681e-01,
    -1.84808520e-20, 9.00738606e-04,-4.28751576e-02;

  Eigen::Matrix<float, 3, 3> cam_intrinsics = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>(
                                      wide_camera_ ? ecam_intrinsic_matrix.v : fcam_intrinsic_matrix.v);
  const mat3 yuv_transform = get_model_yuv_transform();

  while (!apollo::cyber::IsShutdown()) {
    auto extrinsic_matrix = calibration_->extrinsic_matrix();
    Eigen::Matrix<float, 3, 4> extrinsic_matrix_eigen;
    for (int i = 0; i < 4*3; i++) {
      extrinsic_matrix_eigen(i / 4, i % 4) = extrinsic_matrix[i];
    }

    auto camera_frame_from_road_frame = cam_intrinsics * extrinsic_matrix_eigen;
    Eigen::Matrix<float, 3, 3> camera_frame_from_ground;
    camera_frame_from_ground.col(0) = camera_frame_from_road_frame.col(0);
    camera_frame_from_ground.col(1) = camera_frame_from_road_frame.col(1);
    camera_frame_from_ground.col(2) = camera_frame_from_road_frame.col(3);

    auto warp_matrix = camera_frame_from_ground * ground_from_medmodel_frame;
    mat3 transform = {};
    for (int i=0; i<3*3; i++) {
      transform.v[i] = warp_matrix(i / 3, i % 3);
    }
    mat3 model_transform = matmul3(yuv_transform, transform);
    std::lock_guard<std::mutex> lk(mutex_);
    cur_transform_ = model_transform;
    live_calib_seen_ = true;
  }
}
// publish ModelDataV2s
void ModeldComponent::model_publish(uint32_t vipc_frame_id, uint32_t frame_id, float frame_drop,
                                    const ModelDataRaw &net_outputs, uint64_t timestamp_eof,
                                    float model_execution_time, const std::vector<float> raw_pred) {
  const uint32_t frame_age = (frame_id > vipc_frame_id) ? (frame_id - vipc_frame_id) : 0;
  auto out_msg = std::make_shared<ModelDataV2>();
  out_msg->set_error_code(apollo::common::ErrorCode::OK);
  out_msg->set_frame_id(vipc_frame_id);
  out_msg->set_frame_age(frame_age);
  out_msg->set_frame_drop_perc(frame_drop * 100);
  out_msg->set_timestamp_eof(timestamp_eof);
  out_msg->set_model_execution_time(model_execution_time);
  if (send_raw_pred) {
    out_msg->set_raw_predictions(&(*raw_pred.begin()), raw_pred.size());
  }
  fill_model(out_msg, net_outputs);
  model_data_v2_writer_->Write(out_msg);
}
// publish CameraOdometry
void ModeldComponent::posenet_publish(uint32_t vipc_frame_id, uint32_t vipc_dropped_frames,
                                      const ModelDataRaw &net_outputs, uint64_t timestamp_eof) {
  auto out_msg = std::make_shared<CameraOdometry>();
  if(vipc_dropped_frames < 1){
    out_msg->set_error_code(apollo::common::ErrorCode::OK);
  }
  
  for (int i =0; i < 3; i++) {
    out_msg->add_trans(net_outputs.pose[i]);
    out_msg->add_trans_std(exp(net_outputs.pose[6 + i]));

    out_msg->add_rot(net_outputs.pose[3 + i]);
    out_msg->add_rot_std(exp(net_outputs.pose[9 + i]));
  }
  out_msg->set_timestamp_eof(timestamp_eof);
  out_msg->set_frame_id(vipc_frame_id);

  camera_odometry_writer_->Write(out_msg);
}

ModeldComponent::~ModeldComponent(){
  if(running_.load()){
    running_.exchange(false);
    res_.wait();
  }
}
