#pragma once

#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <atomic>
#include <future>

#include <eigen3/Eigen/Dense>

#include "cyber/cyber.h"
// #include "cereal/messaging/messaging.h"
#include "camerad_component/visionipc/visionipc_client.h"
#include "camerad_component/common/clutil.h"
// #include "camerad_component/common/params.h"
#include "camerad_component/common/swaglog.h"
#include "camerad_component/common/util.h"
#include "camerad_component/hardware/hw.h"
#include "modeld_component/models/driving.h"

#include "common_msgs/plannerd/lateral_plan.pb.h"
#include "common_msgs/modeld/camera_odometry.pb.h"
#include "common_msgs/modeld/model_data_v2.pb.h"
#include "common_msgs/camerad/frame_data.pb.h"
#include "common_msgs/calibrationd/live_calibration_data.pb.h"

using apollo::cyber::Component;
using common_msgs::calibration::LiveCalibrationData;
using common_msgs::camera_odometry::CameraOdometry;
using common_msgs::lateral_plan::LateralPlan;
using common_msgs::model_data_v2::ModelDataV2;
using common_msgs::camerad::FrameData;

class ModeldComponent : public Component<FrameData, LateralPlan> {
  public:
    bool Init() override;
    bool Proc(const std::shared_ptr<FrameData>& frame_data,
              const std::shared_ptr<LateralPlan>& lateral_plan) override;

    ~ModeldComponent(){
      res_.wait();
    }
  private:
    void calibration_thread();
    void model_publish(uint32_t vipc_frame_id, uint32_t frame_id, float frame_drop,
                   const ModelDataRaw &net_outputs, uint64_t timestamp_eof,
                   float model_execution_time, const std::vector<float> raw_pred);

    void posenet_publish(uint32_t vipc_frame_id, uint32_t vipc_dropped_frames,
                     const ModelDataRaw &net_outputs, uint64_t timestamp_eof);
    
    std::shared_ptr<apollo::cyber::Writer<CameraOdometry>> camera_odometry_writer_;
    std::shared_ptr<apollo::cyber::Writer<ModelDataV2>> model_data_v2_writer_;

    std::shared_ptr<apollo::cyber::Reader<LiveCalibrationData>> calibration_reader_;

    std::shared_ptr<LiveCalibrationData> calibration_;
    bool wide_camera_;
    std::shared_ptr<ModelState> model_;
    mat3 cur_transform_;
    bool live_calib_seen_;
    std::mutex mutex_;

    Params params_;
    VisionIpcClient vipc_client_;
    cl_device_id device_id_;
    cl_context context_;
    std::future<void> res_;
    std::atomic<bool> running_ = {false};

    FirstOrderFilter frame_dropped_filter_;
    uint32_t frame_id_;
    uint32_t last_vipc_frame_id_;
    double last_;
    uint32_t run_count_;
};
CYBER_REGISTER_COMPONENT(ModeldComponent)
