#pragma once

#include <eigen3/Eigen/Dense>

#include "record_cal.h"
#include "perception/camerad_component/hardware/hw.h"
#include "perception/calibration_component/transformations/camera.h"
#include "common_msgs/plannerd/car_state.pb.h"
#include "common_msgs/calibrationd/live_calibration_data.pb.h"
#include "common_msgs/modeld/camera_odometry.pb.h"

#define INPUTS_NEEDED 5
#define INPUTS_WANTED 50
#define BLOCK_SIZE 100
#define SMOOTH_CYCLES 400

#define MPH_TO_KPH 1.609344
#define MS_TO_KPH 3.6
#define KPH_TO_MS 1. / MS_TO_KPH
#define MPH_TO_MS MPH_TO_KPH * KPH_TO_MS
#define MIN_SPEED_FILTER 15 * MPH_TO_MS

const Eigen::Vector3d RPY_INIT = { 0.0, 0.0, 0.0 };
const double PITCH_LIMITS[] = { -1.5, 1.5 };
const double YAW_LIMITS[] = { -1.5, 1.5 };

#define MAX_ALLOWED_SPREAD 2 * 180.0 / M_PI
#define MAX_YAW_RATE_FILTER 2 * 180.0 / M_PI
#define MAX_VEL_ANGLE_STD 0.25 * 180.0 / M_PI

using apollo::cyber::Component;
using common_msgs::calibration::LiveCalibrationData;
using common_msgs::car_state::CarState;
using common_msgs::camera_odometry::CameraOdometry;

enum Calibration {
  UNCALIBRATED = 0,
  CALIBRATED = 1,
  INVALID = 2,
};

bool isCalibrationValid(Eigen::Vector3d& rpy);

Eigen::Vector3d sanityClip(Eigen::Vector3d& rpy);

class CalibrationComponent : public Component<CarState>{
  public:
    bool Init() override;
    bool Proc(const std::shared_ptr<CarState>& car_state) override;

  private:
    void reset(Eigen::Vector3d rpy_init = RPY_INIT, int valid_blocks = 0,
                  Eigen::Vector3d smooth_from = {});
    void updateStatus();
    std::shared_ptr<LiveCalibrationData> getMsg();
    Eigen::Vector3d getSmoothRpy();
    /// @brief 
    /// @param cam_odom 
    /// @return 
    Eigen::Vector3d handleCamOdom(const std::shared_ptr<CameraOdometry>& cam_odom);

    bool param_put_ = false;
    Params params_;
    std::string calibration_params_;
    bool wide_camera_;
    int valid_blocks_;

    int idx_;
    int block_idx_;
    float v_ego_;
    double old_rpy_weight_;

    int frame_;

    Eigen::Vector3d rpy_init_;
    Eigen::Vector3d rpy_;
    Eigen::Vector3d old_rpy_;
    Eigen::Vector3d smooth_from_;
    Eigen::MatrixXd rpys_;

    Eigen::Vector3d calib_spread_;
    int cal_status_;

    std::mutex mutex_;
    std::shared_ptr<LiveCalibrationData> calibration_;
    std::shared_ptr<CameraOdometry> cam_odometry_;
    std::shared_ptr<apollo::cyber::Writer<LiveCalibrationData>> writer_ = nullptr;
    std::shared_ptr<apollo::cyber::Reader<CameraOdometry>> reader_ = nullptr;
};

CYBER_REGISTER_COMPONENT(CalibrationComponent);