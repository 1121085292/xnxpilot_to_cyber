#include "calibration_component.h"

using apollo::cyber::ComponentBase;
bool isCalibrationValid(Eigen::Vector3d& rpy)
{
  return ((PITCH_LIMITS[0] < rpy[1]) && (rpy[1] < PITCH_LIMITS[1]))
      && ((YAW_LIMITS[0] < rpy[2]) && (rpy[2] < YAW_LIMITS[1]));
}

Eigen::Vector3d sanityClip(Eigen::Vector3d& rpy)
{
  if(rpy.hasNaN()){
    return RPY_INIT;
  }
  double clipped_pitch = std::max(PITCH_LIMITS[0] - 0.005, std::min(rpy[1], PITCH_LIMITS[1] + 0.005));
  double clipped_yaw = std::max(YAW_LIMITS[0] - 0.005, std::min(rpy[2], YAW_LIMITS[1] + 0.005));
  return { rpy[0], clipped_pitch, clipped_yaw };
}

bool CalibrationComponent::Init()
{
  writer_ = node_->CreateWriter<LiveCalibrationData>("liveCalibration");
  // read saved calibration
  calibration_params_ = calRecordReader("calibration_component/CalibrationParams");

  wide_camera_ = Hardware::TICI ? params_.getBool("EnableWideCamera") : false;
  Eigen::Vector3d rpy_init = RPY_INIT;
  int valid_blocks = 0;
  param_put_ = true;

  if(param_put_ && !calibration_params_.empty()){
    try
    {
      calibration_ = std::make_shared<LiveCalibrationData>();
      calibration_->ParseFromString(calibration_params_);
      for(int i = 0; i < calibration_->rpy_calib().size(); ++i){
        rpy_init[i] = calibration_->rpy_calib(i);
      }
      valid_blocks = calibration_->valid_blocks();
    } catch(const std::exception& e) 
    {
      AINFO << "Error reading cached CalibrationParams";
    }
  }
  reset(rpy_init, valid_blocks);
  updateStatus(); 
  return true;
}

bool CalibrationComponent::Proc(
    const std::shared_ptr<CameraOdometry>& cam_odometry, 
    const std::shared_ptr<CarState>& car_state)
{
  if(apollo::cyber::OK()){
    v_ego_ = car_state->v_ego();
  }
  auto new_rpy = handleCamOdom(cam_odometry);
  AINFO << "got new rpy :" << new_rpy;
  // 4Hz
  if(cam_odometry->frame_id() % 5 == 0){
    writer_->Write(this->getMsg());
  }
  return true;
}

void CalibrationComponent::reset(Eigen::Vector3d rpy_init, int valid_blocks,
                                 Eigen::Vector3d smooth_from)
{
  // 判断rpy_init是否有非有限值
  if(!rpy_init.allFinite()){
    rpy_ = RPY_INIT;
  } else {
    rpy_ = rpy_init;
  }
  // 判断valid_blocks是否有非有限值
  if(!std::isfinite(valid_blocks) || valid_blocks < 0){
    valid_blocks_ = 0;
  } else {
    valid_blocks_ = valid_blocks;
  }
  // 初始化rpys_
  rpys_ = rpy_.replicate(INPUTS_WANTED, 1);

  idx_ = 0;
  block_idx_ = 0;
  v_ego_ = 0.0f;

  if(smooth_from.size() == 0){
    old_rpy_ = RPY_INIT;
    old_rpy_weight_ = 0.0;
  } else {
    old_rpy_ = smooth_from;
    old_rpy_weight_ = 1.0;
  }
}

void CalibrationComponent::updateStatus()
{
  if(valid_blocks_ > 0){
    Eigen::Vector3d max_rpy_calib = rpys_.topRows(valid_blocks_).colwise().maxCoeff();
    Eigen::Vector3d min_rpy_calib = rpys_.topRows(valid_blocks_).colwise().minCoeff();
    calib_spread_ = (max_rpy_calib - min_rpy_calib).cwiseAbs();
  } else {
    calib_spread_.setZero();
  }

  if(valid_blocks_ < INPUTS_NEEDED){
    cal_status_ = Calibration::UNCALIBRATED;
  } else if (isCalibrationValid(rpy_)){
    cal_status_ = Calibration::CALIBRATED;
  } else {
    cal_status_ = Calibration::INVALID;
  }

  // If spread is too high, assume mounting was changed and reset to last block.
  // Make the transition smooth. Abrupt transitions are not good foor feedback loop through supercombo model.
  if(calib_spread_.maxCoeff() > MAX_ALLOWED_SPEEDAD && cal_status_ == Calibration::CALIBRATED){
    reset(rpys_.row(block_idx_ - 1), INPUTS_NEEDED, rpy_);
  }
  
  bool write_this_cycle = (idx_ == 0) && (block_idx_ % (INPUTS_WANTED / 5) == 5);
  if(param_put_ && write_this_cycle){
    calRecordWriter("calibration_component/CalibrationParams", getMsg().get());
  }
}

std::shared_ptr<LiveCalibrationData> CalibrationComponent::getMsg()
{
  Eigen::Vector3d smooth_rpy = getSmoothRpy();
  auto extrinsic_matrix = getViewFrameFromRoadFrame(0, smooth_from_[1], smooth_from_[2], model_height);

  auto out_msg = std::make_shared<LiveCalibrationData>();
  out_msg->set_valid_blocks(valid_blocks_);
  out_msg->set_cal_status(cal_status_);
  out_msg->set_cal_perc(std::min(100 * (valid_blocks_ * BLOCK_SIZE + idx_) / (INPUTS_NEEDED * BLOCK_SIZE), 100));

  // 将矩阵展平为一维数组
  Eigen::Map<Eigen::VectorXd> flattened(extrinsic_matrix.data(), extrinsic_matrix.size());
  // 创建一个包含展平后所有元素的向量
  Eigen::VectorXd extrinsic_vector = flattened;
  for(int i = 0; i < extrinsic_vector.size(); ++i){
    out_msg->add_extrinsic_matrix(static_cast<float>(extrinsic_vector[i]));
  }

  // 将smooth_rpy中的元素转成float，存储到消息中
  for(int i = 0; i < smooth_rpy.size(); ++i){
    out_msg->add_rpy_calib(static_cast<float>(smooth_rpy[i]));
  }  

  for(int i = 0; i < calib_spread_.size(); ++i){
    out_msg->add_rpy_calib_spread(static_cast<float>(calib_spread_[i]));
  }
  return out_msg;
}

Eigen::Vector3d CalibrationComponent::getSmoothRpy()
{
  if(old_rpy_weight_ > 0){
    return old_rpy_weight_ * old_rpy_ + (1.0 - old_rpy_weight_) * rpy_;
  }
  return rpy_;
}

Eigen::Vector3d CalibrationComponent::handleCamOdom(
    const std::shared_ptr<CameraOdometry>& cam_odom)
{
  old_rpy_weight_ = std::min(0.0, old_rpy_weight_ - 1 / SMOOTH_CYCLES);
  bool straight_and_fast = (v_ego_ > MIN_SPEED_FILTER) && (cam_odom->trans(0) > MIN_SPEED_FILTER)
                        && (std::abs(cam_odom->rot(2) < MAX_YAW_RATE_FILTER));

  double angle_std_threshold = wide_camera_ ? (4 * MAX_VEL_ANGLE_STD) : MAX_VEL_ANGLE_STD;
  bool certain_if_calib = (std::atan2(cam_odom->trans(1), cam_odom->trans(0)) < angle_std_threshold)
                      || (valid_blocks_ < INPUTS_NEEDED);
  if(!(straight_and_fast && certain_if_calib)){
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d observed_rpy(0,
                               -std::atan2(cam_odom->trans(2), cam_odom->trans(0)),
                               std::atan2(cam_odom->trans(1), cam_odom->trans(0)));

  Eigen::Vector3d new_rpy = rot2euler(euler2rot(smooth_from_) * euler2rot(observed_rpy));
  new_rpy = sanityClid(new_rpy);

  rpys_.block(block_idx_, 0, BLOCK_SIZE, 3) = (idx_ * rpys_block(block_idx_, 0, BLOCK_SIZE, 3)) + 
                        (BLOCK_SIZE - idx_) * new_rpy.transpose() / (static_cast<double>(BLOCK_SIZE));
  idx_ = (idx_ + 1) % BLOCK_SIZE;
  if(idx_ == 0) {
    block_idx_++;
    valid_blocks_ = std::max(block_idx_, valid_blocks_);
    block_idx_ = block_idx_ % INPUTS_WANTED;
  }
  if(valid_blocks_ > 0){
    rpy_ = rpys_.topRows(valid_blocks_).colWise().mean();
  }
  this->updateStatus();
  return new_rpy;
}
