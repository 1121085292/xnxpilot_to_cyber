#pragma once

#include <eigen3/Eigen/Dense>

#include "orientation.hpp"

#define model_height 1.22

Eigen::MatrixXd getViewFrameFromRoadFrame(double roll, double pitch, double yaw, double height);