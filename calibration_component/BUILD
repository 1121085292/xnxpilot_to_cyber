load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "libcalibration_component.so",
  linkshared = True,
  linkstatic = True,
  deps = [
    ":calibration_component_lib",
  ]
)

cc_library(
  name = "calibration_component_lib",
  srcs = ["calibration_component.cc"],
  hdrs = ["calibration_component.h"],
  alwayslink = True,
  copts = ["-std=c++17"],
  deps = [
    ":record_cal",
    "//cyber",
    "//common_msgs/calibrationd:live_calibration_data_cc_proto",
    "//calibration_component/transformations:camera",
    "//camerad_component/common:params",
    "//common_msgs/modeld:camera_odometry_cc_proto",
    "//common_msgs/plannerd:car_state_cc_proto",
  ]
)

cc_library(
  name = "record_cal",
  srcs = ["record_cal.cc"],
  hdrs = ["record_cal.h"],
  alwayslink = True, 
  deps = [
    "//cyber",
    "//common_msgs/calibrationd:live_calibration_data_cc_proto",
  ] 
)