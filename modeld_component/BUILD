load("//tools/install:install.bzl", "install", "install_src_files")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_binary(
    name = "libmodeld_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":modeld_component_lib",
    ]
)

cc_library(
    name = "modeld_component_lib",
    srcs = ["modeld_component.cc"],
    hdrs = ["modeld_component.h"],
    alwayslink = True,
    copts = ["-std=c++17"],
    deps = [
        # "//cyber",
        "//perception/modeld_component/models:driving",
        "//perception/camerad_component/visionipc:visionipc_client",
        "//common_msgs/camerad:frame_data_cc_proto",
        "//common_msgs/calibrationd:live_calibration_data_cc_proto",
        "//common_msgs/modeld:camera_odometry_cc_proto",
        "//common_msgs/modeld:model_data_v2_cc_proto",
        "//common_msgs/plannerd:lateral_plan_cc_proto",
    ]
)
