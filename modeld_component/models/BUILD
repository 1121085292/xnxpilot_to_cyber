load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "driving",
    srcs = ["driving.cc"],
    hdrs = ["driving.h"],
    copts = ["-std=c++17"],
    deps = [
        "//cyber",
        "//common_msgs/modeld:model_data_v2_cc_proto",
        "//modeld_component/runners:onnx_model",
        ":common_model",
        "//camerad_component/messaging:messaging",
    ]
)

cc_library(
    name = "common_model",
    srcs = ["commonmodel.cc"],
    hdrs = ["commonmodel.h"],
    copts = ["-std=c++17"],
    deps = [
        "//modeld_component/transforms:loadyuv",
        "//modeld_component/transforms:transform",
    ]
)