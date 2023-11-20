load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_common",
    srcs = ["camera_common.cc"],
    hdrs = ["camera_common.h"],
    copts = ["-std=c++17"],
    deps = [
        "//cyber",
        "//common_msgs/camerad:frame_data_cc_proto",
        # "//camerad_component/imgproc:utils",
        "//camerad_component/transforms:rgb_to_yuv",
        "//camerad_component/visionipc:visionipc_server",
        "//camerad_component/common:common",
        "@xnxpilot//:jpeglib",
    ]
)