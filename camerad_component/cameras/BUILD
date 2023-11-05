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
        # "//xnxpilot/selfdrive/camerad/imgproc:utils",
        # "//xnxpilot/selfdrive/camerad/transforms:rgb_to_yuv",
        # "//xnxpilot/cereal/visionipc:visionipc_server",
        # "//xnxpilot/third_party/libyuv:libyuv",   
        # "//xnxpilot/third_party/jpeglib:jpeglib",
        # "@libyuv",
        # "@jpeglib",
        "//camerad_component/common:clutil",
        "@xnxpilot//:opencl",
    ]
)