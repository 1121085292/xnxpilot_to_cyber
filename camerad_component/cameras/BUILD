load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_mipi",
    srcs = ["camera_mipi.cc"],
    hdrs = ["camera_mipi.h"],
    copts = ["-std=c++17", "-pthread"],
    deps = [
        ":camera_common",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:videoio",
    ]
)

cc_library(
    name = "camera_common",
    srcs = ["camera_common.cc"],
    hdrs = ["camera_common.h"],
    copts = ["-std=c++17", "-pthread"],
    deps = [
        "//cyber",
        "//common_msgs/camerad:frame_data_cc_proto",
        # "//camerad_component/imgproc:utils",
        "//perception/camerad_component/transforms:rgb_to_yuv",
        "//perception/camerad_component/visionipc:visionipc_server",
        "//perception/camerad_component/common:common",
        "//perception/camerad_component/common:util",
        "@xnxpilot//:jpeglib",
        "@xnxpilot//:libyuv",
    ]
)
